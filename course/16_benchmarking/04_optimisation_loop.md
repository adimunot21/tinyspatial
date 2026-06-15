# The optimisation loop

The governing rule is simple: don't optimise blind, and benchmark before
touching the hot path. This chapter is the corresponding discipline: when
optimisation is warranted, here is the loop to run.

## The four steps

1. **Measure** — what does the code do *today*?
2. **Hypothesise** — what is the specific change expected to help,
   and *why*?
3. **Change** — make the smallest possible diff that tests the hypothesis.
4. **Re-measure** — did it move the number? Is it still correct?

Then commit (with the before/after numbers in the commit message) and
move to the next hypothesis.

## A worked example: `compute_joint_jacobians`

This is the optimisation we shipped in Phase 9a. Let's walk through it.

### Step 1 — Measure

Add `bench_kinematics.cpp` with a row for `compute_joint_jacobians`:

```cpp
void bench_joint_jacobians_impl(benchmark::State& state, const char* urdf) {
  ModelKit kit(fixture(urdf));
  std::vector<tinyspatial::Matrix6X> jacobians;
  for (auto _ : state) {
    tinyspatial::compute_joint_jacobians(kit.model, kit.data, kit.q, jacobians);
    benchmark::DoNotOptimize(jacobians);
  }
}
BENCHMARK_CAPTURE(bench_joint_jacobians_impl, franka_fr3, "franka_fr3.urdf");
```

Run:

```
bench_joint_jacobians_impl/franka_fr3   1881 ns   1881 ns   369249  items_per_second=531.7k/s
```

1.88 μs per call on Franka.

### Step 2 — Hypothesise

Open `include/tinyspatial/diff/fk_derivatives.hpp`:

```cpp
inline void compute_joint_jacobians(const Model& model, Data& data,
                                    const Eigen::Ref<const VectorX>& q,
                                    std::vector<Matrix6X>& joint_jacobians) {
  forward_kinematics(model, data, q);

  const int njoints = model.njoints();
  const int nv_total = model.nv();
  joint_jacobians.assign(njoints, Matrix6X::Zero(6, nv_total));   // <-- HERE
  // ...
}
```

`std::vector::assign(N, Matrix6X::Zero(6, nv))` reconstructs the
vector: it heap-allocates a fresh `Matrix6X` for every slot, copies
zeros into it, and tears down the old contents. **Even when the
caller passes in a correctly-sized vector from a previous call.**

A hot-loop user (e.g. analytical-derivative inner sweeps) calls this
function thousands of times with the same buffer. Each call pays for
`njoints` heap allocations + `njoints` deallocations from the previous
call's contents.

**Hypothesis:** "If the caller's vector is already sized correctly, we
should reuse the existing storage and just `setZero()` per element.
Estimated lift: 5–10% on Franka, since allocation/free of seven 6×7
doubles is roughly an order of magnitude of the inner-loop work."

### Step 3 — Change

The smallest diff that tests the hypothesis:

```cpp
if (static_cast<int>(joint_jacobians.size()) != njoints) {
  joint_jacobians.assign(njoints, Matrix6X::Zero(6, nv_total));
} else {
  for (auto& j_i : joint_jacobians) {
    if (j_i.rows() != 6 || j_i.cols() != nv_total) {
      j_i = Matrix6X::Zero(6, nv_total);
    } else {
      j_i.setZero();
    }
  }
}
```

Two branches: (a) cold-start, behave as before; (b) warm-start, reuse
storage. Both produce identical output. Test it.

### Step 4 — Re-measure

```
bench_joint_jacobians_impl/franka_fr3   1748 ns   1747 ns   401375  items_per_second=572.4k/s
```

1.88 μs → 1.75 μs. About **7% improvement**, matching the hypothesis.
Across all 4 fixtures the improvement was 5–10%.

Were the unit tests still green? Yes — `ctest --preset=debug` is
143/143.

### Step 5 — Commit

```
perf(diff): reuse compute_joint_jacobians storage on warm calls

Before: 1881 ns / call on Franka
After:  1748 ns / call on Franka  (-7%)

When the caller's vector is already sized correctly, `setZero()` each
element rather than re-`assign`-ing the whole vector. Hot-loop users
(analytical-derivative inner sweeps) now pay no allocations per call.
```

The before/after numbers in the commit message let a future maintainer (or a code
reviewer) see exactly what changed, without having to re-run the
benchmark themselves.

## What a *bad* optimisation loop looks like

Here's what to avoid:

1. **"This looks slow."** Without a measurement, there is no way to know. The
   change might have made things worse, undetected.
2. **Changing many things at once.** Replacing `std::variant` with
   `enum` *and* caching rotation matrices *and* inlining the inner sweep,
   then seeing the number go down 30%, gives no indication of which move
   contributed. Worse — one of them might have *hurt*, and the others
   compensated.
3. **Optimising the wrong thing.** Profiling says the hot loop is in
   `Eigen::internal::scalar_product_op`, a week goes into rewriting
   `SE3::operator*`, and it turns out to be a tiny fraction of total time.
   Look at the profile *first*.
4. **Measuring once, calling it done.** Benchmark noise is real. Run
   a few times. If the change is at noise level, it's not an
   improvement.

## When to *stop* optimising

The most disciplined skill in performance work is *not* optimising.
Two rules:

- **The target is met.** The bar is 1.4× of Pinocchio. Past that point,
  every further optimisation costs more in code complexity than it gains
  in speed.
- **One thing has been optimised too far.** When the work involves writing
  `__attribute__((always_inline))` everywhere, or hand-writing SIMD
  intrinsics, ask: is the goal of the codebase still readability?

For `tinyspatial` we're at ~3× of Pinocchio on Franka RNEA right now
(C++ direct), and ~9× from Python. The Python gap is binding overhead —
Phase 9b will address. The C++ gap is architectural — also Phase 9b
(rotation-matrix caching in `Data`, `std::variant` → discriminator
switch). After 9b we'd ideally be at 1.4–1.8× and *stop*. The remaining
gap is just "Pinocchio is older and has more PhD-years of tuning;"
fighting for the last 30% buys diminishing returns.

## The pattern, distilled

```
WHILE not at target:
  profile -> identify hot fn
  hypothesise specific change
  apply minimal diff
  re-bench
  verify correctness
  commit with numbers
  ASK: is this still worth doing?
END
```

That's the whole game. The structure is unglamorous; the payoff is
that every commit makes the code measurably better, with proof
attached.

> ## Where this lives in the library
>
> | Concept                            | File / line                                  |
> | ---------------------------------- | -------------------------------------------- |
> | The Phase 9a allocation fix        | [`fk_derivatives.hpp`](../../include/tinyspatial/diff/fk_derivatives.hpp) |
> | Tracked numbers                    | [`docs/BENCHMARKS.md`](../../docs/BENCHMARKS.md) |
> | Pinocchio side-by-side             | [`python/tools/benchmark_vs_pinocchio.py`](../../python/tools/benchmark_vs_pinocchio.py) |
