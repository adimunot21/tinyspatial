# Using Google Benchmark

Google Benchmark (`benchmark`) is the de-facto standard C++ micro-benchmarking
library. It handles the bookkeeping that makes a fair benchmark — iteration
counts, timing, statistical reporting, CPU affinity warnings — so the
benchmark code doesn't have to.

[`benchmarks/bench_rnea.cpp`](../../benchmarks/bench_rnea.cpp) is the
worked example for this chapter.

## The minimal benchmark

```cpp
#include <benchmark/benchmark.h>

static void BM_Empty(benchmark::State& state) {
  for (auto _ : state) {
    // The 'thing to benchmark' goes here.
  }
}
BENCHMARK(BM_Empty);

BENCHMARK_MAIN();
```

That's it. The `for (auto _ : state)` loop is special — Google Benchmark
adapts the iteration count to reach a target wall-clock budget (default:
0.5 seconds per row). The loop count is almost never written by hand.

`BENCHMARK_MAIN()` expands to a `main()` that handles command-line flags
and dispatches to all registered benchmarks.

## A real benchmark, from our suite

```cpp
struct ModelKit {
  tinyspatial::Model model;
  tinyspatial::Data data;
  tinyspatial::VectorX q, v, a, tau;

  explicit ModelKit(const std::string& urdf)
      : model(tinyspatial::build_model_from_urdf_file(urdf)),
        data(model),
        q(tinyspatial::VectorX::Zero(model.nq())),
        v(tinyspatial::VectorX::Zero(model.nv())),
        a(tinyspatial::VectorX::Zero(model.nv())),
        tau(tinyspatial::VectorX::Zero(model.nv())) {
    // Random non-zero inputs. RNEA at all zeros short-circuits the inner
    // work, so it would lie about the steady-state cost.
    std::mt19937 gen(0xCAFE);
    std::uniform_real_distribution<double> uni(-1.0, 1.0);
    for (int i = 0; i < model.nq(); ++i) q(i) = uni(gen);
    for (int i = 0; i < model.nv(); ++i) {
      v(i) = uni(gen);
      a(i) = uni(gen);
    }
  }
};

void bench_rnea_impl(benchmark::State& state, const char* urdf) {
  ModelKit kit(fixture(urdf));        // setup, outside the timed loop
  for (auto _ : state) {
    tinyspatial::rnea(kit.model, kit.data, kit.q, kit.v, kit.a, kit.tau);
    benchmark::DoNotOptimize(kit.tau);
  }
  state.SetItemsProcessed(state.iterations());
}

BENCHMARK_CAPTURE(bench_rnea_impl, franka_fr3, "franka_fr3.urdf");
```

Five details worth knowing:

### 1. Setup goes *outside* the timed loop

The `ModelKit` constructor reads the URDF, allocates `Data`, and
randomises `q`. None of that is part of the per-call cost under
measurement. It runs *once* per registered row. The `for (auto _ : state)`
loop runs exactly the algorithm of interest.

### 2. `BENCHMARK_CAPTURE` parameterises the benchmark

`BENCHMARK_CAPTURE(fn, suffix, args...)` registers a benchmark whose
displayed name is `fn/suffix` and which calls `fn(state, args...)`. So
one row per fixture URDF:

```cpp
BENCHMARK_CAPTURE(bench_rnea_impl, simple_arm, "simple_arm.urdf");
BENCHMARK_CAPTURE(bench_rnea_impl, franka_fr3, "franka_fr3.urdf");
BENCHMARK_CAPTURE(bench_rnea_impl, ur5e,       "ur5e.urdf");
```

Reads cleanly in the output.

### 3. `DoNotOptimize` is the *anti-elision* tool

Without `DoNotOptimize`, the compiler can see that the result of `rnea(...)`
is never used outside the loop and *elide the entire call* — leaving the
benchmark measuring nothing.

`benchmark::DoNotOptimize(x)` emits an empty assembly clobber that tells
the compiler "treat the value of `x` as if it could be read by some
unknown code." The compiler then can't eliminate `x` or anything that
depends on it. The runtime cost is zero — it's a compiler hint, not a
syscall.

At the assembly level, the implementation is roughly
```cpp
template <typename T>
inline void DoNotOptimize(const T& value) {
  asm volatile("" : : "r,m"(value) : "memory");
}
```
which says "the value of `value` is in either a register or memory, and
treat all of memory as clobbered."

### 4. `state.SetItemsProcessed(state.iterations())`

This tells the framework "1 logical item of work was done per inner
iteration." Google Benchmark then reports `items_per_second=...` which
is the throughput that matters, not just `ns/iter`.

For batched algorithms the call becomes `state.SetItemsProcessed(state.iterations() * batch_size)`.

### 5. The output

```
bench_rnea_impl/franka_fr3   4540 ns   4540 ns   154083  items_per_second=220.287k/s
```

Five columns:

- **Time** — wall-clock per iteration. Wall-clock can include the OS
  scheduling the process off-CPU.
- **CPU** — CPU time per iteration. Should match Time unless the OS
  preempts the process.
- **Iterations** — how many times the inner loop ran. Google Benchmark
  picked this automatically to fit in `--benchmark_min_time` seconds.
- **items_per_second** — the throughput set with `SetItemsProcessed`.

When Time and CPU disagree by more than a few percent, that's
scheduling noise. Re-run, or use `--benchmark_min_warmup_time=0.5s` to
let things settle.

## CPU pinning and frequency scaling

Google Benchmark warns:

```
***WARNING*** CPU scaling is enabled, the benchmark real time measurements
may be noisy and will incur extra overhead.
```

The fix on Linux is to lock the CPU governor to `performance`:

```bash
sudo cpupower frequency-set --governor performance
```

For maximum reproducibility, also pin the benchmark to a specific
core with `taskset -c 3 ./bench_rnea`. The committed numbers don't do this
because it's machine-specific; the observed variance is small enough
to not matter for the conclusions being drawn (3× vs 1× ratios).

## Filtering and reporting

Run only a subset:

```bash
./bench_rnea --benchmark_filter=franka
```

JSON output for scripting:

```bash
./bench_rnea --benchmark_format=json > rnea.json
```

Repeat for confidence intervals:

```bash
./bench_rnea --benchmark_repetitions=5 --benchmark_report_aggregates_only=true
```

For one-off optimisation work, the defaults suffice. For numbers that
go on a graph, run repetitions and report mean+stddev.

## What's next

With timing in hand, the next chapter is about figuring out
*where* the time goes — which is what identifies what to optimise.

> ## Where this lives in the library
>
> | Concept                       | File path                               |
> | ----------------------------- | --------------------------------------- |
> | Benchmark sources             | [`benchmarks/`](../../benchmarks/)      |
> | Bench CMakeLists              | [`benchmarks/CMakeLists.txt`](../../benchmarks/CMakeLists.txt) |
> | Committed throughput numbers  | [`docs/BENCHMARKS.md`](../../docs/BENCHMARKS.md) |
