# Reading the parity table

Open [`docs/PINOCCHIO_PARITY.md`](../../docs/PINOCCHIO_PARITY.md). You'll
see something like this:

```
| Robot         | FK         | J (LOCAL)  | J (WORLD)  | J (LWA)   |
| ------------- | ---------- | ---------- | ---------- | --------- |
| `simple_arm`  | `7.77e-16` | `9.99e-16` | `1.44e-15` | `1.55e-15`|
| `franka_fr3`  | `1.22e-15` | `1.21e-15` | `1.33e-15` | `1.33e-15`|

| Robot         | RNEA (inverse) | CRBA (mass matrix) | ABA (forward)|
| ------------- | -------------- | ------------------ | ------------ |
| `simple_arm`  | `3.55e-15`     | `1.78e-15`         | `7.11e-15`   |
| `franka_fr3`  | `7.11e-14`     | `5.33e-15`         | `9.38e-13`   |
```

This chapter is about what those numbers mean.

## Each cell is a max-abs difference

For each `(algorithm, robot)` cell, we sample 1000 random
configurations, run both libraries, and record:

```
diff_per_sample = max( |tinyspatial_output[i] - pinocchio_output[i]| )
                  over all i in the output

cell = max over 1000 samples of diff_per_sample
```

So `franka_fr3 RNEA = 7.11e-14` means: across 1000 random `(q, v, a)`
triples on Franka, the *worst* single torque component disagreed
between the two libraries by `7.11 × 10⁻¹⁴`. Everything else was at
or below that.

This is intentionally a max, not a mean: a single rogue sample
disagreeing by 1e-3 should fail the test even if the other 999 agree
at 1e-15.

## Three magnitude bands

| Band         | Interpretation                                         |
| ------------ | ------------------------------------------------------ |
| `< 1e-15`    | Identical up to floating-point order of operations.    |
| `1e-14 … 1e-12` | Identical up to FP catastrophic-cancellation noise.|
| `1e-11 … 1e-10` | Suspicious. Probably accumulated round-off in a deep chain. |
| `> 1e-10`    | **Failure.** This is the tolerance bar. |

The reason `1e-15` is the floor: doubles have ~16 significant decimal
digits. Two algorithms that compute the same answer through *different*
order-of-operations will accumulate FP noise on the order of
`O(ε · n)` where `ε ≈ 2.2e-16` and `n` is the number of operations.

For a 7-DoF Franka RNEA with maybe 500 floating-point ops per call,
`O(500 · ε) ≈ 1.1e-13`. That's exactly what we see (`7.11e-14`). The
fact that we're *below* that and not above is just Pinocchio and
`tinyspatial` having similar (but not identical) order-of-operations.

The tolerance bar at `1e-10` is conservative — five orders of
magnitude above the FP noise floor. If a parity test crosses 1e-10,
something *qualitatively* changed: a bug, a convention difference,
or a real algorithm regression. We don't relax this without writing
a justification.

## Why some cells are looser than others

Two patterns:

**Pattern 1: Deeper chains accumulate more noise.**

The Franka FK is `1.22e-15` (one matrix multiply per joint, 7 deep);
Franka RNEA is `7.11e-14` (one inertia application + one Plücker
transport + one cross product per joint, two passes, much more
arithmetic). The ratio is ~58×, roughly matching the op-count ratio.

**Pattern 2: ABA's `1e-12` is real.**

Look at Franka ABA: `9.38e-13`. That's 7× the RNEA value. ABA does
*more* per joint — it has an extra symmetric congruence (`X * IA * X^T`)
in the inward pass, plus solving a small linear system. The noise
accumulates faster.

The fact that this number is sitting at `1e-12` rather than `1e-10`
tells you it's *just* FP noise — not a convention bug. If you saw
`1e-7`, you'd worry.

## A field-by-field tour

### Forward kinematics (FK)

```
| `franka_fr3` | `1.22e-15` |
```

For each of 1000 random `q`, the worst element of the worst 4×4
homogeneous matrix differed by `1.22e-15`. This is FP-noise floor.

### Jacobian per frame

```
| `franka_fr3` | J (LOCAL): 1.21e-15 | J (WORLD): 1.33e-15 | J (LWA): 1.33e-15 |
```

Three frames means three separate parity tests. Note that WORLD and
LOCAL_WORLD_ALIGNED are slightly looser than LOCAL — they involve
extra transformations on the output (multiplying by `Ad_T` and a
block-diagonal rotation respectively), so a tiny bit more arithmetic.
Still all at FP noise.

### RNEA, CRBA, ABA

```
| franka_fr3 | RNEA: 7.11e-14 | CRBA: 5.33e-15 | ABA: 9.38e-13 |
```

- RNEA: ~50× the FK noise. Expected.
- CRBA: ~3× the FK noise. The mass matrix is symmetric and the
  algorithm is more parallel; less accumulation.
- ABA: ~600× the FK noise. The articulated-inertia inward pass is
  the most arithmetic-heavy of any algorithm we've shipped.

### Analytical RNEA derivatives

```
| franka_fr3 | ∂τ/∂q: 1.28e-13 | ∂τ/∂v: 1.42e-14 | ∂τ/∂a (= M): 6.22e-15 |
```

`∂τ/∂a = M(q)` matches the CRBA cell directly (same mathematical
object). `∂τ/∂q` is looser because it sweeps through every body
twice (forward to accumulate `dv_dq`, then backward to project
through each joint) — more arithmetic, more accumulated noise.

## Regeneration: when and how

The committed numbers in `docs/PINOCCHIO_PARITY.md` are *snapshots*.
The validation suite regenerates them at the end of every successful
parity run:

```bash
cmake --preset=validation
cmake --build build/validation -j
ctest --preset=validation -L pinocchio_parity --output-on-failure
# ↑ updates docs/PINOCCHIO_PARITY.md if all parity tests pass
```

The committed table moves whenever:

- An algorithm's implementation changes (Phase 9b's spatial-algebra
  inlining shifted a few cells by ~1×10⁻¹⁵ — FP order of ops).
- Pinocchio's version is bumped.
- A new algorithm is added to the suite.

Reviewers of a PR look at the *diff* of `PINOCCHIO_PARITY.md`. A row
moving from `1e-14` to `5e-14` is unremarkable. A row moving from
`1e-14` to `1e-7` is a flag — go find the change that caused it.

## The anatomy of `test_kinematics.py`

Open [`tests/validation/test_kinematics.py`](../../tests/validation/test_kinematics.py).
It's ~300 lines, structured as:

1. **Imports + constants.** `NUM_SAMPLES = 1000`, `TOLERANCE = 1e-10`,
   `SEED = 0xC0FFEE`. The seed is committed so the same 1000
   configurations are tested every run — deterministic.
2. **`Result` dataclass.** One per robot, holds the max-abs diff for
   each algorithm × frame.
3. **`validate(robot)` function.** Loads the URDF in both libraries,
   samples `(q, v, a)` 1000 times, runs FK / J / RNEA / CRBA / ABA /
   RNEA-derivatives in both, accumulates max-abs differences.
4. **`write_parity_table(results)` function.** Emits
   `docs/PINOCCHIO_PARITY.md` as a Markdown table.
5. **`main()` function.** Runs `validate` on each fixture, asserts
   `< TOLERANCE` everywhere, writes the table.

There's also `python/tests/test_parity.py` — a smaller, pytest-friendly
subset that exercises the *public Python API* end-to-end. Run that
during binding development; run the full `test_kinematics.py` after any
algorithm change.

## What this discipline gets you

Every PR comment that says "I'm not sure my implementation is right" can
be answered with: "Did the parity test pass? Then it's right."

Every reviewer who needs to gut-check a change reads the
`PINOCCHIO_PARITY.md` diff to see the actual numerical impact.

Every reader who wants to know how trustworthy the library is opens
the committed table and sees `1e-13 to 1e-15`.

That last one is the credibility signal. Hiring managers reading this
repo see "parity to 1e-13 with Pinocchio on 1000 configurations" and
form a fundamentally different opinion than they would from a README
that says "we tested it on a few configurations."

The numbers are not the test; the *commitment* to the numbers is.

> ## Where this lives in the library
>
> | Concept                       | File path                                |
> | ----------------------------- | ---------------------------------------- |
> | The validation script         | [`tests/validation/test_kinematics.py`](../../tests/validation/test_kinematics.py) |
> | The committed parity numbers  | [`docs/PINOCCHIO_PARITY.md`](../../docs/PINOCCHIO_PARITY.md) |
> | Tolerance constant            | `TOLERANCE = 1e-10` in `test_kinematics.py` |
> | Python pytest subset          | [`python/tests/test_parity.py`](../../python/tests/test_parity.py) |
