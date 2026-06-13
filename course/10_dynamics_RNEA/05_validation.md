# Validating against Pinocchio

A dynamics implementation is the kind of code where you can be very wrong
in very subtle ways. A sign flip on a cross product, an angular-vs-linear
mix-up, a wrong adjoint — all of these will pass unit tests for a single
robot and fail catastrophically on a different one. The only defense is
**numerical agreement with a known-good implementation on real robots**.

Our oracle is [Pinocchio](https://github.com/stack-of-tasks/pinocchio) 3.9
— the reference for rigid-body dynamics in robotics research. The
validation harness lives at `tests/validation/test_kinematics.py` and runs
on every CI build flagged with `TINYSPATIAL_BUILD_VALIDATION=ON`.

## How it works

For each fixture URDF (`simple_arm`, `franka_fr3`, `ur5e`, `so_arm101`):

1. **Load the URDF in both libraries.** Pinocchio's `buildModelFromUrdf`
   and tinyspatial's `build_model_from_urdf_file` parse the same file.
   They must agree on `nq` and `nv` — the harness asserts this up front.

2. **Sample 1000 random configurations from a fixed seed.** We use
   `pin.randomConfiguration` so the harness is reproducible and so
   floating-base joints (when we add them) automatically respect
   quaternion-normalisation constraints. The seed is `0xC0FFEE` —
   change-detection in CI relies on this being deterministic.

3. **For each sample $(q, v, a)$, call both implementations:**
   - `pin.rnea(pin_model, pin_data, q, v, a)` → $\tau_{\text{pin}}$
   - `ts.rnea(ts_model, q, v, a, gravity)` → $\tau_{\text{ts}}$

4. **Compare:** $\| \tau_{\text{pin}} - \tau_{\text{ts}} \|_\infty$.

5. **Aggregate the max across 1000 samples.** If any robot's max diff
   exceeds the tolerance, CI fails the build.

## The tolerance

The cross-check tolerance is fixed at **`1e-10` absolute**, with
**no relaxation without a written justification**. In practice we observe
agreement at $10^{-14}$–$10^{-15}$ — four to five orders of magnitude
inside the spec. That margin is meaningful: it tells you we are not just
*close* to Pinocchio, we are doing literally the same arithmetic in the
same order, modulo Eigen's vectorisation choices.

The latest run is in [`docs/PINOCCHIO_PARITY.md`](../../docs/PINOCCHIO_PARITY.md).

## Convention differences (and how the harness handles them)

Two real differences between the libraries that the harness corrects for at
the boundary, *not* by changing tinyspatial:

1. **Joint indexing.** Pinocchio has a "universe" joint at index 0 and lists
   movable joints from 1 to $n$. We list every joint from 0. The harness
   uses `pin_index - 1 → ts_index`.

2. **Gravity-vector setter.** Pinocchio stores gravity as a `Motion` on the
   model (`pin_model.gravity`); the harness sets
   `pin_model.gravity.linear = gravity` explicitly to make the comparison
   deterministic, in case Pinocchio's default has been mutated.

For RNEA specifically, no convention conversion is needed once those two
boundary fixes are applied. RNEA's output is joint torque, a per-joint
scalar; both libraries return it in the same `idx_v` order.

(For the Jacobian, by contrast, there is a row permutation between
linear-first and angular-first orderings; see chapter 09. RNEA doesn't see
that.)

## What this catches

- **Wrong sign on a cross product.** Single-robot tests can miss this if
  the wrong sign happens to cancel another wrong sign for a specific
  geometry. 1000 random configurations × 4 robots × all joints does not.
- **Confused frames.** A bug where you transport with the motion adjoint
  instead of the dual would pass `NoGravityNoMotionGivesZeroTorque` (both
  give zero) but fail random `(q, v, a)` immediately.
- **URDF parsing diverging from Pinocchio's.** If we misread the
  parent-link rotation in a URDF, all the FK is shifted by that rotation,
  and that propagates into every dynamic quantity. The harness catches it.

## What this does *not* catch

- **Both libraries having the same bug.** If Featherstone's textbook has a
  typo and both libraries follow it, you'd never know from cross-validation
  alone. This is unlikely in practice (Pinocchio is used by enough research
  groups that pure-algorithmic bugs would be caught externally), but it's
  worth naming: cross-validation is one defense, not the only one.
- **Performance.** This harness checks *what* the algorithm computes, not
  *how fast*. Benchmarks are separate (chapter 16).

## Running it yourself

```bash
cmake --preset=validation
cmake --build build/validation
.venv/bin/python tests/validation/test_kinematics.py
```

The script prints per-robot max-diffs and (re)generates
[`docs/PINOCCHIO_PARITY.md`](../../docs/PINOCCHIO_PARITY.md). Treat that file
as ground truth: it's auto-generated, and the table you see is the actual
numbers from your local run.

> ## Where this lives in the library
>
> | Concept | File / line |
> | ------- | ----------- |
> | The harness | [`tests/validation/test_kinematics.py`](../../tests/validation/test_kinematics.py) |
> | RNEA comparison | [`test_kinematics.py:114-123`](../../tests/validation/test_kinematics.py#L114-L123) |
> | Parity table | [`docs/PINOCCHIO_PARITY.md`](../../docs/PINOCCHIO_PARITY.md) |
