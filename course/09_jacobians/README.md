# Chapter 09 — Jacobians

Forward kinematics tells you *where* a link is. The **Jacobian** tells you
*how fast it would move* if you moved the joints. It's the linear map between
joint velocities and link velocity, and it sits between FK (chapter 08) and
every iterative algorithm that wants to descend on it: IK (chapter 12),
optimisation, robust control.

Three things make Jacobians trickier than FK:

1. They depend on the *reference frame* — there are three reasonable
   choices, and they're related by adjoints (chapter 04).
2. The same matrix is called both "geometric" and "analytical" depending on
   conventions; we use only the geometric form, with a flagged exception.
3. Where they go *singular*, the robot loses controllability. The intuition
   is worth time on its own.

## The sub-chapters

1. [What a Jacobian is](01_what_a_jacobian_is.md) — the derivative of FK,
   one column per joint velocity.
2. [Geometric vs analytical](02_geometric_vs_analytical.md) — what we compute
   and what the alternative would be.
3. [Reference frames](03_reference_frames.md) — LOCAL / WORLD /
   LOCAL_WORLD_ALIGNED, and how each is related by a Plücker / adjoint.
4. [Computing it: the screw-axis walk](04_in_code.md) — walk to root, place
   each joint's screw axis, transform to the chosen frame.
5. [Singularities](05_singularities.md) — what it means when the Jacobian
   drops rank, and why every 6-DOF arm has them.

Then: [exercises](exercises.md).

## Prerequisites

Chapters 04 (Lie groups), 05 (spatial algebra), 06–08. Especially the
adjoint, twists, and FK.

## Where this lives in the library

| Concept | File |
| ------- | ---- |
| Jacobian algorithm | [`include/tinyspatial/algo/jacobian.hpp`](../../include/tinyspatial/algo/jacobian.hpp) |
| Tests | [`tests/unit/algo/test_jacobian.cpp`](../../tests/unit/algo/test_jacobian.cpp) |
| Pinocchio parity | [`docs/PINOCCHIO_PARITY.md`](../../docs/PINOCCHIO_PARITY.md) |
