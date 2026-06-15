# Chapter 12 — Inverse kinematics

Forward kinematics (chapter 08) maps joint angles to a hand pose:

$$
q \;\longmapsto\; \mathrm{FK}(q) \;\in\; \mathrm{SE}(3).
$$

Inverse kinematics goes the other way. Given a desired pose $T^*$ for
some link of the robot, find a configuration $q^*$ such that
$\mathrm{FK}(q^*) = T^*$.

This is harder than it sounds:

1. **Many or no solutions.** A 7-DoF arm can reach the same point with
   the elbow up *or* the elbow down (and any of infinitely many
   in-between configurations); a 3-DoF arm might not reach the point
   at all.
2. **Closed-form is rare.** A handful of robots (the standard 6-DoF
   "spherical-wrist" arm, for instance) have analytic IK; most don't.
3. **Singularities are intrinsic.** Near certain configurations the
   Jacobian loses rank, and the standard inverse-Jacobian methods
   explode.

This chapter covers the **iterative Jacobian-based** family of IK
algorithms — the workhorses of robotics. They are not the only family
(closed-form, sampling-based, optimisation-based all exist), but they
match what the library provides in `include/tinyspatial/ik/`:

- **DLS** (damped least-squares): the entry-level robust solver.
- **Nullspace IK**: DLS plus a secondary objective for redundant arms.

## The sub-chapters

1. [The inverse problem](01_the_inverse_problem.md) — formal statement;
   why it's underdetermined; how to *write down* the error on a Lie
   group.
2. [The Jacobian approach](02_the_jacobian_approach.md) — Newton-Raphson
   on $\mathrm{SE}(3)$ Lie-tangent error.
3. [Pseudoinverse and damping](03_pseudoinverse_and_damping.md) —
   what the textbook Moore–Penrose pseudoinverse is, why DLS exists, and
   when each is appropriate.
4. [In code: solve_ik_dls](04_in_code.md) — `dls.hpp` walked through line
   by line.
5. [Null-space and secondary tasks](05_nullspace_secondary.md) —
   redundancy as a feature, posture attraction, the two-tier damping
   trick.
6. [When IK fails](06_convergence_and_failure.md) — singularities,
   basin failures, random restarts, debugging tips.

Then: [exercises](exercises.md).

## Prerequisites

Chapters 03 (rotations/transforms), 04 (Lie groups — especially
`exp`/`log` on $\mathrm{SE}(3)$), 08 (FK), 09 (Jacobians — especially
the LOCAL frame).

## Where this lives in the library

| Concept | File |
| ------- | ---- |
| DLS solver | [`include/tinyspatial/ik/dls.hpp`](../../include/tinyspatial/ik/dls.hpp) |
| Nullspace solver | [`include/tinyspatial/ik/nullspace.hpp`](../../include/tinyspatial/ik/nullspace.hpp) |
| Unit tests | [`tests/unit/ik/test_dls.cpp`](../../tests/unit/ik/test_dls.cpp), [`tests/unit/ik/test_nullspace.cpp`](../../tests/unit/ik/test_nullspace.cpp) |

## Further reading

- **Buss & Kim**, "Selectively Damped Least Squares for Inverse
  Kinematics" (J. Graphics Tools, 2005). Short, practical paper that
  explains why DLS works and where it struggles.
- **Siciliano**, "Kinematic control of redundant robot manipulators: A
  tutorial" (J. Intelligent and Robotic Systems, 1990). The classic
  nullspace projection reference.
- **Lynch & Park**, *Modern Robotics* §6 — same material with different
  notation; good for cross-checking conventions.
