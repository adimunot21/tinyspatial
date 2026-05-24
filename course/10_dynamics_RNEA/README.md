# Chapter 10 — Inverse dynamics with RNEA

You have a robot's kinematic tree (chapter 6), you can load it from a URDF
(chapter 7), and you know where every link is (chapter 8) and how it would
move under joint velocity (chapter 9). Now the next question:

> *"To make the joints follow this trajectory `q(t), q̇(t), q̈(t)`, what
> torques do my motors need to produce?"*

The answer is **inverse dynamics**: solve

$$
\tau \;=\; M(q)\,\ddot q \;+\; C(q, \dot q)\,\dot q \;+\; g(q)
$$

for $\tau$ given $q, \dot q, \ddot q$. The algorithm we use is **Featherstone's
Recursive Newton–Euler Algorithm (RNEA)**, and it does this in $O(N)$ without
ever forming $M$, $C$, or $g$ separately — it just adds them up as it walks
the tree twice.

This chapter teaches RNEA from the ground up: what each pass is doing, why
the "gravity trick" works, and how the code in `include/tinyspatial/algo/rnea.hpp`
maps line-by-line onto the textbook recursion.

## The sub-chapters

1. [The equation of motion](01_equation_of_motion.md) — what $\tau = M\ddot q +
   h$ means, and why each piece exists.
2. [Newton–Euler on one body](02_one_body.md) — RNEA on a single rigid body,
   so the machinery is visible without the tree.
3. [The two passes](03_two_passes.md) — outward (kinematics) and inward
   (forces), with the gravity trick.
4. [In code](04_in_code.md) — the angular-first, body-fixed implementation
   in `rnea.hpp`.
5. [Validating against Pinocchio](05_validation.md) — how we know it's right
   to machine precision.

Then: [exercises](exercises.md).

## Prerequisites

Chapters 04 (Lie groups — exp, log, adjoint), 05 (spatial algebra — twists,
wrenches, the spatial cross product, spatial inertia), 06 (kinematic trees),
08 (forward kinematics).

## Where this lives in the library

| Concept | File |
| ------- | ---- |
| RNEA algorithm | [`include/tinyspatial/algo/rnea.hpp`](../../include/tinyspatial/algo/rnea.hpp) |
| Unit tests | [`tests/unit/algo/test_rnea.cpp`](../../tests/unit/algo/test_rnea.cpp) |
| Pinocchio parity | [`docs/PINOCCHIO_PARITY.md`](../../docs/PINOCCHIO_PARITY.md) |

## Further reading

- Featherstone, *Rigid Body Dynamics Algorithms* (2008), chapters 5 and 6.
- Lynch & Park, *Modern Robotics* §8.3 — same algorithm with different
  notation; useful for cross-checking conventions.
