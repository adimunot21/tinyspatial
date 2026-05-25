# Chapter 11 — ABA and CRBA

Chapter 10 solved *inverse* dynamics: given a desired motion $(q, \dot q,
\ddot q)$, what torque $\tau$ must the motors produce? Here we solve the
two complementary problems:

1. **Forward dynamics** — given $\tau$ at each motor and the current $(q,
   \dot q)$, what acceleration $\ddot q$ results? This is the *simulator's*
   problem: every physics step, you have torques and you want to integrate
   forward.

2. **The mass matrix $M(q)$** — sometimes you really do want the joint-space
   inertia in closed form. For instance: stability analysis of a controller,
   detection of dynamics-singularities, projection onto contact constraints.

Two algorithms; both due to Featherstone; both $O(n)$ on a tree.

- **ABA** (the *Articulated-Body Algorithm*) is the elegant one: forward
  dynamics in $O(n)$ without ever forming $M$. Read it for the
  articulated-inertia idea, which keeps showing up in modern simulators.

- **CRBA** (the *Composite-Rigid-Body Algorithm*) is the workhorse: it
  computes $M(q)$ directly in $O(n^2)$, which is asymptotically worse than
  ABA but easier to reason about. Often you want $M$ for its own sake.

The two together — $M$ from CRBA and the bias $h(q, \dot q) := C \dot q + g$
from RNEA with $\ddot q = 0$ — give you the long way to forward dynamics:
$\ddot q = M^{-1}(\tau - h)$. ABA computes the same thing in one sweep.

## The sub-chapters

1. [Forward vs inverse dynamics](01_forward_vs_inverse.md) — what each
   problem looks like and when you'd want which.
2. [Composite rigid bodies (CRBA)](02_crba.md) — the idea of "the inertia of
   everything below this joint, seen at this joint."
3. [Articulated bodies (ABA)](03_aba.md) — Featherstone's clever
   refactoring: an articulated body is a rigid body with a different inertia
   matrix.
4. [ABA in code](04_in_code.md) — the three-pass implementation in
   `aba.hpp`.
5. [Validating against Pinocchio](05_validation.md) — how the cross-check
   harness extends to CRBA and ABA.

Then: [exercises](exercises.md).

## Prerequisites

Chapter 10 (RNEA), chapter 05 (spatial algebra), chapters 06–08.

## Where this lives in the library

| Concept | File |
| ------- | ---- |
| CRBA | [`include/tinyspatial/algo/crba.hpp`](../../include/tinyspatial/algo/crba.hpp) |
| ABA | [`include/tinyspatial/algo/aba.hpp`](../../include/tinyspatial/algo/aba.hpp) |
| Unit tests | [`tests/unit/algo/test_crba.cpp`](../../tests/unit/algo/test_crba.cpp), [`tests/unit/algo/test_aba.cpp`](../../tests/unit/algo/test_aba.cpp) |
| Pinocchio parity | [`docs/PINOCCHIO_PARITY.md`](../../docs/PINOCCHIO_PARITY.md) |

## Further reading

- Featherstone, *Rigid Body Dynamics Algorithms* (2008), chapters 6 (CRBA)
  and 7 (ABA).
- Featherstone & Orin, "Robot Dynamics: Equations and Algorithms" (ICRA
  2000) — overview of the family, including how it relates to Newton–Euler
  and Lagrangian formulations.
