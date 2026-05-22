# Chapter 04 — Lie groups

Chapter 03 gave us rotations and rigid transforms as *things*. This chapter is
about how they *change* — velocities, small perturbations, derivatives. That
turns out to need a genuinely different mental picture, because the set of
rotations is curved, not flat. The mathematics for "calculus on a curved space
of transformations" is the theory of **Lie groups**, and it is the backbone of
every algorithm in this library.

Don't be put off by the name. We only need a working subset, and we'll build it
from pictures, not abstraction.

## The sub-chapters

1. [The manifold idea](01_the_manifold_idea.md) — why rotations live on a curved
   surface, and what "three degrees of freedom" really means.
2. [The tangent space](02_tangent_space.md) — angular velocity as an arrow in a
   flat space attached to the manifold.
3. [exp and log](03_exp_and_log.md) — the bridge between the flat tangent space
   and the curved group.
4. [Jacobians](04_jacobians.md) — how a wiggle in the tangent space turns into a
   wiggle on the manifold.
5. [The adjoint](05_the_adjoint.md) — moving a velocity from one frame to
   another.
6. [Why robots need this](06_why_robots_need_this.md) — cashing it all in.

Then: [exercises](exercises.md).

## Prerequisites

Chapter 03 (rotations and transforms). A little calculus intuition (what a
derivative *is*) helps but isn't essential.

## Where this lives in the library

| Concept | File |
| ------- | ---- |
| SO(3): exp, log, Jacobians | [`include/tinyspatial/liegroup/so3.hpp`](../../include/tinyspatial/liegroup/so3.hpp) |
| SE(3): exp, log, adjoint, Jacobians | [`include/tinyspatial/liegroup/se3.hpp`](../../include/tinyspatial/liegroup/se3.hpp) |
| Spatial cross products (the Lie bracket) | [`include/tinyspatial/spatial/cross.hpp`](../../include/tinyspatial/spatial/cross.hpp) |
