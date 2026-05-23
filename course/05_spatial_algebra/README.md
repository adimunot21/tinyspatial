# Chapter 05 — Spatial algebra

Chapter 04 told you that the tangent space of $SE(3)$ holds twists — 6-vectors
combining angular and linear velocity. Now we make that idea full-strength: a
*language* for rigid-body kinematics and dynamics where every physical quantity
— velocity, force, inertia — lives in a single, transformable 6-D space.

This is the vocabulary that Featherstone's dynamics algorithms speak. Once you
know it, RNEA / ABA / CRBA (chapters 10–11) read like prose. The library code
in `include/tinyspatial/spatial/` is the dictionary.

## The sub-chapters

1. [Why six-vectors?](01_why_six_vectors.md) — what we gain by packing rotation
   and translation into one object.
2. [Twists and wrenches](02_twists_and_wrenches.md) — the typed
   `Motion`/`Force` distinction and why it matters.
3. [The Plücker transform](03_plucker.md) — how a 6-vector changes when you
   change frames. Same object as the SE(3) adjoint.
4. [Spatial inertia](04_spatial_inertia.md) — the $6\times6$ map from velocity
   to momentum.
5. [How Featherstone thinks](05_how_featherstone_thinks.md) — the mental model
   you carry into the dynamics chapters.

Then: [exercises](exercises.md).

## Prerequisites

Chapter 04 (Lie groups), especially exp/log and the adjoint.

## Where this lives in the library

| Concept | File |
| ------- | ---- |
| Spatial 6-vectors (motion, force) | [`include/tinyspatial/spatial/motion.hpp`](../../include/tinyspatial/spatial/motion.hpp), [`force.hpp`](../../include/tinyspatial/spatial/force.hpp) |
| Plücker transforms | [`include/tinyspatial/spatial/plucker.hpp`](../../include/tinyspatial/spatial/plucker.hpp) |
| Spatial inertia | [`include/tinyspatial/spatial/inertia.hpp`](../../include/tinyspatial/spatial/inertia.hpp) |
| Convention note | [`docs/ALGORITHMS.md`](../../docs/ALGORITHMS.md) |
