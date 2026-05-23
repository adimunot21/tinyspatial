# Chapter 03 — Rotations and transforms

Everything a robot arm does is, at bottom, moving rigid things around in space.
Before we can compute *where the hand goes*, we need a precise, unambiguous way
to describe orientation and position. That's this chapter.

By the end you'll know the four ways we write down a rotation — and crucially,
*why* each exists and when to reach for it — plus how rotation and translation
combine into a single object, the rigid transform.

## The sub-chapters

1. [What is a rotation?](01_what_is_a_rotation.md) — the idea, active vs passive,
   and why "orientation" needs care.
2. [Rotation matrices](02_rotation_matrices.md) — the 3×3 workhorse and the
   group SO(3).
3. [Axis–angle](03_axis_angle.md) — the most physical description, and the
   gateway to `exp`/`log`.
4. [Quaternions](04_quaternions.md) — the compact, drift-free storage we
   actually use, and the `q = −q` sign trap.
5. [Rigid transforms](05_rigid_transforms.md) — gluing rotation and translation
   into SE(3).

Then: [exercises](exercises.md).

## Prerequisites

You should be comfortable with vectors and matrix multiplication. If not, watch
3Blue1Brown's *Essence of Linear Algebra* (chapters 1–4) first — see
[chapter 02](../02_linear_algebra/README.md) for the link.

## Where this lives in the library

| Concept | File |
| ------- | ---- |
| Rotations (SO(3)) | [`include/tinyspatial/liegroup/so3.hpp`](../../include/tinyspatial/liegroup/so3.hpp) |
| Rigid transforms (SE(3)) | [`include/tinyspatial/liegroup/se3.hpp`](../../include/tinyspatial/liegroup/se3.hpp) |
| Type aliases (Vector3, Matrix3, Quaternion) | [`include/tinyspatial/core/types.hpp`](../../include/tinyspatial/core/types.hpp) |
