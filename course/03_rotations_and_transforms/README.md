# Chapter 03 — Rotations and transforms

Every motion a robot arm performs reduces to moving rigid bodies through space.
Computing *where the end-effector goes* therefore requires a precise, unambiguous
description of orientation and position. This chapter establishes it: the four
representations of a rotation, why each exists, and how rotation and translation
combine into a single rigid transform.

## The sub-chapters

1. [What is a rotation?](01_what_is_a_rotation.md) — the idea, active versus
   passive, and why orientation requires care.
2. [Rotation matrices](02_rotation_matrices.md) — the $3\times3$ workhorse and
   the group SO(3).
3. [Axis–angle](03_axis_angle.md) — the most physical description, and the
   gateway to `exp`/`log`.
4. [Quaternions](04_quaternions.md) — the compact, drift-free storage the library
   uses, and the $q = -q$ sign ambiguity.
5. [Rigid transforms](05_rigid_transforms.md) — combining rotation and
   translation into SE(3).
6. [In code: `SO3T`](06_in_code.md) — the rotation type as implemented:
   quaternion storage, canonicalisation, and the accessors.

Then: [exercises](exercises.md).

## Prerequisites

Vectors and matrix multiplication, per [Chapter 02](../02_linear_algebra/README.md).

## Where this lives in the library

| Concept | File · symbol |
| ------- | ------------- |
| Rotations (SO(3)) | [`liegroup/so3.hpp`](../../include/tinyspatial/liegroup/so3.hpp) · `SO3T`, `SO3` |
| Rigid transforms (SE(3)) | [`liegroup/se3.hpp`](../../include/tinyspatial/liegroup/se3.hpp) · `SE3T`, `SE3` |
| Type aliases (`Vector3`, `Matrix3`, `Quaternion`) | [`core/types.hpp`](../../include/tinyspatial/core/types.hpp) · `Types<S>` |
