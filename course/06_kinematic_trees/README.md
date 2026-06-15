# Chapter 06 — Kinematic trees

A robot, for the purposes of this library, is a *tree of rigid bodies connected
by joints*. Chapter 01 presented that picture in words. This chapter turns the
picture into a precise data structure — the `Model` — that every algorithm in
the library consumes.

## The sub-chapters

1. [Links and joints, in code](01_links_and_joints.md) — what each one stores
   and why.
2. [The tree](02_the_tree.md) — parent/child indexing, topological order,
   why we keep arrays instead of pointers.
3. [The joint variants](03_joint_variants.md) — revolute, prismatic, fixed,
   floating, and `std::variant` as compile-time dispatch.
4. [Model and Data](04_model_and_data.md) — the constant Model vs the
   per-configuration scratchpad.

Then: [exercises](exercises.md).

## Prerequisites

Chapters 03–05 (rotations, Lie groups, spatial algebra). Chapter 05 is
load-bearing: Featherstone's vocabulary is the language Model speaks.

## Where this lives in the library

| Concept | File |
| ------- | ---- |
| Joint types | [`include/tinyspatial/model/joint.hpp`](../../include/tinyspatial/model/joint.hpp) |
| Model + Data | [`include/tinyspatial/model/model.hpp`](../../include/tinyspatial/model/model.hpp) |
| Tests | [`tests/unit/model/test_joint.cpp`](../../tests/unit/model/test_joint.cpp), [`test_model.cpp`](../../tests/unit/model/test_model.cpp) |
