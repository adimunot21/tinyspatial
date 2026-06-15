# Chapter 08 — Forward kinematics

Given a robot — a `Model` from chapter 06 — with its joints set to some
configuration `q`, where does the gripper end up? Where does each link sit in
the world? **Forward kinematics** is the function that answers that. It is the
simplest algorithm in the library: one loop, ten lines, exact at machine
precision.

## The sub-chapters

1. [What forward kinematics is](01_what_fk_is.md) — and what it isn't.
2. [The recursion](02_the_recursion.md) — child = parent · local. One pass
   over a topologically ordered tree.
3. [Walking the tree in code](03_in_code.md) — the actual implementation in
   `forward_kinematics.hpp`.
4. [Validating against Pinocchio](04_validation.md) — what the parity table
   means and why `1e-15` matters more than `1e-10`.

Then: [exercises](exercises.md).

## Prerequisites

Chapters 03–07. Especially chapter 06 (Model / Data) and chapter 04 (exp/log
for joint transforms).

## Where this lives in the library

| Concept | File |
| ------- | ---- |
| FK algorithm | [`include/tinyspatial/algo/forward_kinematics.hpp`](../../include/tinyspatial/algo/forward_kinematics.hpp) |
| Tests | [`tests/unit/algo/test_forward_kinematics.cpp`](../../tests/unit/algo/test_forward_kinematics.cpp) |
| Pinocchio parity | [`docs/PINOCCHIO_PARITY.md`](../../docs/PINOCCHIO_PARITY.md) |
