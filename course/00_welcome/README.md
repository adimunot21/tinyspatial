# Chapter 0 — Welcome

This chapter fixes scope: what the library computes, what the course covers, and
how the two are linked.

## What this is

`tinyspatial` is a header-mostly C++20 library for rigid-body kinematics and
dynamics — the math a robot arm needs in order to move. It answers four
questions: where is the end-effector now, how fast is it moving, what joint
torques produce a given motion, and — inverting the first — what joint angles
place the end-effector at a target pose. The library is deliberately small: the
entire implementation is meant to be read end to end.

The course is the reference manual for that library. It develops the underlying
robotics — rotations, spatial algebra, the Featherstone dynamics algorithms —
and then shows exactly how each result is realised in the source. Every chapter
quotes real code and explains the load-bearing lines.

## What the course covers

By the end you will be able to explain, and modify, the core of the library:

- why a rotation has several useful representations, and which the library stores;
- what a twist and a wrench are, and why rigid-body dynamics packs angular and
  linear parts into a single spatial 6-vector;
- how a chain of joints maps to the pose of the end-effector (forward kinematics)
  and how joint rates map to spatial velocity (the Jacobian);
- how the same recursive structure, run in reverse, yields the required joint
  torques (RNEA) and the resulting accelerations (ABA, CRBA);
- and how every result is checked, to `1e-10`, against the reference
  implementation, Pinocchio.

## How the chapters are built

Each chapter pairs theory with implementation. The mathematics is developed in
prose and MathJax; the implementation is quoted directly from `include/` and
dissected line by line. Chapters close with a **"Where this lives in the
library"** table — a direct index from concept to source file and symbol. Keep
the source open alongside the text; the course is written to be read against it.

Next: [What is a robot?](01_what_is_a_robot.md)

---

### Where this lives in the library

| Concept | Where it lives |
| ------- | -------------- |
| Library version / entry point | [`include/tinyspatial/version.hpp`](../../include/tinyspatial/version.hpp) |
| Build configuration | [`CMakeLists.txt`](../../CMakeLists.txt) |
| Public headers (the API surface) | [`include/tinyspatial/`](../../include/tinyspatial) |
