# Chapter 07 — URDF: robot models from XML

A robot doesn't fall out of the sky as a `Model` object. Someone — usually
the manufacturer — describes its kinematics and inertias in a plain-text file,
which a parser then turns into the in-memory tree from chapter 06. The
universal format in robotics is **URDF** (Unified Robot Description Format),
and parsing one is what this chapter is about.

## The sub-chapters

1. [XML, in five minutes](01_xml_basics.md) — what an XML element is, what
   attributes are, and how to read one if you've never seen it.
2. [The URDF tags we care about](02_urdf_tags.md) — `<robot>`, `<link>`,
   `<joint>`, `<origin>`, `<axis>`. (And what we deliberately ignore.)
3. [The inertial element](03_inertias.md) — mass, COM, inertia tensor — and
   how that becomes a `SpatialInertia`.
4. [The loader](04_the_loader.md) — three passes from XML to `Model`.
5. [Exploring the three robots](05_exploring_the_robots.md) — load FR3, UR5e,
   and SO-ARM101; count joints; print placements; read the chain.

Then: [exercises](exercises.md).

## Prerequisites

Chapters 03–06. You don't need to know XML — chapter 01 covers it.

## Where this lives in the library

| Concept | File |
| ------- | ---- |
| Loader API | [`include/tinyspatial/urdf/urdf_loader.hpp`](../../include/tinyspatial/urdf/urdf_loader.hpp) |
| Loader impl | [`src/urdf/urdf_loader.cpp`](../../src/urdf/urdf_loader.cpp) |
| Fixture URDFs | [`data/robots/`](../../data/robots) |
| Loader tests | [`tests/unit/urdf/test_urdf_loader.cpp`](../../tests/unit/urdf/test_urdf_loader.cpp) |
| Placement tests | [`tests/unit/urdf/test_urdf_placements.cpp`](../../tests/unit/urdf/test_urdf_placements.cpp) |
| Fuzz tests | [`tests/unit/urdf/test_urdf_fuzz.cpp`](../../tests/unit/urdf/test_urdf_fuzz.cpp) |
