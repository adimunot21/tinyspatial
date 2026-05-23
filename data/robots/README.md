# `data/robots/` — fixture URDFs

This directory holds URDF files consumed by the URDF-loader tests, by future
forward-kinematics and dynamics validation, and by the course exercises.

## ⚠️ Status: SYNTHETIC fixtures (Phase 3)

Every URDF currently shipped here is a hand-written **synthetic** model
derived from publicly-documented kinematic parameters (joint counts, axes, and
link lengths). They are **not** drop-in replacements for the upstream URDFs
maintained by Franka Robotics, Universal Robots, or The Robot Studio. They
are sufficient to exercise the URDF parser, model construction, and the
Phase 3 acceptance criterion ("all three fixture URDFs load without errors").

**Why synthetic for now.** The upstream URDFs are usually distributed as
`.urdf.xacro` files that require the ROS xacro preprocessor; we explicitly do
not support xacro (CLAUDE.md §3 Phase 3 risks). Resolving xacro and obtaining
permissively-licensed expanded URDFs is a small project of its own, and is
deferred to Phase 4, where the Pinocchio cross-validation will require URDFs
that produce the same kinematics in both libraries.

## Fixtures

| File | DOF | Notes |
| ---- | --- | ----- |
| `simple_arm.urdf` | 2 | Tiny test fixture: 2 revolute joints about z, unit-length links, 1 kg point masses at the link tips. Used by `tests/unit/urdf/test_urdf_loader.cpp` and the course-06 exercises. |
| `franka_fr3.urdf` | 7 | Synthetic chain with FR3-like link lengths and axes. Joint origins approximate the link-to-link transforms from Franka Robotics' published DH parameters. |
| `ur5e.urdf` | 6 | Synthetic chain matching UR5e's joint count, with link lengths `d1=0.1625`, `a2=-0.425`, `a3=-0.3922`, `d4=0.1333`, `d5=0.0997`, `d6=0.0996` from Universal Robots' public documentation. |
| `so_arm101.urdf` | 5 + 1 fixed | Synthetic 5-DOF educational arm with link lengths loosely matching The Robot Studio's SO-ARM100/101 specification. The gripper link is a fixed joint (the finger DOF is not part of the kinematic chain we exercise). |

## Phase 4 swap plan

When the Pinocchio validation harness arrives, these URDFs will be replaced
(file-by-file, in their own PRs) with expanded versions of the upstream
xacro-derived URDFs, each cleared against its license and documented here with
the source URL and SHA. Until then, **do not** treat the numbers in these
files as authoritative.

## What URDF features the loader supports

A pure-XML subset, no xacro. See `include/tinyspatial/urdf/urdf_loader.hpp` for
the full contract. Specifically:

- `<robot name>` (root)
- `<link name>` with optional `<inertial>` (origin, mass, inertia matrix)
- `<joint name type>` with `<parent link>`, `<child link>`, `<origin>`, `<axis>`
- Joint types: `revolute`, `continuous` (treated as revolute), `prismatic`,
  `fixed`, `floating`
- `<visual>`, `<collision>`, `<transmission>`, `<gazebo>`, and friends are
  silently ignored
- xacro syntax, `<mimic>`, `<planar>`, `<screw>` will throw `UrdfParseError`

## Meshes

Meshes referenced by URDF `<visual>` / `<collision>` blocks are intentionally
not loaded (we are not a renderer) and intentionally not tracked in git (see
top-level `.gitignore`).
