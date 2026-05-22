# The tinyspatial course

Welcome. This is a course about how robots *move*, and how a computer figures
out that movement. It uses a real, working C++ library — the one living in this
very repository — as its laboratory.

You do **not** need to be a programmer to start. You do **not** need to know
linear algebra yet. You need curiosity and a willingness to work small examples
by hand. We'll build the rest together.

## What you'll be able to do by the end

- Read and reason about the math that controls a robot arm.
- Understand rotations, rigid transforms, twists, and wrenches — the vocabulary
  of robotics — and *why* they're built the way they are.
- Follow (and modify) real algorithms: forward kinematics, Jacobians, and the
  Featherstone dynamics algorithms RNEA, ABA, and CRBA.
- Drive the whole thing from Python and check your answers against the
  industry-standard library, Pinocchio.

## How the course is organised

One folder per chapter, numbered in the order you should read them. Each chapter
has a `README.md` (the chapter), an `exercises.md`, and ends with a **"Where
this lives in the library"** table linking the idea to the actual source file.

| Part | Chapters | What it covers |
| ---- | -------- | -------------- |
| **Getting started** | `00_welcome/` | What this is, what a robot is, setting up your machine. |
| **Foundations** | `01_cpp_foundations/`, `02_linear_algebra/` | Mostly curated links to the best free resources, plus framing. |
| **The math of motion** | `03_rotations_and_transforms/`, `04_lie_groups/`, `05_spatial_algebra/` | Rotations, the Lie-group view, spatial 6-vectors. |
| **Robot models** | `06_kinematic_trees/`, `07_urdf_robot_models/` | How a robot is described as data. |
| **Kinematics** | `08_forward_kinematics/`, `09_jacobians/` | Where the hand goes; how joint speed maps to hand speed. |
| **Dynamics** | `10_dynamics_RNEA/`, `11_ABA_and_CRBA/` | Forces, torques, and acceleration. |
| **Inverse kinematics** | `12_inverse_kinematics/`, `13_differentiable_ik/` | Going from a target back to joint angles. |
| **Using & trusting it** | `14_python_bindings/`, `15_validation_vs_pinocchio/`, `16_benchmarking/` | Python, correctness, speed. |
| **Reference** | `99_glossary.md` | Every term, defined. |

## The learning path

```
        ┌─────────────┐
        │ 00 Welcome  │  ← you are here
        └──────┬──────┘
               │
   ┌───────────┴───────────┐
   │ 01 C++   │ 02 Linear  │   (foundations — link out, read as needed)
   │ basics   │ algebra    │
   └───────────┬───────────┘
               │
        ┌──────▼──────────────┐
        │ 03 Rotations &      │
        │    transforms       │
        └──────┬──────────────┘
               │
        ┌──────▼──────┐   ┌──────────────────┐
        │ 04 Lie      │──▶│ 05 Spatial       │
        │    groups   │   │    algebra       │
        └─────────────┘   └────────┬─────────┘
                                   │
                  ┌────────────────▼────────────────┐
                  │ 06 Kinematic trees → 07 URDF     │
                  └────────────────┬─────────────────┘
                                   │
              ┌────────────────────▼────────────────────┐
              │ 08 Forward kinematics → 09 Jacobians     │
              └────────────────────┬─────────────────────┘
                                   │
              ┌────────────────────▼────────────────────┐
              │ 10 RNEA → 11 ABA & CRBA (dynamics)       │
              └────────────────────┬─────────────────────┘
                                   │
              ┌────────────────────▼────────────────────┐
              │ 12 IK → 13 Differentiable IK             │
              └────────────────────┬─────────────────────┘
                                   │
              ┌────────────────────▼────────────────────┐
              │ 14 Python · 15 Validation · 16 Benchmarks│
              └──────────────────────────────────────────┘
```

Ready? Start with [`00_welcome/`](00_welcome/README.md).
