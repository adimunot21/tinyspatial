# The tinyspatial course

This is a course on rigid-body kinematics and dynamics — how a robot arm moves,
and how a computer computes that motion — taught against a real, working C++
library. The library lives in this repository; the course uses its source as the
reference implementation, quoting and dissecting the actual code rather than
pseudocode.

## Prerequisites

The course assumes:

- **Working C++.** You can read modern C++ (templates, `const` references,
  `struct`/`class`, the standard containers). The library targets C++20 and uses
  `concepts`, `constexpr`, and `std::expected`; these are introduced where they
  appear. You do not need prior template-metaprogramming experience.
- **Linear-algebra fundamentals.** Matrix multiplication, transpose, inverse,
  eigenvalues, and the geometric reading of a matrix as a linear map. Chapter 02
  links to refreshers if needed.
- **No robotics background.** Rotations, rigid transforms, twists, wrenches, Lie
  groups, and the dynamics algorithms are all developed from first principles.

## Scope

By the end you will be able to:

- Reason about the mathematics that governs a robot arm: rotations and rigid
  transforms as elements of SO(3) and SE(3), and the spatial 6-vectors (twists
  and wrenches) that carry velocity, acceleration, and force.
- Read and modify the core algorithms: forward kinematics, the geometric and
  analytical Jacobians, and the Featherstone dynamics algorithms RNEA, ABA, and
  CRBA — including their analytical derivatives.
- Use the library from C++ and Python, and verify every result against the
  reference implementation, Pinocchio.

## Structure

One directory per chapter, numbered in reading order. Each chapter contains a
`README.md` (the chapter text), an `exercises.md`, and a final **"Where this
lives in the library"** table mapping each concept to its source file and the
relevant symbol. Theory chapters additionally carry an **"In code"** section
that quotes the implementation and explains the load-bearing lines.

| Part | Chapters | Coverage |
| ---- | -------- | -------- |
| **Getting started** | `00_welcome/` | Orientation; building and testing the library. |
| **Foundations** | `01_cpp_foundations/`, `02_linear_algebra/` | Curated references for the prerequisites, plus the specific C++20 features the library relies on. |
| **The math of motion** | `03_rotations_and_transforms/`, `04_lie_groups/`, `05_spatial_algebra/` | Rotations and transforms, the Lie-group view, spatial 6-vectors. |
| **Robot models** | `06_kinematic_trees/`, `07_urdf_robot_models/` | The kinematic tree and the URDF format that describes it. |
| **Kinematics** | `08_forward_kinematics/`, `09_jacobians/` | End-effector pose; the map from joint rates to spatial velocity. |
| **Dynamics** | `10_dynamics_RNEA/`, `10b_rnea_derivatives/`, `11_ABA_and_CRBA/` | Inverse and forward dynamics, and the analytical derivatives. |
| **Inverse kinematics** | `12_inverse_kinematics/`, `13_differentiable_ik/` | Solving for joint angles from a target; differentiating through the solver. |
| **Using & trusting it** | `14_python_bindings/`, `15_validation_vs_pinocchio/`, `16_benchmarking/`, `17_interactive_wasm/` | Python bindings, Pinocchio parity, performance, and the in-browser build. |
| **Reference** | `99_glossary.md` | Every term, defined. |

## Reading path

```
        ┌─────────────┐
        │ 00 Welcome  │
        └──────┬──────┘
               │
   ┌───────────┴───────────┐
   │ 01 C++   │ 02 Linear  │   (foundations — reference as needed)
   │ features │ algebra    │
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
              │ 10 RNEA → 10b derivatives → 11 ABA & CRBA│
              └────────────────────┬─────────────────────┘
                                   │
              ┌────────────────────▼────────────────────┐
              │ 12 IK → 13 Differentiable IK             │
              └────────────────────┬─────────────────────┘
                                   │
       ┌───────────────────────────▼───────────────────────────┐
       │ 14 Python · 15 Validation · 16 Benchmarks · 17 WASM    │
       └───────────────────────────────────────────────────────┘
```

Begin with [`00_welcome/`](00_welcome/README.md).
