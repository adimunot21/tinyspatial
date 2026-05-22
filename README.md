# tinyspatial

> A header-mostly **C++20** library for rigid-body kinematics and dynamics — the
> parts of [Pinocchio](https://github.com/stack-of-tasks/pinocchio) that matter
> for robot arms — built to be **read**, not just used.

[![ci](https://github.com/adimunot21/tinyspatial/actions/workflows/ci.yml/badge.svg)](https://github.com/adimunot21/tinyspatial/actions/workflows/ci.yml)
![C++20](https://img.shields.io/badge/C%2B%2B-20-blue.svg)
![License](https://img.shields.io/badge/license-Apache--2.0-green.svg)

`tinyspatial` re-implements SE(3)/SO(3) Lie-group operations, spatial inertias,
Featherstone **RNEA / ABA / CRBA**, geometric and analytical Jacobians,
analytical derivatives, and damped-least-squares inverse kinematics — on top of
Eigen 3.4, with nanobind Python bindings. Every algorithm is validated
numerically against **Pinocchio 3.9** to a tolerance of `1e-10` on the Franka
FR3, UR5e, and SO-ARM101 robots.

> **Status:** Phase 0 (bootstrap). The build, test, and CI skeleton is up; the
> algebra lands phase by phase. See [`PROJECT_PLAN.md`](PROJECT_PLAN.md) for the
> roadmap.

## Two things in one repo

This project is both a **library** and a **course**.

- **`/include`, `/src`, `/tests`** — the library. Production-grade C++20 for
  reviewers who want to read clean rigid-body-dynamics code.
- **`/course`** — a from-scratch tutorial that takes someone who has *never
  programmed* from "what is a robot" all the way to writing a Jacobian, using
  this codebase as the lab. Start at [`course/README.md`](course/README.md).

## Quick start

```bash
git clone https://github.com/adimunot21/tinyspatial.git
cd tinyspatial
git submodule update --init --recursive   # fetch Eigen, GoogleTest, etc.

cmake --preset=debug                       # configure -> build/debug
cmake --build build/debug -j
ctest --preset=debug --output-on-failure   # run the unit tests
```

A fresh Ubuntu box needs only `sudo apt install build-essential cmake git`.
Never opened a terminal before? Go straight to
[`course/00_welcome/02_setup_your_machine.md`](course/00_welcome/02_setup_your_machine.md).

## Dependencies

All vendored as pinned git submodules under `third_party/` — the library builds
standalone with no system packages beyond a compiler and CMake:

| Dependency        | Version  | Purpose                       |
| ----------------- | -------- | ----------------------------- |
| Eigen             | 3.4.0    | Linear algebra (header-only)  |
| GoogleTest        | pinned   | Unit tests                    |
| Google Benchmark  | pinned   | Performance tests             |
| tinyxml2          | v10.x    | URDF XML parsing              |
| urdfdom_headers   | v1.1.x   | URDF type definitions         |
| nanobind          | ≥ 2.0    | Python bindings               |

Pinocchio 3.9 is used **only** as a validation oracle, behind
`TINYSPATIAL_BUILD_VALIDATION=ON`, and never enters the runtime path.

## License

[Apache-2.0](LICENSE).
