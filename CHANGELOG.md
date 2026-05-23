# Changelog

All notable changes to this project are documented here. The format follows
[Keep a Changelog](https://keepachangelog.com/en/1.1.0/), and this project
adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Added

- **Phase 2 — Spatial algebra.**
  - `spatial/motion.hpp`, `spatial/force.hpp`: typed `Motion` (twist) and
    `Force` (wrench), angular-first; SE(3) acts on each via `operator*`
    (`Ad_T` for motions, `Ad_T^{-T}` for forces).
  - `spatial/cross.hpp`: typed `cross(Motion, Motion)` and `cross(Motion, Force)`
    overloads alongside the existing matrix forms.
  - `spatial/inertia.hpp`: `SpatialInertia` with separable storage (mass / COM /
    inertia about COM), `matrix6()` for the 6×6 form, composite-body
    `operator+`, and an SE(3) transform that moves each parameter cleanly.
  - `spatial/plucker.hpp`: `motion_plucker()` / `force_plucker()` naming the
    SE(3) adjoint and its dual in Featherstone language.
  - 22 new tests (47 total green, no compiler warnings, clang-tidy clean):
    duality of motion/force adjoints, Jacobi identity for the typed cross,
    kinetic-energy identity for `SpatialInertia`, separable inertia transform
    agrees with the 6×6 congruence, composite-body linearity, Plücker = adjoint.
  - `docs/ALGORITHMS.md`: convention reference for the angular-first / body-fixed
    spatial-algebra layer.
  - Course chapter 05 (5 sub-chapters + exercises).

- **Phase 1 — Lie groups SO(3) and SE(3).**
  - `core/types.hpp`: concrete `double` algebra aliases (Vector3/6, Matrix3/4/6,
    Quaternion). Spatial 6-vectors are angular-first (ω; v).
  - `liegroup/so3.hpp`: `SO3` (canonical w≥0 quaternion) with exp/log, right/left
    Jacobians and inverses, `skew`/`unskew`. Quaternion-based `log`, stable at θ=π.
  - `liegroup/se3.hpp`: `SE3` with exp/log, 6×6 adjoint and inverse, and SE(3)
    group Jacobians via Barfoot's Q matrix (angular-first).
  - `spatial/cross.hpp`: `cross_motion` and `cross_force` spatial cross products.
  - `src/examples/se3_basics.cpp`: worked transform-composition example.
  - 25 unit tests (group axioms, exp/log round-trips to 1e-10, π-angle, FD
    Jacobians, adjoint identity, Jacobi identity). Course chapters 03 & 04.

- **Phase 0 — Bootstrap.** Repo skeleton that compiles, tests, and renders the
  course locally.
  - Top-level `CMakeLists.txt` (header-only `tinyspatial` INTERFACE target) and
    `CMakePresets.json` with `debug` (ASan+UBSan), `release`, and `validation`
    presets.
  - `.gitignore`, Apache-2.0 `LICENSE`, `.clang-format`, `.clang-tidy`,
    `.editorconfig`.
  - `third_party/` submodules pinned by SHA: Eigen 3.4.0, GoogleTest,
    Google Benchmark, tinyxml2, urdfdom_headers, nanobind.
  - Placeholder `include/tinyspatial/version.hpp` + `tests/unit/test_version.cpp`.
  - `docker/` skeleton (builder, runtime, validation-oracle stub).
  - `.github/workflows/ci.yml`: configure → build → test on Ubuntu 22.04 / GCC 12.
  - Course welcome: `course/README.md`, `00_welcome/` chapters, `mkdocs.yml`.
