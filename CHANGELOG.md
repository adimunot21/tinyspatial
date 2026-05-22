# Changelog

All notable changes to this project are documented here. The format follows
[Keep a Changelog](https://keepachangelog.com/en/1.1.0/), and this project
adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Added

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
