# Changelog

All notable changes to this project are documented here. The format follows
[Keep a Changelog](https://keepachangelog.com/en/1.1.0/), and this project
adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Added

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
