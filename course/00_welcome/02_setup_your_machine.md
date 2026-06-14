# Build and test the library

This page takes the repository from a fresh clone to a green test run. It assumes
a working command line and basic Git. Two paths are given: a direct toolchain
install (Path A) and a containerised build (Path B). Path A is the default on
Linux; Path B is preferred on Windows or where full isolation is wanted.

## Path A — direct toolchain (Linux)

The build needs three tools: a C++20 compiler (GCC 12+ or Clang 17+), CMake 3.20+,
and Git. On Ubuntu/Debian:

```bash
sudo apt update
sudo apt install -y build-essential cmake git
```

Verify the versions:

```bash
g++ --version      # 12 or higher
cmake --version    # 3.20 or higher
git --version
```

## Path B — Docker (any OS)

[Install Docker](https://docs.docker.com/get-docker/) and confirm it runs:

```bash
docker --version
```

The image carries its own toolchain; no compiler or CMake is required on the host.

## Get the code

The third-party dependencies (Eigen, GoogleTest, and others) are pinned as Git
submodules and must be fetched once after cloning:

```bash
git clone https://github.com/adimunot21/tinyspatial.git
cd tinyspatial
git submodule update --init --recursive
```

## Build and test

### Path A

```bash
cmake --preset=debug                       # configure -> build/debug
cmake --build build/debug -j               # compile (all cores)
ctest --preset=debug --output-on-failure   # run the unit tests
```

The configure → build → test loop is the same one every later chapter uses. The
debug preset enables AddressSanitizer and UndefinedBehaviorSanitizer; use
`--preset=release` for an optimised build (required for the benchmarks).

### Path B

```bash
docker build --target builder -t tinyspatial:builder .
```

The `builder` stage configures, compiles, and runs the unit tests inside the
container. A clean exit means the toolchain and tests are working.

## Troubleshooting

- **`command not found`** — the tool is not installed, or the shell predates the
  install. Re-check the install step and open a fresh shell.
- **`cmake: version too old`** — CMake 3.20+ is required. On older Ubuntu, install
  from [Kitware's APT repository](https://apt.kitware.com/) or use Docker.
- **Empty submodule directories** — run `git submodule update --init --recursive`.
- **Build or test failure** — open an issue with the exact command and the exact
  error output.

Next: [Chapter 03 — Rotations and transforms](../03_rotations_and_transforms/README.md).

---

### Where this lives in the library

| Concept | Where it lives |
| ------- | -------------- |
| Build configuration | [`CMakeLists.txt`](../../CMakeLists.txt), [`CMakePresets.json`](../../CMakePresets.json) |
| The version smoke test | [`tests/unit/test_version.cpp`](../../tests/unit/test_version.cpp) |
| Vendored dependencies | [`third_party/`](../../third_party) |
| Docker setup | [`docker/`](../../docker) |
