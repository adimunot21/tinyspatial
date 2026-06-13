# Development guide

How to build, test, validate, and contribute to `tinyspatial`. If you have never
opened a terminal, start instead at
[`course/00_welcome/02_setup_your_machine.md`](../course/00_welcome/02_setup_your_machine.md);
this page assumes you are comfortable with a shell and CMake.

## Prerequisites

A fresh Ubuntu box needs only:

```bash
sudo apt install build-essential cmake git
```

Everything else (Eigen, GoogleTest, Google Benchmark, tinyxml2, urdfdom_headers,
nanobind) is vendored as a pinned git submodule. After cloning:

```bash
git submodule update --init --recursive
```

## Build & test

The presets in `CMakePresets.json` are the supported configurations.

```bash
# Configure (writes build/<preset>/)
cmake --preset=debug          # Debug + ASan + UBSan — use this for development
cmake --preset=release        # -O3, enables benchmarks

# Build
cmake --build build/debug -j

# Run the unit tests
ctest --preset=debug --output-on-failure
```

`debug` is the canonical development build: it turns on AddressSanitizer and
UndefinedBehaviorSanitizer, so memory and UB bugs surface immediately. Run it
before every commit.

## Validate against Pinocchio

The parity suite needs a Python environment with Pinocchio 3.9. It is **not**
part of the default build (its toolchain must never contaminate the standalone
C++ build), so it lives behind its own preset.

```bash
# One-time: a venv with the pinned oracle.
python -m venv .venv
.venv/bin/pip install -r tests/validation/requirements.txt

# Build the binding + run the cross-check (regenerates docs/PINOCCHIO_PARITY.md).
. .venv/bin/activate
cmake --preset=validation
cmake --build build/validation -j
ctest --preset=validation -L pinocchio_parity --output-on-failure
```

CI runs this automatically (`.github/workflows/validation.yml`) on any PR that
touches `include/`, `src/`, or `tests/validation/`, and comments the parity
table on the PR. **Never relax the `1e-10` tolerance to make a test pass** — find
the bug, or if it is a genuine convention difference, document it in
`PINOCCHIO_PARITY.md` and convert in the *test*, not the library.

## Benchmark

```bash
cmake --preset=release
cmake --build build/release -j
./build/release/benchmarks/bench_rnea --benchmark_min_time=1s
```

Tracked numbers live in [`BENCHMARKS.md`](BENCHMARKS.md). If you change anything
in `algo/`, re-run the relevant benchmark and note the delta in your PR.

## Style & linting

```bash
# Formatting is enforced in CI. The .clang-format file is the source of truth.
clang-format --dry-run --Werror $(git ls-files '*.hpp' '*.cpp')

# clang-tidy is advisory but warnings should be zero.
clang-tidy -p build/debug <changed-files>
```

The house style, in one paragraph: C++20; `PascalCase` types,
`snake_case` functions/variables, `kCamelCase` constants; no `using namespace`
at file scope; pass Eigen inputs by `const Eigen::Ref<const MatT>&` and outputs
by `Eigen::Ref<MatT>`; fixed-size Eigen types wherever the dimension is known;
no `auto` return types on the public API; Doxygen (`///` with `\param`,
`\returns`, `\pre`) on public declarations only.

## The dual-track rule

The repo serves two audiences and they must not bleed:

- **Library** (`include/`, `src/`, `tests/`) — terse, production-grade. **No
  tutorial prose in headers.**
- **Course** (`course/`) — beginner tutorial. Explanations that don't belong in
  a header go here.

Every new public type or function needs a home in a course chapter and a "Where
this lives in the library" cross-reference. Adding an algorithm means: design →
library code → tests → the course-chapter section. The course is part of the
deliverable, not a follow-up.

## Adding an algorithm — the standard flow

1. **Design first.** For anything above ~50 lines, sketch the approach (frames,
   conventions, which Featherstone/paper algorithm) before writing code.
2. **Library code** in the right `include/tinyspatial/` subdirectory, following
   the dependency direction (`core ← liegroup ← spatial ← model ← algo`).
3. **Unit tests** in `tests/unit/`, mirroring the include layout. Test real
   numbers: hand-computed values, cross-algorithm identities (e.g. CRBA columns
   from RNEA unit vectors), finite-difference checks for derivatives.
4. **Validation** entry in `tests/validation/test_kinematics.py` if it has a
   Pinocchio counterpart.
5. **Course chapter** section + cross-reference table.
6. **Benchmark** if it's on a performance-relevant path.

## Git & PRs

- Trunk-based. `main` is always green. Branch as `feat/<topic>` or `fix/<topic>`.
- **Conventional Commits**, e.g. `feat(liegroup): add SE(3) logarithm`,
  `fix(rnea): correct gravity sign`, `docs(course): chapter 4 draft`.
- Keep PRs small (target < 400 LoC). An algorithm + its tests + its course
  chapter is one PR. Squash on merge.
- Never force-push `main`; `--force-with-lease` is fine on feature branches.

## Where to look when stuck

In priority order: re-read the corresponding Pinocchio source
(it is the reference), then Featherstone (2008) for dynamics or Lynch & Park
(2017) for kinematics conventions, then open a focused issue. Do **not** fabricate
a workaround, comment out a failing test, or relax a numerical tolerance.
