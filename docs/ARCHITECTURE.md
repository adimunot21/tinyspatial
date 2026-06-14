# Architecture

For reviewers who want the shape of the codebase before reading it. The math
itself lives in [`ALGORITHMS.md`](ALGORITHMS.md); the conventions that bite are
in [`PINOCCHIO_PARITY.md`](PINOCCHIO_PARITY.md). This file is about *structure*.

## Design philosophy

`tinyspatial` is **header-mostly**. The public target is a CMake `INTERFACE`
library (`add_library(tinyspatial INTERFACE)` in the top-level `CMakeLists.txt`);
everything a downstream user calls is in `include/tinyspatial/`. Only two things
are compiled into objects, both for principled reasons:

| Compiled unit          | Why it isn't a header                                            |
| ---------------------- | --------------------------------------------------------------- |
| `src/urdf/urdf_loader.cpp` | URDF parsing pulls in tinyxml2; no reason to inline it into every TU. |
| `src/bindings/main.cpp`    | nanobind glue is heavy template instantiation; keep it off the library's compile path. |

This is a deliberate credibility signal: the standalone
library has **no Boost, no ROS, no conda**, and compiles against only a vendored
Eigen. The dependency surface is the README's dependency table and nothing else.

## The two-layer type story

Everything is built on concrete fixed-size Eigen aliases declared once in
`include/tinyspatial/core/types.hpp`:

```
Scalar   = double
Vector3, Matrix3, Vector6, Matrix6   // fixed-size, known at compile time
VectorX, MatrixX, Matrix6X           // dynamic, ONLY at the kinematic-tree boundary
Quaternion
```

Fixed-size types are used everywhere the dimension is known;
dynamic sizing appears only where the number of joints is a runtime property
(`Model`/`Data`, FK/Jacobian outputs). This split is what keeps the hot path
allocation-free.

> **Forward pointer.** A planned change templates these aliases on `Scalar`
> (keeping `double` the default) so the same algorithms can run on an autodiff
> dual type. See the roadmap; nothing in this document changes for the `double`
> path.

## Module map

```
include/tinyspatial/
├── core/        types.hpp            — the alias layer above
├── liegroup/    so3.hpp, se3.hpp     — exp/log, adjoint, left/right Jacobians
├── spatial/     motion, force,       — twists, wrenches, Plücker transforms,
│                inertia, cross,        spatial inertia, spatial cross products
│                plucker
├── model/       joint.hpp,           — joint variants (std::variant), the
│                model.hpp              kinematic tree, the Model/Data split
├── urdf/        urdf_loader.hpp      — declaration; impl in src/urdf/
├── algo/        forward_kinematics,  — the kinematics + dynamics algorithms
│                jacobian, rnea,
│                aba, crba
├── diff/        fk_derivatives,      — analytical derivatives
│                rnea_derivatives        (Carpentier–Mansard 2018)
└── ik/          dls, nullspace,      — damped least squares, task-priority
                 differentiable          null-space, implicit-function ∂q*/∂T*
```

The dependency direction is strictly downward: `core` ← `liegroup` ← `spatial`
← `model` ← `algo` ← `diff`/`ik`. Nothing reaches back up.

## The Model / Data split

This mirrors Pinocchio and is the single most important structural decision.

- **`Model`** is the *constant* kinematic tree: parent indices, joint
  placements (`SE3` from parent to joint frame), link spatial inertias, and
  pre-computed per-joint motion subspaces `S_i`. Built once (from a URDF) and
  never mutated by an algorithm.
- **`Data`** is the *per-configuration scratchpad*: per-link poses, velocities,
  accelerations, and the working buffers each algorithm needs. Sized from the
  `Model` once, then reused across calls.

Algorithms take `(const Model&, Data&, q, ...)`. Reusing one `Data` across calls
is what makes repeated evaluation (an IK loop, a benchmark) allocation-free. The
Python bindings allocate `Data` internally per call for ergonomics; C++ callers
should hoist it.

A structural invariant relied on throughout: **`parent[i] < i`** (topological
order). The URDF loader establishes this via a BFS from the root link, so every
algorithm can sweep `0 → n` outward and `n → 0` inward without a separate sort.

## Conventions that will trip you up

These are the things that diverge from other libraries; get them wrong and
Pinocchio parity breaks:

- **Angular-first 6-vectors.** Spatial vectors are `(ω; v)` / `(τ; f)` — angular
  part first. Pinocchio is linear-first. The validation harness permutes
  Pinocchio's rows with `[[0,I],[I,0]]` before comparing; this is a documented
  convention difference, never patched in the library.
- **Body-fixed (local) frame** for spatial quantities, following Featherstone
  (2008). If a derivation disagrees with Pinocchio, suspect a frame convention
  before a bug.
- **Quaternion `w ≥ 0`.** `SO3` stores a quaternion canonicalised to the
  positive-real-part hemisphere, resolving the `q ↔ −q` ambiguity. Matches
  Pinocchio.
- **Type-separated Motion vs Force.** Adding a twist to a wrench is a *compile
  error*, not silent nonsense — the duality is encoded in the type system.

## Error handling

- **Programmer errors** (precondition violations: mismatched sizes, invalid
  joint indices) are preconditions, not runtime-checked. Documented with `\pre`.
- **User-input errors** (malformed URDF) throw `UrdfParseError` at the parse
  boundary, with a descriptive message; exceptions never cross into algorithm
  hot paths.
- **Iterative best-effort** results (IK convergence) return a result struct with
  a `converged` flag rather than `std::expected`, because "ran N iterations,
  here's the best answer and whether it met tolerance" is the honest shape of
  the operation. This is the one documented exception to the
  `std::expected`-at-the-boundary rule.

## Validation as architecture

Correctness is not asserted, it is *measured*. `tests/validation/` loads each
fixture URDF into both `tinyspatial` and Pinocchio 3.9, samples 1000 random
configurations from a fixed seed, runs all eight algorithms in both, and writes
the max-abs-diff table to [`PINOCCHIO_PARITY.md`](PINOCCHIO_PARITY.md). The
`validation.yml` workflow runs this on every PR that touches the algorithms and
comments the table. The oracle (Pinocchio) lives behind
`TINYSPATIAL_BUILD_VALIDATION=ON` and never enters the standalone build — its
toolchain is kept in a separate environment on purpose.

## Build targets at a glance

| Target                 | Kind        | Contains                                  |
| ---------------------- | ----------- | ----------------------------------------- |
| `tinyspatial`          | INTERFACE   | the whole public header API               |
| `tinyspatial_urdf`     | STATIC      | the URDF loader (links tinyxml2_vendor)   |
| `tinyxml2_vendor`      | STATIC      | vendored tinyxml2, SYSTEM-included        |
| `_tinyspatial`         | nanobind    | the Python extension (validation/Python)  |
| `tinyspatial_warnings` | INTERFACE   | `-Wall -Wextra -Wpedantic` for *our* TUs  |

Warning flags are applied only to our translation units, never to `third_party`.
