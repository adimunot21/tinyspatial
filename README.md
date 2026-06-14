# tinyspatial

> A header-mostly **C++20** rigid-body kinematics and dynamics library — the
> parts of [Pinocchio](https://github.com/stack-of-tasks/pinocchio) that matter
> for robot arms — built to be **read**, not just used.

[![ci](https://github.com/adimunot21/tinyspatial/actions/workflows/ci.yml/badge.svg)](https://github.com/adimunot21/tinyspatial/actions/workflows/ci.yml)
![C++20](https://img.shields.io/badge/C%2B%2B-20-blue.svg)
![Pinocchio parity](https://img.shields.io/badge/Pinocchio_parity-1e--13-brightgreen.svg)
![License](https://img.shields.io/badge/license-Apache--2.0-green.svg)

`tinyspatial` re-implements SE(3) / SO(3) Lie-group operations, spatial
inertias, Featherstone **RNEA / ABA / CRBA**, geometric and analytical
Jacobians, analytical RNEA derivatives, and damped-least-squares inverse
kinematics with task-priority null-space control — on top of Eigen 3.4,
with nanobind Python bindings.

Every algorithm is **numerically validated against Pinocchio 3.9** to a
tolerance of `1e-10` on 1000 random configurations of the Franka FR3,
UR5e, and SO-ARM101 robots. See [`docs/PINOCCHIO_PARITY.md`](docs/PINOCCHIO_PARITY.md)
for the current parity table — most cells sit at `1e-13` to `1e-15`,
well below the bar.

## Headline numbers

Single-call throughput on a 7-DoF Franka FR3 (Intel i7-9750H, GCC 13.3,
`-O3 -DNDEBUG`):

| Algorithm     | tinyspatial | Pinocchio C++ (est.) | ratio  |
| ------------- | ----------: | -------------------: | -----: |
| **RNEA**      |   **391 K / s** |             ~625 K / s |  1.6×  |
| **CRBA**      |   **326 K / s** |             ~590 K / s |  1.8×  |
| **ABA**       |    **90 K / s** |             ~130 K / s |  1.5×  |
| **FK**        |   **1.70 M / s** |            ~3.6 M / s |  2.1×  |
| **Jacobian**  |   **1.39 M / s** |            ~2.0 M / s |  1.4×  |

The **tinyspatial** column is measured (Google Benchmark, median of
~488 K iterations). The **Pinocchio** column is an *estimate* — a measured
C++ head-to-head is tracked as future work; the only number measured against
Pinocchio today is the Python-side comparison in
[`docs/BENCHMARKS.md`](docs/BENCHMARKS.md). In honest terms: tinyspatial sits at
**~1.6× of Pinocchio's C++ RNEA** on a 7-DoF arm — short of the project's
original `≥ 6 M/s` aspiration, which the fixed-size /
compile-time work on the roadmap is the path to closing. The gap, the
optimisation history, and the remaining levers are all documented in
[`docs/BENCHMARKS.md`](docs/BENCHMARKS.md).

## Two things in one repo

This project is both a **library** and a **course**.

- **`/include`, `/src`, `/tests`** — the library. Production-grade C++20 for
  reviewers who want to read clean rigid-body-dynamics code.
- **`/course`** — a from-scratch tutorial that takes someone who has *never
  programmed* from "what is a robot" all the way to differentiable IK. Uses
  this codebase as the lab. Read it online at
  **[adimunot21.github.io/tinyspatial](https://adimunot21.github.io/tinyspatial/)**
  or start locally at [`course/README.md`](course/README.md).

## What's in the box

| Module                                 | Algorithms                                                         |
| -------------------------------------- | ------------------------------------------------------------------ |
| `tinyspatial/liegroup/`                | `SO3`, `SE3`: `exp` / `log`, adjoint, left/right Jacobians         |
| `tinyspatial/spatial/`                 | `Motion`, `Force`, `SpatialInertia`, motion / force Plücker        |
| `tinyspatial/model/`                   | `Model`, `Data`, joint variants (revolute, prismatic, fixed, floating) |
| `tinyspatial/urdf/`                    | URDF loader (tinyxml2 + urdfdom_headers, no Boost)                 |
| `tinyspatial/algo/forward_kinematics`  | FK over the kinematic tree                                          |
| `tinyspatial/algo/jacobian`            | Geometric Jacobian in three frames (LOCAL / WORLD / LOCAL_WORLD_ALIGNED) |
| `tinyspatial/algo/rnea`                | Recursive Newton-Euler inverse dynamics                            |
| `tinyspatial/algo/crba`                | Composite-Rigid-Body mass matrix                                   |
| `tinyspatial/algo/aba`                 | Articulated-Body forward dynamics                                  |
| `tinyspatial/diff/fk_derivatives`      | Per-joint Jacobians sweep                                          |
| `tinyspatial/diff/rnea_derivatives`    | Analytical `∂τ/∂q`, `∂τ/∂v`, `∂τ/∂a` (Carpentier-Mansard 2018)     |
| `tinyspatial/ik/dls`                   | Damped-least-squares IK                                            |
| `tinyspatial/ik/nullspace`             | Task-priority IK with secondary posture objective                  |
| `tinyspatial/ik/differentiable`        | Analytical `∂q*/∂T*` via the implicit function theorem             |

## Differentiable in pure C++

Every kinematics and dynamics algorithm — FK, Jacobian, **RNEA, CRBA, ABA** — is
templated on its scalar type, so running it on a header-only forward-mode
autodiff scalar (`Jet<N>`) yields **exact derivatives with no finite
differencing and no external autodiff library** (no CppAD, no JAX, no PyTorch).
Lift a model, seed the configuration, read the partials out of the result:

```cpp
using J = tinyspatial::Jet<7>;                         // 7-DoF arm
const auto model = tinyspatial::model_cast<J>(franka); // double model -> autodiff
tinyspatial::DataT<J> data(model);

VectorXj q(7);
for (int k = 0; k < 7; ++k) q(k) = J(q0(k), k);        // seed q as the variables

tinyspatial::rnea(model, data, q, v, a, tau);          // the SAME rnea
// tau(r).a    = torque ;  tau(r).v[k] = ∂τ_r/∂q_k     — the exact derivative
```

The autodiff path is cross-checked against the library's hand-written analytical
derivatives to machine precision: AD ∂FK/∂q matches `fk_derivatives` to `1e-10`;
AD ∂τ/∂q matches the Carpentier–Mansard recursion, and AD ∂τ/∂a independently
equals CRBA's `M(q)`; AD ∂q̈/∂τ equals `M(q)⁻¹`. Two independently-derived
derivative paths agreeing — see the runnable
[`src/examples/differentiable_dynamics.cpp`](src/examples/differentiable_dynamics.cpp)
and [course chapter 13.5](course/13_differentiable_ik/05_differentiable_dynamics.md).

## Quick start

```bash
git clone https://github.com/adimunot21/tinyspatial.git
cd tinyspatial
git submodule update --init --recursive   # fetch Eigen, GoogleTest, etc.

cmake --preset=debug                       # configure -> build/debug
cmake --build build/debug -j
ctest --preset=debug --output-on-failure   # run all 158 unit tests
```

A fresh Ubuntu box needs only `sudo apt install build-essential cmake git`.
Never opened a terminal before? Go straight to
[`course/00_welcome/02_setup_your_machine.md`](course/00_welcome/02_setup_your_machine.md).

## C++ example

```cpp
#include "tinyspatial/algo/rnea.hpp"
#include "tinyspatial/urdf/urdf_loader.hpp"

using namespace tinyspatial;

int main() {
  Model model = build_model_from_urdf_file("data/robots/franka_fr3.urdf");
  Data  data(model);

  VectorX q   = VectorX::Random(model.nq());   // configuration
  VectorX v   = VectorX::Random(model.nv());   // velocity
  VectorX a   = VectorX::Random(model.nv());   // desired acceleration
  VectorX tau = VectorX::Zero(model.nv());

  rnea(model, data, q, v, a, tau);             // -> joint torques required
}
```

## Python example

```python
import tinyspatial as ts
import numpy as np

model = ts.build_model_from_urdf_file("data/robots/franka_fr3.urdf")
ee_link = model.njoints - 1

# IK to a Cartesian target with elbow-up bias on the null-space.
target_pose = ts.forward_kinematics(model, np.array([0, -0.4, 0, -1.8, 0, 1.4, 0.7]))[-1]
q_rest = np.array([0.0, -0.5, 0.0, -2.0, 0.0, 1.5, 0.5])
result = ts.solve_ik_nullspace(model, ee_link, target_pose,
                                q_init=np.zeros(model.nq), q_rest=q_rest)
print(f"converged: {result.converged}, iterations: {result.iterations}")

# Analytical d q* / d T* via the implicit function theorem.
dq_dT = ts.ik_implicit_derivative(model, ee_link, result.q, damping=1e-3)
print(f"dq*/dT* shape: {dq_dT.shape}")    # (7, 6) for Franka
```

Three executable Jupyter notebooks in [`python/examples/`](python/examples/):
FK tour on a UR5e, RNEA vs Pinocchio residual histogram on Franka, IK +
null-space elbow control.

## Dependencies

All vendored as pinned git submodules under `third_party/` — the library builds
standalone with no system packages beyond a compiler and CMake:

| Dependency        | Version       | Purpose                       |
| ----------------- | ------------- | ----------------------------- |
| Eigen             | 3.4.0         | Linear algebra (header-only)  |
| GoogleTest        | latest stable | Unit tests                    |
| Google Benchmark  | latest stable | Performance tests             |
| tinyxml2          | v10.x         | URDF XML parsing              |
| urdfdom_headers   | v1.1.x        | URDF type definitions         |
| nanobind          | ≥ 2.0         | Python bindings               |

**Pinocchio 3.9 is used only as a validation oracle**, behind
`TINYSPATIAL_BUILD_VALIDATION=ON`, and never enters the runtime path.
**No Boost. No ROS. No conda.** This is a credibility signal: the standalone
library has no transitive dependency soup.

## Project layout

```
tinyspatial/
├── include/tinyspatial/   # public C++ headers (header-mostly)
├── src/                   # non-header impl (URDF parser, nanobind glue)
├── tests/                 # GoogleTest unit tests + Pinocchio validation
├── benchmarks/            # Google Benchmark perf tests
├── python/                # the Python package (tinyspatial), examples, tests
├── data/robots/           # URDFs for Franka FR3, UR5e, SO-ARM101, simple_arm
├── course/                # 16-chapter from-scratch tutorial
└── docs/                  # ARCHITECTURE.md, BENCHMARKS.md, PINOCCHIO_PARITY.md
```

## Status

Currently at **v0.1.0** (initial release). All planned algorithms
implemented, all Pinocchio parity tests passing at `1e-13` or below.
See [`CHANGELOG.md`](CHANGELOG.md) for the full history.

## License

[Apache-2.0](LICENSE).
