# tinyspatial

A header-mostly C++20 library for rigid-body kinematics and dynamics, covering the parts of [Pinocchio](https://github.com/stack-of-tasks/pinocchio) that robot arms need. It is written to be readable, and comes with a course that uses the code as its lab.

[![ci](https://github.com/adimunot21/tinyspatial/actions/workflows/ci.yml/badge.svg)](https://github.com/adimunot21/tinyspatial/actions/workflows/ci.yml)

## What it implements

| Module | Contents |
|---|---|
| `liegroup/` | SO(3) and SE(3): exp, log, adjoint, left and right Jacobians |
| `spatial/` | Motion and force vectors, spatial inertia |
| `model/` | Model and data, with revolute, prismatic, fixed and floating joints |
| `urdf/` | URDF loader using tinyxml2 and urdfdom_headers |
| `algo/` | Forward kinematics, geometric Jacobian in three frames, RNEA, CRBA, ABA |
| `diff/` | Per-joint Jacobians and analytical RNEA derivatives (Carpentier and Mansard, 2018) |
| `ik/` | Damped least squares IK, task-priority null-space IK, and the derivative of the IK solution with respect to the target pose |

Python bindings are built with nanobind.

## Validation

Every algorithm is compared against Pinocchio 3.9 on 1000 random configurations each of the Franka FR3, UR5e and SO-ARM101, with a tolerance of 1e-10. Most results agree to between 1e-13 and 1e-15. The table is in `docs/PINOCCHIO_PARITY.md`. Pinocchio is used only as a test oracle (`TINYSPATIAL_BUILD_VALIDATION=ON`) and is not a runtime dependency.

## Speed

Single calls on a 7-DoF Franka FR3 (i7-9750H, GCC 13.3, `-O3`), measured with Google Benchmark:

| Algorithm | Calls per second |
|---|---|
| RNEA | 391 K |
| CRBA | 326 K |
| ABA | 90 K |
| Forward kinematics | 1.70 M |
| Jacobian | 1.39 M |

Estimates for Pinocchio's C++ code are about 1.4 to 2.1 times faster. Those figures are estimates, since only the Python-side comparison has been measured so far. The optimisation history is in `docs/BENCHMARKS.md`.

## Automatic differentiation

FK, Jacobian, RNEA, CRBA and ABA are templated on the scalar type. Running them on the header-only forward-mode type `Jet<N>` gives exact derivatives with no finite differencing and no autodiff library.

    using J = tinyspatial::Jet<7>;
    const auto model = tinyspatial::model_cast<J>(franka);
    tinyspatial::DataT<J> data(model);

    VectorXj q(7);
    for (int k = 0; k < 7; ++k) q(k) = J(q0(k), k);

    tinyspatial::rnea(model, data, q, v, a, tau);
    // tau(r).a is the torque, tau(r).v[k] is d tau_r / d q_k

The results match the hand-written analytical derivatives. Autodiff dtau/dq agrees with the Carpentier-Mansard recursion, dtau/da equals the CRBA mass matrix, and dqdd/dtau equals its inverse. See `src/examples/differentiable_dynamics.cpp`.

## Building

    git clone https://github.com/adimunot21/tinyspatial.git
    cd tinyspatial
    git submodule update --init --recursive
    cmake --preset=debug
    cmake --build build/debug -j
    ctest --preset=debug --output-on-failure

158 unit tests. A fresh Ubuntu machine needs `build-essential cmake git`. Eigen 3.4, GoogleTest, Google Benchmark, tinyxml2, urdfdom_headers and nanobind are pinned submodules under `third_party/`. There is no Boost or ROS dependency.

## Usage

C++:

    Model model = build_model_from_urdf_file("data/robots/franka_fr3.urdf");
    Data data(model);
    VectorX q = VectorX::Random(model.nq());
    VectorX v = VectorX::Random(model.nv());
    VectorX a = VectorX::Random(model.nv());
    VectorX tau = VectorX::Zero(model.nv());
    rnea(model, data, q, v, a, tau);

Python:

    import tinyspatial as ts
    model = ts.build_model_from_urdf_file("data/robots/franka_fr3.urdf")
    result = ts.solve_ik_nullspace(model, ee_link, target_pose, q_init=q0, q_rest=q_rest)
    dq_dT = ts.ik_implicit_derivative(model, ee_link, result.q, damping=1e-3)

Notebooks in `python/examples/` cover a UR5e forward kinematics tour, an RNEA comparison with Pinocchio, and IK with null-space elbow control.

## Course

`course/` has 16 chapters, from the basics of programming through Lie groups, spatial algebra, dynamics and differentiable inverse kinematics. Online at [adimunot21.github.io/tinyspatial](https://adimunot21.github.io/tinyspatial/).

## Layout

    include/tinyspatial/   public headers
    src/                   URDF parser, Python binding code, examples
    tests/                 unit tests and Pinocchio validation
    benchmarks/            Google Benchmark
    python/                Python package and notebooks
    data/robots/           URDFs for Franka FR3, UR5e, SO-ARM101, simple_arm
    course/
    docs/                  ARCHITECTURE.md, BENCHMARKS.md, PINOCCHIO_PARITY.md

Version 0.1.0, Apache-2.0. See `CHANGELOG.md`.
