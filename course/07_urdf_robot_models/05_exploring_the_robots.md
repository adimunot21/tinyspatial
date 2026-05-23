# Exploring the three robots

`data/robots/` ships three fixture URDFs we'll see again and again. Get them
loaded into your head now and the dynamics chapters will read faster.

> **Status check.** The shipped URDFs are *synthetic*: hand-written from
> publicly documented kinematic parameters. They have the right DOF counts
> and plausible link lengths, but the exact joint origins and inertias do
> not match the official upstream URDFs from Franka, Universal Robots, or
> The Robot Studio. They are sufficient for parser tests and for the course.
> Phase 4 (Pinocchio cross-validation) will swap them for upstream-derived
> URDFs as part of that work. See [`data/robots/README.md`](../../data/robots/README.md).

## Franka FR3 — 7 DOF

```cpp
const Model fr3 = build_model_from_urdf_file("data/robots/franka_fr3.urdf");

// 7 revolute joints, single serial chain.
assert(fr3.njoints() == 7);
assert(fr3.nq() == 7);
assert(fr3.nv() == 7);
```

The Franka FR3 is a 7-DOF research arm. Seven revolute joints in a serial
line means a single chain — `parent[i] == i-1` for all `i ≥ 1`. The extra
DOF over a 6-DOF arm gives it kinematic redundancy: there's a 1-parameter
family of arm configurations that put the gripper in the same pose, which
is useful for avoiding obstacles in the wrist.

## UR5e — 6 DOF

```cpp
const Model ur5e = build_model_from_urdf_file("data/robots/ur5e.urdf");
assert(ur5e.njoints() == 6);
assert(ur5e.find_joint("elbow_joint") == 2);
```

The UR5e is a 6-DOF industrial arm — three "shoulder" joints near the base
to position the wrist in space, and three "wrist" joints near the gripper to
orient it. The classic 6-DOF *spherical wrist* design: the wrist's last
three axes intersect at a single point, which makes inverse kinematics
analytically solvable. We'll exploit that fact in chapter 12.

## SO-ARM101 — 5 DOF arm + fixed gripper

```cpp
const Model so = build_model_from_urdf_file("data/robots/so_arm101.urdf");
assert(so.njoints() == 6);
assert(so.nq() == 5);  // the gripper is a fixed joint — 0 DOF
```

The SO-ARM101 is an educational 3-D-printed arm: shoulder pan, shoulder
lift, elbow, wrist pitch, wrist roll. The end is a fixed mount for a
gripper (the gripper's internal finger DOF is separately modelled in the
upstream URDF; we elide it here). It's a great robot for the dynamics
chapters because the masses are small, the geometry is intuitive, and
nothing about it is hidden behind a license wall.

## Why three robots and not one?

- **FR3 (7-DOF)** exercises the *redundancy* path: forward kinematics has a
  one-parameter null space, and IK has to pick from a family of solutions.
- **UR5e (6-DOF, spherical wrist)** exercises the *closed-form IK* path:
  the analytic solver in chapter 12 keys on this geometry.
- **SO-ARM101 (mixed types)** exercises the *fixed joint* and small-scale
  cases, and gives us a robot we can actually 3-D-print and play with.

The Pinocchio validation suite (Phase 4) runs against all three to ensure
no convention bug only shows up at certain link counts or wrist topologies.

## Try it

A minimal program that prints each robot's name and DOF count:

```cpp
#include "tinyspatial/urdf/urdf_loader.hpp"
#include <iostream>

int main() {
  using tinyspatial::build_model_from_urdf_file;
  for (auto path : {"data/robots/franka_fr3.urdf",
                    "data/robots/ur5e.urdf",
                    "data/robots/so_arm101.urdf"}) {
    auto m = build_model_from_urdf_file(path);
    std::cout << m.name << ": njoints=" << m.njoints()
              << " nq=" << m.nq() << "\n";
  }
}
```

(Build it like `se3_basics.cpp` — add to `src/examples/CMakeLists.txt`.)

## Where this lives in the library

| Concept | File |
| ------- | ---- |
| The three URDFs | [`data/robots/franka_fr3.urdf`](../../data/robots/franka_fr3.urdf), [`ur5e.urdf`](../../data/robots/ur5e.urdf), [`so_arm101.urdf`](../../data/robots/so_arm101.urdf) |
| Provenance | [`data/robots/README.md`](../../data/robots/README.md) |
| Per-robot tests | [`test_urdf_loader.cpp`](../../tests/unit/urdf/test_urdf_loader.cpp), [`test_urdf_placements.cpp`](../../tests/unit/urdf/test_urdf_placements.cpp) |

Next: the [exercises](exercises.md), then [chapter 08 — Forward kinematics](../08_forward_kinematics/README.md).
