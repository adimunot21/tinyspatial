# The joint variants

`tinyspatial` supports four joint types, captured as four small `struct`s:

| Type | DOF | What it does |
| ---- | --- | ------------ |
| `JointRevolute` | 1 | Rotates by `q` radians about a unit axis |
| `JointPrismatic` | 1 | Translates by `q` along a unit axis |
| `JointFixed` | 0 | Welded — used for non-actuated URDF connections |
| `JointFloating` | 6 | Free 6-DOF (e.g. a humanoid's base in the world) |

These are *not* polymorphic classes with virtual methods. They are concrete
value types unified by `std::variant`:

```cpp
using Joint = std::variant<JointFixed,
                           JointRevolute,
                           JointPrismatic,
                           JointFloating>;
```

## Why std::variant, not virtual dispatch

A revolute joint stores a 3-vector. A floating joint stores nothing (its
state lives in `Model::idx_q`). Forcing them through a common abstract base
class would require pointer indirection, heap allocation, vtable dispatch on
every joint-transform call, and — worst — would hide the type at compile
time, blocking the optimiser.

`std::visit` over a variant does the dispatch at **compile time**. The
optimiser can see "this branch is a `JointRevolute`," inline the
`exp(axis * q)` math, and generate tight code. The four types together
total a few hundred bytes; you store them by value in `std::vector<Joint>`
without any allocator drama.

## The free-function interface

Most code shouldn't even *see* the variant. We expose three free functions
that hide the visit:

```cpp
int nq(const Joint& j);
int nv(const Joint& j);
SE3 joint_transform(const Joint& j, Eigen::Ref<const VectorX> q_slice);
```

`nq` is the number of configuration coordinates this joint contributes; `nv`
the number of velocity coordinates. They're equal for revolute and
prismatic, both `7`/`6` for floating (because the 6-DOF velocity is a twist
but the 7-number configuration is `(translation, quaternion)`), and both `0`
for fixed. Algorithms walk `Model::idx_q[i]` and `idx_v[i]` to slice the
global vectors into per-joint pieces.

## `JointFloating` and the configuration layout

This is the one place where the configuration storage matters. We follow
Pinocchio:

```
q = (t_x, t_y, t_z,  q_x, q_y, q_z, q_w)   // 7 numbers
v = (ω_x, ω_y, ω_z,  v_x, v_y, v_z)        // 6 numbers, angular-first
```

The translation is plain. The quaternion is stored *scalar-last*
(`q_w` at index 6), matching most URDF / ROS conventions. The velocity
honours our angular-first convention from chapter 04.

You will eventually need to *integrate* a velocity into a configuration —
that's `q ← q ⊕ v` for a small `v`. For the floating joint that step is more
than addition (you have to integrate the quaternion correctly). The
ingredients are exactly what `SO3::exp` and `SE3::exp` give you, and the IK
chapter (12) will wire it up.

## What's *not* in the variant

URDF supports a few more joint types we don't model:

- `planar` — 3 DOF in a plane. Rare in practice and decomposable.
- `screw` (helical) — coupled rotation and translation. Niche.
- `mimic` — one joint copies another's value. A composition layer, not a
  primitive; the loader rejects it.

If you need one, add it to the variant and to `joint_transform`'s
`if constexpr` ladder. The change is local; no algorithm cares beyond that.

## Where this lives in the library

| Concept | File · symbol |
| ------- | ------------- |
| Joint variants | [`joint.hpp`](../../include/tinyspatial/model/joint.hpp) · `JointRevolute`, `JointPrismatic`, `JointFixed`, `JointFloating` |
| Variant alias | `Joint` |
| Free-function interface | `nq()`, `nv()`, `joint_transform()` |
| Per-variant tests | [`test_joint.cpp`](../../tests/unit/model/test_joint.cpp) |

Next: [Model and Data](04_model_and_data.md).
