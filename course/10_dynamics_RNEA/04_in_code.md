# RNEA in code

The whole algorithm lives in [`include/tinyspatial/algo/rnea.hpp`](../../include/tinyspatial/algo/rnea.hpp).
~50 lines of substance, two `for` loops. This sub-chapter is a guided
read-through.

## The signature

```cpp
inline void rnea(const Model& model, Data& data,
                 const Eigen::Ref<const VectorX>& q,
                 const Eigen::Ref<const VectorX>& v,
                 const Eigen::Ref<const VectorX>& a,
                 Eigen::Ref<VectorX> tau,
                 const Vector3& gravity = Vector3(0, 0, -9.81));
```

Inputs:
- `q` — configuration, length `model.nq()`.
- `v` — velocity, length `model.nv()`.
- `a` — desired acceleration, length `model.nv()`.
- `gravity` — world-frame gravitational acceleration; the default is Earth
  with $z$ up.

Outputs:
- `tau` — joint torque, written in place; length `model.nv()`.
- `data.v[i]`, `data.a[i]`, `data.f[i]` — per-body twists, accelerations,
  and net wrenches, filled as a side effect. The next algorithm (CRBA, ABA)
  can reuse them.

The snippets below are shown through the `double` API for readability. In the
source the function is templated — `template <typename S> void rnea(const
ModelT<S>&, DataT<S>&, …)` — so the identical code runs on the autodiff scalar
`Jet` and yields `∂τ/∂q`, `∂τ/∂v`, and `∂τ/∂a` directly (Chapter 13). Read `Model`
for `ModelT<S>`, `Motion` for `MotionT<S>`, and so on.

`Eigen::Ref` lets the caller pass any Eigen expression with the right
shape — a slice, a column of a bigger matrix, etc. — without copying.
The `const Ref<const>` form is the read-only variant: a zero-copy view the
callee promises not to modify.

## The forward kinematics call

```cpp
forward_kinematics(model, data, q);
```

RNEA needs the joints' parent-to-body transforms to propagate twists. Rather
than ask the caller to pre-compute them, we just call FK ourselves. It's
$O(n)$ and the result is exactly what the outward pass needs.

## The two helpers

There are two per-joint operations RNEA needs: "build the joint's velocity
contribution `S · v_slice`" and "project a wrench onto the joint's
subspace, writing `Sᵀ · F`." Both dispatch on the joint variant via
`std::visit`:

```cpp
namespace detail {

inline Motion joint_subspace_motion(const Joint& j,
                                    const Eigen::Ref<const VectorX>& v_slice) {
  return std::visit([&](const auto& jj) -> Motion {
    using JT = std::decay_t<decltype(jj)>;
    if constexpr (std::is_same_v<JT, JointRevolute>) {
      return Motion(jj.axis * v_slice(0), Vector3::Zero());
    } else if constexpr (std::is_same_v<JT, JointPrismatic>) {
      return Motion(Vector3::Zero(), jj.axis * v_slice(0));
    } else if constexpr (std::is_same_v<JT, JointFloating>) {
      return Motion(v_slice.head<3>(), v_slice.tail<3>());
    } else {  // JointFixed
      return Motion::zero();
    }
  }, j);
}
// joint_subspace_project mirrors this with Sᵀ projection.

}  // namespace detail
```

The `std::visit + if constexpr` pattern compiles each branch independently,
so the dispatch is a simple jump table at run time. No virtual calls, no
allocations.

## The outward pass

```cpp
const Motion neg_gravity_world(Vector3::Zero(), -gravity);

for (int i = 0; i < model.njoints(); ++i) {
  const Joint& j = model.joints[i];
  const int joint_nv = nv(j);
  const Motion v_j = detail::joint_subspace_motion(j, v.segment(model.idx_v[i], joint_nv));
  const Motion a_j = detail::joint_subspace_motion(j, a.segment(model.idx_v[i], joint_nv));
  const SE3 i_from_parent = data.pose_in_parent[i].inverse();

  if (model.parent[i] == -1) {
    data.v[i] = v_j;
    data.a[i] = (i_from_parent * neg_gravity_world) + a_j + cross(data.v[i], v_j);
  } else {
    const Motion v_parent_in_i = i_from_parent * data.v[model.parent[i]];
    const Motion a_parent_in_i = i_from_parent * data.a[model.parent[i]];
    data.v[i] = v_parent_in_i + v_j;
    data.a[i] = a_parent_in_i + a_j + cross(data.v[i], v_j);
  }

  const Force inertia_a = model.inertia[i] * data.a[i];
  const Force inertia_v = model.inertia[i] * data.v[i];
  data.f[i] = inertia_a + cross(data.v[i], inertia_v);
}
```

Things worth noticing:

- `data.pose_in_parent[i]` was filled by `forward_kinematics`. We invert it
  once per joint to transport the parent's twist into body $i$'s frame
  (the formula in chapter 03).
- The `cross(Motion, Motion)` returns a `Motion`; the `cross(Motion,
  Force)` returns a `Force`. The compiler keeps wrenches and twists from
  ever getting mixed up.
- `model.inertia[i] * data.v[i]` returns a `Force` — `I · v` is momentum,
  which is a wrench-like quantity. Again the type tracks the meaning.

## The inward pass

```cpp
for (int i = model.njoints() - 1; i >= 0; --i) {
  const Joint& j = model.joints[i];
  const int joint_nv = nv(j);
  if (joint_nv > 0) {
    detail::joint_subspace_project(j, data.f[i], tau.segment(model.idx_v[i], joint_nv));
  }
  if (model.parent[i] != -1) {
    data.f[model.parent[i]] += data.pose_in_parent[i] * data.f[i];
  }
}
```

For fixed joints (`joint_nv == 0`) we skip the projection but still push the
force up to the parent — a fixed link is rigid, so any wrench on it goes
straight through.

`data.pose_in_parent[i] * data.f[i]` invokes `operator*(SE3, Force)`, which
applies the **dual** adjoint $X^{*}_{T} = \mathrm{Ad}_{T}^{-\top}$. That's the
correct transport for wrenches (chapter 05); the typed `Force` ensures we
get it.

## A worked call

```cpp
#include "tinyspatial/algo/rnea.hpp"
#include "tinyspatial/urdf/urdf_loader.hpp"

const auto m = tinyspatial::build_model_from_urdf_file("data/robots/simple_arm.urdf");
tinyspatial::Data d(m);
tinyspatial::VectorX q = tinyspatial::VectorX::Zero(m.nq());
tinyspatial::VectorX v = tinyspatial::VectorX::Zero(m.nv());
tinyspatial::VectorX a = tinyspatial::VectorX::Zero(m.nv());
tinyspatial::VectorX tau(m.nv());

tinyspatial::rnea(m, d, q, v, a, tau, tinyspatial::Vector3(0, -9.81, 0));
// tau ≈ (29.43, 9.81) Nm — the static gravity moments.
```

This is the same call made by `test_rnea.cpp:HandComputedGravityCompensation`.

> ## Where this lives in the library
>
> | Concept | File · symbol |
> | ------- | ------------- |
> | Function signature | [`rnea.hpp`](../../include/tinyspatial/algo/rnea.hpp) · `rnea` |
> | Outward pass | `rnea.hpp` · the `for (int i = 0; …)` loop |
> | Inward pass | `rnea.hpp` · the `for (int i = njoints()-1; …)` loop |
> | Joint dispatch helpers | `rnea.hpp` · `detail::joint_subspace_motion`, `detail::joint_subspace_project` |
