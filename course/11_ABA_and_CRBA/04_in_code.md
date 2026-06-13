# ABA in code

`include/tinyspatial/algo/aba.hpp` is the densest of the dynamics
algorithms — three passes, six per-body scratch vectors — but each step
maps directly onto the chapter-03 formulas. This sub-chapter walks through
it.

## The signature

```cpp
inline void aba(const Model& model, Data& data,
                const Eigen::Ref<const VectorX>& q,
                const Eigen::Ref<const VectorX>& v,
                const Eigen::Ref<const VectorX>& tau,
                Eigen::Ref<VectorX> qdd,
                const Vector3& gravity = Vector3(0, 0, -9.81));
```

Inputs: `q` (config), `v` (velocity), `tau` (joint torques). Output:
`qdd` (joint accelerations). The `Data` and `gravity` parameters mirror
RNEA.

## The scratch vectors

```cpp
std::vector<Matrix6> i_a(njoints);   // articulated inertia per body
std::vector<Vector6> p_a(njoints);   // articulated bias per body
std::vector<Motion>  c(njoints);     // velocity-product accel per joint

std::vector<Eigen::Matrix<Scalar, 6, Eigen::Dynamic>> u_mat(njoints);            // IA · S
std::vector<Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic>> d_inv(njoints); // (Sᵀ IA S)⁻¹
std::vector<VectorX> u_vec(njoints);  // τ − Sᵀ pA
```

These are allocated once per call. For a 30-DoF robot, that's ~30
small matrices and a couple of vectors — sub-microsecond overhead. The
hot inner loops do no heap allocation themselves (they reuse this
scratch via `.noalias()`).

`i_a[i]` and `p_a[i]` are **raw** 6×6 / 6-vector storage, not
`SpatialInertia` / `Force`. Articulated inertia is *not* the spatial
inertia of any actual body — it's an effective matrix that need not be
expressible in `(m, c, \bar I)` form. So we use the unstructured Matrix6
representation. (`SpatialInertia::matrix6()` converts a real body's inertia
into this representation as a starting point in pass 1.)

## Pass 1: outward (kinematics + seeding)

```cpp
for (int i = 0; i < njoints; ++i) {
  const Joint& j = model.joints[i];
  const int joint_nv = nv(j);
  const Motion v_j = detail::joint_subspace_motion(j, v.segment(model.idx_v[i], joint_nv));
  const SE3 i_from_parent = data.pose_in_parent[i].inverse();

  if (model.parent[i] == -1) {
    data.v[i] = v_j;
    c[i] = Motion::zero();
  } else {
    const Motion v_parent_in_i = i_from_parent * data.v[model.parent[i]];
    data.v[i] = v_parent_in_i + v_j;
    c[i] = cross(data.v[i], v_j);
  }

  i_a[i] = model.inertia[i].matrix6();
  const Force inertia_v = model.inertia[i] * data.v[i];
  p_a[i] = cross(data.v[i], inertia_v).vector();
}
```

For each body:

- `data.v[i]` is set the same way as in RNEA's outward pass.
- `c[i]` is the velocity-product acceleration $v_i \times (S_i \dot
  q_i)$, saved for pass 3.
- `i_a[i]` is seeded with the body's own spatial inertia (as a 6×6
  matrix). This becomes the *articulated* inertia after pass 2.
- `p_a[i]` is the bias force $v_i \times^* (I_i v_i)$, the spatial
  analogue of "$\omega \times I\omega$" for a single body.

No acceleration is computed yet — that's pass 3.

## Pass 2: inward (articulated reduction + transport)

```cpp
for (int i = njoints - 1; i >= 0; --i) {
  const Joint& j = model.joints[i];
  const int joint_nv = nv(j);
  const auto s_i = detail::joint_motion_subspace(j);

  Matrix6 ia_articulated = i_a[i];
  Vector6 pa_articulated = p_a[i];

  if (joint_nv > 0) {
    u_mat[i] = i_a[i] * s_i;
    const auto d = s_i.transpose() * u_mat[i];
    d_inv[i] = d.inverse();
    u_vec[i] = tau.segment(model.idx_v[i], joint_nv) - s_i.transpose() * p_a[i];
    ia_articulated.noalias() -= u_mat[i] * d_inv[i] * u_mat[i].transpose();
    pa_articulated.noalias() += ia_articulated * c[i].vector() + u_mat[i] * d_inv[i] * u_vec[i];
  } else {
    pa_articulated.noalias() += ia_articulated * c[i].vector();
  }

  if (model.parent[i] != -1) {
    const Matrix6 x_force = force_plucker(data.pose_in_parent[i]);
    i_a[model.parent[i]].noalias() += x_force * ia_articulated * x_force.transpose();
    p_a[model.parent[i]].noalias() += x_force * pa_articulated;
  }
}
```

The per-joint rank-$n_{v,i}$ reduction is:

$$
I^A_a = I^A_i - U_i D_i^{-1} U_i^\top, \qquad
p^A_a = p^A_i + I^A_a c_i + U_i D_i^{-1} u_i.
$$

For revolute / prismatic joints, $D_i$ is a $1 \times 1$ scalar, so
`d_inv[i]` is just a reciprocal. For a floating joint, $D_i$ is $6
\times 6$ and we use Eigen's general `.inverse()`. (For a *real*
floating-base simulator one would want an LLT factor here; the library's
default `.inverse()` is fine for cross-check but not the last word in
performance.)

The fixed-joint branch skips the reduction (rank 0) but still transports
`ia_articulated` and `pa_articulated` into the parent — a welded link
still has inertia.

## Pass 3: outward (acceleration propagation)

```cpp
const Motion neg_gravity_world(Vector3::Zero(), -gravity);

for (int i = 0; i < njoints; ++i) {
  const Joint& j = model.joints[i];
  const int joint_nv = nv(j);
  const auto s_i = detail::joint_motion_subspace(j);
  const SE3 i_from_parent = data.pose_in_parent[i].inverse();

  const Motion a_parent_in_i = (model.parent[i] == -1)
      ? (i_from_parent * neg_gravity_world)
      : (i_from_parent * data.a[model.parent[i]]);

  const Vector6 a_no_joint = (a_parent_in_i + c[i]).vector();

  if (joint_nv > 0) {
    const VectorX qdd_i = d_inv[i] * (u_vec[i] - u_mat[i].transpose() * a_no_joint);
    qdd.segment(model.idx_v[i], joint_nv) = qdd_i;
    data.a[i] = Motion(a_no_joint + s_i * qdd_i);
  } else {
    data.a[i] = Motion(a_no_joint);
  }
}
```

The root gets the gravity-trick acceleration. Then for each joint,
$\ddot q_i$ falls out of the formula from chapter 03; we propagate
`data.a[i]` so children can read it. By the time the loop finishes,
`qdd` is fully populated.

## A worked call

```cpp
#include "tinyspatial/algo/aba.hpp"
#include "tinyspatial/urdf/urdf_loader.hpp"

const auto m = tinyspatial::build_model_from_urdf_file("data/robots/franka_fr3.urdf");
tinyspatial::Data d(m);
auto q   = tinyspatial::VectorX::Zero(m.nq());
auto v   = tinyspatial::VectorX::Zero(m.nv());
auto tau = tinyspatial::VectorX::Zero(m.nv());
tinyspatial::VectorX qdd(m.nv());

tinyspatial::aba(m, d, q, v, tau, qdd);  // default gravity (0, 0, -9.81)
// qdd is the joint acceleration the arm has under its own weight,
// motors off, at the home configuration.
```

This is the "robot in free-fall, motors disabled" simulation — exactly
the kind of thing a physics step would do at the first tick.

> ## Where this lives in the library
>
> | Concept | File · symbol |
> | ------- | ------------- |
> | Function signature | [`aba.hpp`](../../include/tinyspatial/algo/aba.hpp) · `aba` |
> | Pass 1 (outward kinematics + bias) | `aba.hpp` · the first `for (int i = 0; …)` loop |
> | Pass 2 (articulated-inertia reduction) | `aba.hpp` · the `for (int i = njoints-1; …)` loop |
> | Pass 3 (acceleration propagation) | `aba.hpp` · the final `for (int i = 0; …)` loop |
