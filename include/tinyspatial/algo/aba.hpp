/// \file aba.hpp
/// \brief Featherstone's Articulated-Body Algorithm — forward dynamics.
///
/// Given a configuration `q`, velocity `v`, and joint torque `τ`, solves for
/// the joint acceleration `q̈` satisfying
///
///   τ = M(q) · q̈ + C(q, v) v + g(q)
///
/// in O(N) without ever forming `M`. Three passes:
///
///   1. **Outward (base → leaves).** Same outward sweep as RNEA computes
///      `v[i]`. We also record the joint-velocity-product acceleration
///      `c[i] = v[i] × (S_i · v_slice)` (zero for the root) and seed each
///      body's *articulated inertia* `IA[i] = I[i]` and *articulated bias
///      force* `pA[i] = v[i] ×* (I[i] · v[i])` from its rigid-body inertia.
///
///   2. **Inward (leaves → base).** Each body's articulated inertia /
///      bias absorb the contribution of every joint below it. For joint
///      `i` with motion subspace `S_i`:
///          U_i = IA[i] · S_i,     D_i = S_iᵀ · U_i,     u_i = τ_i − S_iᵀ · pA[i]
///      and the articulated inertia / bias seen *through* the joint are
///          IA_a = IA[i] − U_i · D_i⁻¹ · U_iᵀ
///          pA_a = pA[i] + IA_a · c[i] + U_i · D_i⁻¹ · u_i
///      transported into the parent's frame via the force Plücker transform
///      (`IA` by congruence with `X*`; `pA` by `X*`).
///
///   3. **Outward (base → leaves).** With every IA/pA known, propagate
///      acceleration: gravity enters at the root via the same trick as RNEA,
///      and at each joint
///          q̈_i = D_i⁻¹ · (u_i − U_iᵀ · (a_parent + c[i]))
///          a[i]  = a_parent + c[i] + S_i · q̈_i
///
/// Featherstone (2008) §7.3, body-fixed angular-first. Forward kinematics is
/// run internally; the caller does not need to pre-call `forward_kinematics`.
#ifndef TINYSPATIAL_ALGO_ABA_HPP
#define TINYSPATIAL_ALGO_ABA_HPP

#include <vector>

#include "tinyspatial/algo/forward_kinematics.hpp"
#include "tinyspatial/algo/rnea.hpp"  // joint_subspace_motion
#include "tinyspatial/core/types.hpp"
#include "tinyspatial/model/joint.hpp"
#include "tinyspatial/model/model.hpp"
#include "tinyspatial/spatial/cross.hpp"
#include "tinyspatial/spatial/force.hpp"
#include "tinyspatial/spatial/inertia.hpp"
#include "tinyspatial/spatial/motion.hpp"
#include "tinyspatial/spatial/plucker.hpp"

namespace tinyspatial {

/// Forward dynamics via ABA. Writes joint accelerations into `qdd`.
///
/// \pre `q.size() == model.nq()`, `v.size() == tau.size() == qdd.size() == model.nv()`.
/// `data` is sized to `model.njoints()`. `gravity` is in the world frame.
inline void aba(const Model& model, Data& data, const Eigen::Ref<const VectorX>& q,
                const Eigen::Ref<const VectorX>& v, const Eigen::Ref<const VectorX>& tau,
                Eigen::Ref<VectorX> qdd, const Vector3& gravity = Vector3(0, 0, -9.81)) {
  forward_kinematics(model, data, q);

  const int njoints = model.njoints();

  // Per-body scratch. IA / pA are mutated in passes 1 and 2; U / D_inv / u
  // are computed in pass 2 and reused in pass 3.
  std::vector<Matrix6> i_a(njoints);
  std::vector<Vector6> p_a(njoints);
  std::vector<Motion> c(njoints);
  std::vector<Eigen::Matrix<Scalar, 6, Eigen::Dynamic>> u_mat(njoints);
  std::vector<Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic>> d_inv(njoints);
  std::vector<VectorX> u_vec(njoints);

  // ---- Pass 1: outward. Fill v[i], c[i], IA[i], pA[i].
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

  // ---- Pass 2: inward. Accumulate articulated inertia / bias at each parent.
  for (int i = njoints - 1; i >= 0; --i) {
    const Joint& j = model.joints[i];
    const int joint_nv = nv(j);
    const Matrix6X& s_i = model.motion_subspace[i];

    Matrix6 ia_articulated = i_a[i];
    Vector6 pa_articulated = p_a[i];

    if (joint_nv > 0) {
      u_mat[i] = i_a[i] * s_i;
      const Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> d = s_i.transpose() * u_mat[i];
      d_inv[i] = d.inverse();
      u_vec[i] = tau.segment(model.idx_v[i], joint_nv) - s_i.transpose() * p_a[i];
      // Reduce articulated quantities through the joint.
      ia_articulated.noalias() -= u_mat[i] * d_inv[i] * u_mat[i].transpose();
      pa_articulated.noalias() += ia_articulated * c[i].vector() + u_mat[i] * d_inv[i] * u_vec[i];
    } else {
      // Fixed joint: no rank reduction, but still carries the c[i] term.
      pa_articulated.noalias() += ia_articulated * c[i].vector();
    }

    if (model.parent[i] != -1) {
      // Both transports share the 6×6 X* matrix; building it once is cheaper
      // than two separate inline expansions.
      const Matrix6 x_force = force_plucker(data.pose_in_parent[i]);
      // Symmetric congruence: IA_parent += X* · IA_a · X*ᵀ.
      i_a[model.parent[i]].noalias() += x_force * ia_articulated * x_force.transpose();
      p_a[model.parent[i]].noalias() += x_force * pa_articulated;
    }
  }

  // ---- Pass 3: outward. Solve for q̈ at each joint and propagate `a[i]`.
  const Motion neg_gravity_world(Vector3::Zero(), -gravity);
  for (int i = 0; i < njoints; ++i) {
    const Joint& j = model.joints[i];
    const int joint_nv = nv(j);
    const Matrix6X& s_i = model.motion_subspace[i];
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
}

}  // namespace tinyspatial

#endif  // TINYSPATIAL_ALGO_ABA_HPP
