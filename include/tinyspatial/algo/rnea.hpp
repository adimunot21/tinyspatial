/// \file rnea.hpp
/// \brief Recursive Newton–Euler inverse dynamics.
///
/// Given a configuration `q`, velocity `v`, and desired acceleration `a`,
/// returns the joint-space generalised forces `τ` such that
///
///   τ = M(q) a + C(q, v) v + g(q)
///
/// using Featherstone's algorithm 5.4 (body-fixed, angular-first).
///
/// Two passes over the topologically-ordered tree:
///
///   1. **Outward (base → leaves)** propagates body-frame spatial velocity
///      and acceleration. Gravity enters via the standard "trick" — treat
///      the base as having a fictitious upward acceleration of −g — so no
///      gravitational force needs to be added explicitly.
///   2. **Inward (leaves → base)** accumulates each body's net wrench and
///      projects onto each joint's motion subspace.
///
/// Forward kinematics is run internally; the caller does not need to pre-call
/// `forward_kinematics`.
#ifndef TINYSPATIAL_ALGO_RNEA_HPP
#define TINYSPATIAL_ALGO_RNEA_HPP

#include <type_traits>
#include <variant>

#include "tinyspatial/algo/forward_kinematics.hpp"
#include "tinyspatial/core/types.hpp"
#include "tinyspatial/model/joint.hpp"
#include "tinyspatial/model/model.hpp"
#include "tinyspatial/spatial/cross.hpp"
#include "tinyspatial/spatial/force.hpp"
#include "tinyspatial/spatial/inertia.hpp"
#include "tinyspatial/spatial/motion.hpp"

namespace tinyspatial {

namespace detail {

/// Build the joint's spatial motion contribution `S · q_slice` (Featherstone's
/// "v_J"). Returns a Motion in the joint's body frame.
[[nodiscard]] inline Motion joint_subspace_motion(const Joint& j,
                                                  const Eigen::Ref<const VectorX>& v_slice) {
  return std::visit(
      [&](const auto& jj) -> Motion {
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
      },
      j);
}

/// Project a wrench onto a joint's motion subspace: returns the per-joint
/// generalised-force slice (`Sᵀ · F`). Output length equals `nv(j)`.
inline void joint_subspace_project(const Joint& j, const Force& f, Eigen::Ref<VectorX> tau_slice) {
  std::visit(
      [&](const auto& jj) {
        using JT = std::decay_t<decltype(jj)>;
        if constexpr (std::is_same_v<JT, JointRevolute>) {
          tau_slice(0) = jj.axis.dot(f.angular());
        } else if constexpr (std::is_same_v<JT, JointPrismatic>) {
          tau_slice(0) = jj.axis.dot(f.linear());
        } else if constexpr (std::is_same_v<JT, JointFloating>) {
          tau_slice.head<3>() = f.angular();
          tau_slice.tail<3>() = f.linear();
        } else {  // JointFixed
          (void)jj;
          (void)f;
          (void)tau_slice;
        }
      },
      j);
}

}  // namespace detail

/// Inverse dynamics via RNEA. Writes joint torques into `tau`.
///
/// \pre `q.size() == model.nq()`, `v.size() == a.size() == tau.size() == model.nv()`.
/// `data` is sized to `model.njoints()`.
/// `gravity` is in the world frame; default `(0, 0, −9.81)`.
inline void rnea(const Model& model, Data& data, const Eigen::Ref<const VectorX>& q,
                 const Eigen::Ref<const VectorX>& v, const Eigen::Ref<const VectorX>& a,
                 Eigen::Ref<VectorX> tau, const Vector3& gravity = Vector3(0, 0, -9.81)) {
  forward_kinematics(model, data, q);

  // Gravity trick: base "accelerates" at −g in the world frame.
  const Motion neg_gravity_world(Vector3::Zero(), -gravity);

  // Outward pass: fill data.v[i], data.a[i], and data.f[i] (= net wrench on body i).
  for (int i = 0; i < model.njoints(); ++i) {
    const Joint& j = model.joints[i];
    const int joint_nv = nv(j);
    const Motion v_j = detail::joint_subspace_motion(j, v.segment(model.idx_v[i], joint_nv));
    const Motion a_j = detail::joint_subspace_motion(j, a.segment(model.idx_v[i], joint_nv));
    const SE3 i_from_parent = data.pose_in_parent[i].inverse();

    if (model.parent[i] == -1) {
      // Root: parent is the world. Velocity zero; acceleration is the
      // gravity trick expressed in body i.
      data.v[i] = v_j;
      data.a[i] = (i_from_parent * neg_gravity_world) + a_j + cross(data.v[i], v_j);
    } else {
      const Motion v_parent_in_i = i_from_parent * data.v[model.parent[i]];
      const Motion a_parent_in_i = i_from_parent * data.a[model.parent[i]];
      data.v[i] = v_parent_in_i + v_j;
      data.a[i] = a_parent_in_i + a_j + cross(data.v[i], v_j);
    }

    // Net body wrench: I·a + v ×* (I·v).
    const Force inertia_a = model.inertia[i] * data.a[i];
    const Force inertia_v = model.inertia[i] * data.v[i];
    data.f[i] = inertia_a + cross(data.v[i], inertia_v);
  }

  // Inward pass: project to joint torques, accumulate force at parent.
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
}

}  // namespace tinyspatial

#endif  // TINYSPATIAL_ALGO_RNEA_HPP
