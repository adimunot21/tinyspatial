/// \file jacobian.hpp
/// \brief Geometric Jacobian of a link's spatial velocity w.r.t. joint velocities.
///
/// For a link L, the spatial velocity is `v_L = J(q) · qdot`, where the columns
/// of J correspond to entries of `qdot` (i.e. velocity coordinates as listed in
/// `Model::idx_v`). J is 6×nv with rows in our angular-first ordering.
///
/// Convention mismatch is the #1 cause of Pinocchio parity bugs (CLAUDE.md
/// §15), so the **reference frame** of J is an explicit input. Three options
/// mirror Pinocchio's:
///
///   - `kLocal`              — J expressed in L's own body frame.
///   - `kWorld`              — J expressed in the world frame.
///   - `kLocalWorldAligned`  — translation at L's origin, but rotation aligned
///                             with the world (frame "WORLD_ALIGNED" in
///                             Pinocchio terminology; useful for IK).
#ifndef TINYSPATIAL_ALGO_JACOBIAN_HPP
#define TINYSPATIAL_ALGO_JACOBIAN_HPP

#include <type_traits>
#include <variant>

#include "tinyspatial/core/types.hpp"
#include "tinyspatial/model/joint.hpp"
#include "tinyspatial/model/model.hpp"

namespace tinyspatial {

/// Reference frame for the geometric Jacobian output.
enum class JacobianFrame {
  kLocal,
  kWorld,
  kLocalWorldAligned,
};

/// Compute the 6×nv geometric Jacobian of joint `link_id`'s body frame.
/// \pre Forward kinematics has been run for `(model, data, q)` already.
/// \pre `j_out` is sized 6×`model.nv()`.
inline void compute_jacobian(const Model& model, const Data& data, int link_id,
                             Eigen::Ref<Matrix6X> j_out,
                             JacobianFrame frame = JacobianFrame::kLocal) {
  j_out.setZero();
  const SE3 pose_world_link_inv = data.pose_in_world[link_id].inverse();

  // Walk from `link_id` toward the root through parent[]. Only joints on this
  // path can contribute to L's velocity.
  for (int i = link_id; i != -1; i = model.parent[i]) {
    const SE3 t_link_i = pose_world_link_inv * data.pose_in_world[i];
    const Matrix6 ad_link_from_i = t_link_i.adjoint();
    const int col_start = model.idx_v[i];
    std::visit(
        [&](const auto& joint) {
          using JT = std::decay_t<decltype(joint)>;
          if constexpr (std::is_same_v<JT, JointRevolute>) {
            Vector6 s_in_i;
            s_in_i.head<3>() = joint.axis;
            s_in_i.tail<3>().setZero();
            j_out.col(col_start) = ad_link_from_i * s_in_i;
          } else if constexpr (std::is_same_v<JT, JointPrismatic>) {
            Vector6 s_in_i;
            s_in_i.head<3>().setZero();
            s_in_i.tail<3>() = joint.axis;
            j_out.col(col_start) = ad_link_from_i * s_in_i;
          } else if constexpr (std::is_same_v<JT, JointFloating>) {
            // Six unit twists in joint i's frame, each Plücker-transported.
            j_out.template block<6, 6>(0, col_start) = ad_link_from_i;
          } else {
            // JointFixed contributes nothing.
            (void)joint;
          }
        },
        model.joints[i]);
  }

  // Apply the final frame transform.
  switch (frame) {
    case JacobianFrame::kLocal:
      break;
    case JacobianFrame::kWorld:
      j_out = data.pose_in_world[link_id].adjoint() * j_out;
      break;
    case JacobianFrame::kLocalWorldAligned: {
      // T_lwa_local = (R_link_world, 0); adjoint is block-diagonal in R.
      const Matrix3 r = data.pose_in_world[link_id].rotation().matrix();
      Matrix6 ad_lwa = Matrix6::Zero();
      ad_lwa.topLeftCorner<3, 3>() = r;
      ad_lwa.bottomRightCorner<3, 3>() = r;
      j_out = ad_lwa * j_out;
      break;
    }
  }
}

}  // namespace tinyspatial

#endif  // TINYSPATIAL_ALGO_JACOBIAN_HPP
