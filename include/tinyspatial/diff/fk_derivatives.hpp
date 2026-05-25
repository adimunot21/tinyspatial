/// \file fk_derivatives.hpp
/// \brief Per-joint Jacobians — the analytical derivative of every joint's
/// world pose with respect to the configuration `q`.
///
/// For each joint `i` we produce a 6×nv matrix `J_i` whose column `k` is the
/// body-frame spatial twist of joint `i` induced by unit velocity at
/// velocity-coordinate `k`. Equivalently, in the Lie sense,
///
///   `∂(oMi)/∂q_k`  =  oMi · `[J_i col k]^∧`
///
/// where `[ξ]^∧` is the se(3) hat. Each `J_i` is sparse — only the columns
/// corresponding to ancestor joints are non-zero — but we store the full
/// 6×nv matrix anyway, matching Pinocchio's layout so the validation harness
/// and downstream RNEA-derivative code can read it directly.
///
/// The single-joint `compute_jacobian` in `algo/jacobian.hpp` is the right
/// API when you only want one link's Jacobian (the 80% case: IK, control).
/// Use this header when you need the full set, *e.g.* for analytical
/// derivatives of dynamics quantities.
///
/// Recurrence: for joint `i` with parent `p = parent[i]`,
///
///   J_i = Ad_{T_{ip}} · J_p,    then  add joint i's own screw axis at
///                                     column idx_v[i].
///
/// Topological order makes this a single forward sweep, O(n · nv).
#ifndef TINYSPATIAL_DIFF_FK_DERIVATIVES_HPP
#define TINYSPATIAL_DIFF_FK_DERIVATIVES_HPP

#include <type_traits>
#include <variant>
#include <vector>

#include "tinyspatial/algo/forward_kinematics.hpp"
#include "tinyspatial/core/types.hpp"
#include "tinyspatial/model/joint.hpp"
#include "tinyspatial/model/model.hpp"

namespace tinyspatial {

/// Body-frame Jacobian of every joint, in one sweep.
///
/// Fills `joint_jacobians[i]` (size 6 × `model.nv()`) with `J_i` as described
/// above. The vector is resized if needed; existing matrices are reused.
///
/// \pre `q.size() == model.nq()`. `data` is sized to `model.njoints()`.
/// `forward_kinematics(model, data, q)` is called internally; the caller does
/// not need to pre-run it.
inline void compute_joint_jacobians(const Model& model, Data& data,
                                    const Eigen::Ref<const VectorX>& q,
                                    std::vector<Matrix6X>& joint_jacobians) {
  forward_kinematics(model, data, q);

  const int njoints = model.njoints();
  const int nv_total = model.nv();
  joint_jacobians.assign(njoints, Matrix6X::Zero(6, nv_total));

  for (int i = 0; i < njoints; ++i) {
    Matrix6X& j_i = joint_jacobians[i];
    const int p = model.parent[i];
    if (p >= 0) {
      // Transport parent's Jacobian into body i's frame.
      const Matrix6 ad_i_from_p = data.pose_in_parent[i].inverse().adjoint();
      j_i.noalias() = ad_i_from_p * joint_jacobians[p];
    }
    // Add joint i's own screw axis at columns idx_v[i] .. idx_v[i] + nv_i - 1.
    const int col_start = model.idx_v[i];
    std::visit(
        [&](const auto& joint) {
          using JT = std::decay_t<decltype(joint)>;
          if constexpr (std::is_same_v<JT, JointRevolute>) {
            j_i.template block<3, 1>(0, col_start) = joint.axis;
          } else if constexpr (std::is_same_v<JT, JointPrismatic>) {
            j_i.template block<3, 1>(3, col_start) = joint.axis;
          } else if constexpr (std::is_same_v<JT, JointFloating>) {
            j_i.template block<6, 6>(0, col_start) = Matrix6::Identity();
          } else {
            (void)joint;  // JointFixed contributes nothing.
          }
        },
        model.joints[i]);
  }
}

}  // namespace tinyspatial

#endif  // TINYSPATIAL_DIFF_FK_DERIVATIVES_HPP
