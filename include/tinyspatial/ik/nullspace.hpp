/// \file nullspace.hpp
/// \brief Task-priority IK: primary end-effector pose plus a secondary
/// objective projected through the primary's null-space.
///
/// For a 7-DoF arm (or any redundant robot), the Jacobian has a nontrivial
/// null-space — motions that don't move the end-effector. We exploit it to
/// satisfy a *secondary* objective without compromising the primary one:
///
///   Δq = J⁺ · e_primary + α · N · ∇(secondary)
///   N  = I - J⁺ · J         (projector onto null(J))
///
/// where `J⁺` is the DLS damped pseudoinverse `J^⊤ (J J^⊤ + λ² I)^{-1}`.
///
/// This implementation uses the *canonical* secondary task: "stay near a
/// rest posture `q_rest`," with gradient `(q_rest - q)`. That's the
/// classic "elbow-up" or "natural pose" objective. A future generalization
/// could accept an arbitrary `(q -> ∇secondary)` callable.
///
/// For non-redundant robots (6-DoF arms at full rank), `N` is the zero
/// matrix and the secondary objective contributes nothing. The solver
/// still converges on the primary task — it just doesn't do anything extra.
#ifndef TINYSPATIAL_IK_NULLSPACE_HPP
#define TINYSPATIAL_IK_NULLSPACE_HPP

#include "tinyspatial/algo/forward_kinematics.hpp"
#include "tinyspatial/algo/jacobian.hpp"
#include "tinyspatial/core/types.hpp"
#include "tinyspatial/ik/dls.hpp"
#include "tinyspatial/liegroup/se3.hpp"
#include "tinyspatial/model/model.hpp"

namespace tinyspatial {

/// Tuning parameters for nullspace IK. Inherits the DLS knobs (primary
/// task) plus a secondary-task gain.
struct NullspaceOptions {
  /// Maximum iterations before giving up.
  int max_iters = 200;
  /// Convergence tolerance on the body-frame primary error (Lie ∞-norm).
  Scalar tolerance = 1e-6;
  /// DLS damping for the primary task.
  Scalar damping = 1e-2;
  /// Step size on the combined `Δq`.
  Scalar step_size = 1.0;
  /// Gain on the secondary (posture) gradient. Larger means stronger pull
  /// toward `q_rest`; too large can destabilise the primary convergence.
  Scalar secondary_gain = 0.5;
};

/// Task-priority IK targeting `target_in_world` on `link_id`'s body
/// frame, with a posture-attraction secondary task pulling toward `q_rest`.
///
/// \pre `q_init.size() == q_rest.size() == model.nq()`. For non-revolute
/// joint types with manifold-valued configurations (floating bases), the
/// raw `(q_rest - q)` gradient is no longer the right secondary; this
/// implementation is correct for revolute/prismatic-only models (which
/// includes all current fixtures).
[[nodiscard]] inline IkResult solve_ik_nullspace(const Model& model, Data& data, int link_id,
                                                 const SE3& target_in_world,
                                                 const Eigen::Ref<const VectorX>& q_init,
                                                 const Eigen::Ref<const VectorX>& q_rest,
                                                 const NullspaceOptions& options = {}) {
  IkResult result;
  result.q = q_init;

  const int nv_t = model.nv();
  Matrix6X j(6, nv_t);
  const Matrix6 lambda_sq_i6 = options.damping * options.damping * Matrix6::Identity();
  const MatrixX identity_nv = MatrixX::Identity(nv_t, nv_t);

  for (int iter = 0; iter < options.max_iters; ++iter) {
    forward_kinematics(model, data, result.q);
    const SE3 current = data.pose_in_world[link_id];
    const SE3 err_se3 = current.inverse() * target_in_world;
    result.error = err_se3.log();

    if (result.error.cwiseAbs().maxCoeff() <= options.tolerance) {
      result.iterations = iter;
      result.converged = true;
      return result;
    }

    compute_jacobian(model, data, link_id, j, JacobianFrame::kLocal);

    // Primary step uses the *damped* pseudoinverse J⁺ = J^⊤ (J J^⊤ + λ² I)^{-1}
    // — singularity-robust at the cost of a biased step.
    const Matrix6 jjt_damped = j * j.transpose() + lambda_sq_i6;
    const Vector6 alpha = jjt_damped.ldlt().solve(result.error);
    const VectorX dq_primary = j.transpose() * alpha;

    // Projector uses a *much smaller* regularizer so the leak from
    // null(J) into row(J) is ≤ O(λ_proj² / σ_min²) ~ 1e-20 in practice.
    // Without this two-tier damping, residual leakage at λ=1e-2
    // prevents primary convergence below ~1e-4.
    constexpr Scalar kProjectorReg = 1e-10;
    const Matrix6 jjt_projector = j * j.transpose() + kProjectorReg * Matrix6::Identity();
    const auto jjt_proj_ldlt = jjt_projector.ldlt();

    // Secondary task: pull toward q_rest. dq_secondary lives in (≈) null(J).
    const VectorX dq_secondary_raw = q_rest - result.q;
    const Vector6 j_dq_raw = j * dq_secondary_raw;
    const VectorX dq_secondary_proj_into_row = j.transpose() * jjt_proj_ldlt.solve(j_dq_raw);
    const VectorX dq_secondary = dq_secondary_raw - dq_secondary_proj_into_row;

    const VectorX dq = dq_primary + options.secondary_gain * dq_secondary;
    result.q.noalias() += options.step_size * dq;
  }

  result.iterations = options.max_iters;
  result.converged = false;
  return result;
}

}  // namespace tinyspatial

#endif  // TINYSPATIAL_IK_NULLSPACE_HPP
