/// \file dls.hpp
/// \brief Damped least-squares (DLS) inverse kinematics.
///
/// Iteratively solves for a configuration `q*` that places a chosen link's
/// body frame at a target SE(3) pose. Each iteration:
///
///   1. Forward kinematics → current pose `oM_link(q)`.
///   2. Body-frame error: `e = log(oM_link(q)^{-1} · target)` ∈ ℝ⁶.
///   3. Local Jacobian `J` (6×nv) at the link.
///   4. Damped step: `Δq = J^⊤ (J J^⊤ + λ² I₆)^{-1} · e`.
///   5. `q ← q + α · Δq`.
///
/// Step 4 is the *damped* pseudoinverse — Nakamura & Hanafusa's
/// formulation. The `λ²` term keeps `(J J^⊤)` invertible even at kinematic
/// singularities, at the cost of biased steps away from those singularities.
/// Tunes: smaller `λ` is more accurate but less robust; larger `λ` is the
/// reverse.
///
/// Convergence: not guaranteed (IK is non-convex). The success rate on a
/// 7-DoF arm is typically 80–99% depending on seed quality and target
/// reachability; we return a status flag so the caller can retry from a
/// different seed if needed.
#ifndef TINYSPATIAL_IK_DLS_HPP
#define TINYSPATIAL_IK_DLS_HPP

#include "tinyspatial/algo/forward_kinematics.hpp"
#include "tinyspatial/algo/jacobian.hpp"
#include "tinyspatial/core/types.hpp"
#include "tinyspatial/liegroup/se3.hpp"
#include "tinyspatial/model/model.hpp"

namespace tinyspatial {

/// Tuning parameters for DLS IK. Defaults are sensible for revolute 6/7-DoF
/// arms working in metres / radians.
struct DlsOptions {
  /// Maximum iterations before giving up.
  int max_iters = 200;
  /// Convergence tolerance on the body-frame error (Lie-tangent ∞-norm).
  Scalar tolerance = 1e-6;
  /// Damping coefficient `λ`; the squared form `λ²` is what enters the linear
  /// solve. Small positive values bias the step toward `J^⊤ e` near
  /// singularities.
  Scalar damping = 1e-2;
  /// Multiplicative step size. `1.0` is the textbook value; reduce if the
  /// solver oscillates.
  Scalar step_size = 1.0;
};

/// Output of an IK solve.
struct IkResult {
  /// The (possibly final, non-converged) configuration.
  VectorX q;
  /// Final body-frame error vector (Lie tangent).
  Vector6 error;
  /// Number of iterations actually run.
  int iterations = 0;
  /// `true` iff `||error||_∞ <= options.tolerance`.
  bool converged = false;
};

/// Damped least-squares IK for placing `link_id`'s body frame at `target_in_world`.
///
/// \pre `q_init.size() == model.nq()`. `link_id` is a valid joint index
/// in `model`. `data` is sized to `model.njoints()`.
[[nodiscard]] inline IkResult solve_ik_dls(const Model& model, Data& data, int link_id,
                                           const SE3& target_in_world,
                                           const Eigen::Ref<const VectorX>& q_init,
                                           const DlsOptions& options = {}) {
  IkResult result;
  result.q = q_init;

  Matrix6X j(6, model.nv());
  const Matrix6 lambda_sq_i6 = options.damping * options.damping * Matrix6::Identity();

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

    // Δq = J^T (J J^T + λ² I)^{-1} · e   (size: nv × 1).
    const Matrix6 jjt_damped = j * j.transpose() + lambda_sq_i6;
    const Vector6 alpha = jjt_damped.ldlt().solve(result.error);
    const VectorX dq = j.transpose() * alpha;

    result.q.noalias() += options.step_size * dq;
  }

  result.iterations = options.max_iters;
  result.converged = false;
  return result;
}

}  // namespace tinyspatial

#endif  // TINYSPATIAL_IK_DLS_HPP
