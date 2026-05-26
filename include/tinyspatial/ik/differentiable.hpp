/// \file differentiable.hpp
/// \brief The Jacobian `∂q*/∂target` of the IK map at a converged solution.
///
/// "Differentiable IK" makes the IK solver itself a differentiable building
/// block. Given a converged solution `q*` for a target pose `T*`, this
/// header returns the matrix `∂q*/∂T*` (size `nv × 6`) so a higher-level
/// pipeline can back-propagate gradients through IK without finite
/// differencing.
///
/// Derivation: at convergence the residual `r(q, T) = log(T(q)⁻¹ · T) = 0`.
/// Differentiating both sides w.r.t. `T` (parametrized as `T → T · exp(δξ)`
/// with `δξ ∈ ℝ⁶` in body-local Lie-tangent ordering):
///
///   ∂r/∂q · (∂q*/∂T)  +  ∂r/∂T  =  0,
///   ∂r/∂q = -J_local(q*),   ∂r/∂T = +I_6,
///
/// so
///
///   ∂q*/∂T = J_local(q*)⁺ = J⊤ (J J⊤ + λ² I)⁻¹      (damped, matching DLS).
///
/// The damped form is what the DLS solver effectively uses as its
/// "tangent." Reusing the same damping here keeps the analytical
/// derivative consistent with the solver near singularities.
///
/// Use cases:
///   - End-to-end learning: gradient of a loss `L(q*)` w.r.t. the target.
///   - Sensitivity analysis: "if my target moves by 1 mm in z, how do the
///     joints change?"
///   - SQP / outer-loop optimisers that need IK Jacobians.
///
/// **The parametrization:** target perturbations are body-local
/// (right-multiplicative). To get world-frame derivatives, post-multiply
/// the returned matrix by `Ad_{T*}⁻¹` (i.e., apply
/// `T*.adjoint_inverse()` to each column). See course chapter 13.
#ifndef TINYSPATIAL_IK_DIFFERENTIABLE_HPP
#define TINYSPATIAL_IK_DIFFERENTIABLE_HPP

#include "tinyspatial/algo/forward_kinematics.hpp"
#include "tinyspatial/algo/jacobian.hpp"
#include "tinyspatial/core/types.hpp"
#include "tinyspatial/model/model.hpp"

namespace tinyspatial {

/// Compute `∂q*/∂target` at a converged IK solution `q_star`.
///
/// Target perturbations are body-local Lie-tangent: a column `k` of the
/// output is "how `q*` changes per unit `δξ_k` where the target is
/// perturbed as `T* → T* · exp(δξ^∧)` with `δξ` in (ω; v) ordering."
///
/// \pre `q_star.size() == model.nq()`. `link_id` is a valid joint index.
/// `data` is sized to `model.njoints()`. `damping` must match the value
/// used by the solver that produced `q_star` for the derivative to be
/// consistent at the per-step level.
[[nodiscard]] inline MatrixX ik_implicit_derivative(const Model& model, Data& data, int link_id,
                                                    const Eigen::Ref<const VectorX>& q_star,
                                                    Scalar damping = 1e-2) {
  forward_kinematics(model, data, q_star);

  Matrix6X j(6, model.nv());
  compute_jacobian(model, data, link_id, j, JacobianFrame::kLocal);

  const Matrix6 jjt_damped = j * j.transpose() + damping * damping * Matrix6::Identity();
  const Matrix6 inv_jjt = jjt_damped.ldlt().solve(Matrix6::Identity());

  MatrixX dq_dtarget(model.nv(), 6);
  dq_dtarget.noalias() = j.transpose() * inv_jjt;
  return dq_dtarget;
}

}  // namespace tinyspatial

#endif  // TINYSPATIAL_IK_DIFFERENTIABLE_HPP
