/// \file se3.hpp
/// \brief SE(3): rigid transforms (rotation + translation), stored as SO3 + R³.
///
/// Conventions (see the course chapter 04):
///   - A transform is `(rotation, translation)`; `act(p) = R·p + t`.
///   - The twist tangent is ANGULAR-FIRST: `ξ = (ω; v) ∈ ℝ⁶`, ω the angular
///     part (indices 0..2), v the linear part (3..5). This matches Featherstone
///     and diverges from Pinocchio's linear-first ordering (CLAUDE.md §5, §11).
///   - `exp(ξ) = (SO3::exp(ω),  Jl_SO3(ω)·v)`, where Jl_SO3 is the SO(3) left
///     Jacobian. `log` inverts it.
///   - The 6×6 adjoint (angular-first) is `Ad = [[R, 0], [ [t]_× R, R ]]`.
///   - The 6×6 group Jacobians use Barfoot's Q matrix (State Estimation for
///     Robotics, eq. 7.86), reordered for our angular-first convention.
///
/// Templated on the scalar (`SE3T<S>`), with `SE3 = SE3T<double>`. See so3.hpp
/// for why the templating exists and how the `Jet` (differentiable) path works;
/// SE(3) adds no new transcendental beyond what SO(3) already handles.
#ifndef TINYSPATIAL_LIEGROUP_SE3_HPP
#define TINYSPATIAL_LIEGROUP_SE3_HPP

#include <cmath>

#include "tinyspatial/core/types.hpp"
#include "tinyspatial/liegroup/so3.hpp"

namespace tinyspatial {

/// A rigid transform in 3-D, templated on the scalar type.
template <typename S>
class SE3T {
 public:
  using Scalar = S;
  using Vector3 = typename Types<S>::Vector3;
  using Vector6 = typename Types<S>::Vector6;
  using Matrix3 = typename Types<S>::Matrix3;
  using Matrix4 = typename Types<S>::Matrix4;
  using Matrix6 = typename Types<S>::Matrix6;
  /// In-class alias so the bodies read against `SO3` as before; this resolves to
  /// `SO3T<S>` (the matching-scalar rotation), not the global `SO3T<double>`.
  using SO3 = SO3T<S>;

  /// Identity transform.
  SE3T() : rotation_(SO3::identity()), translation_(Vector3::Zero()) {}

  /// Construct from a rotation and a translation.
  SE3T(const SO3& rotation, const Eigen::Ref<const Vector3>& translation)
      : rotation_(rotation), translation_(translation) {}

  /// The identity transform.
  [[nodiscard]] static SE3T identity() { return SE3T(); }

  [[nodiscard]] const SO3& rotation() const { return rotation_; }
  [[nodiscard]] const Vector3& translation() const { return translation_; }

  /// The equivalent 4×4 homogeneous matrix.
  [[nodiscard]] Matrix4 matrix() const {
    Matrix4 m = Matrix4::Identity();
    m.template topLeftCorner<3, 3>() = rotation_.matrix();
    m.template topRightCorner<3, 1>() = translation_;
    return m;
  }

  /// Group composition: `(*this) ∘ rhs`.
  [[nodiscard]] SE3T operator*(const SE3T& rhs) const {
    return SE3T(rotation_ * rhs.rotation_, rotation_.act(rhs.translation_) + translation_);
  }

  /// Group inverse.
  [[nodiscard]] SE3T inverse() const {
    const SO3 r_inv = rotation_.inverse();
    return SE3T(r_inv, -r_inv.act(translation_));
  }

  /// Transform a point: returns `R·p + t`.
  [[nodiscard]] Vector3 act(const Eigen::Ref<const Vector3>& p) const {
    return rotation_.act(p) + translation_;
  }

  /// The 6×6 adjoint, angular-first: maps a twist expressed in this frame to
  /// the parent frame.
  [[nodiscard]] Matrix6 adjoint() const {
    const Matrix3 r = rotation_.matrix();
    Matrix6 ad = Matrix6::Zero();
    ad.template topLeftCorner<3, 3>() = r;
    ad.template bottomRightCorner<3, 3>() = r;
    ad.template bottomLeftCorner<3, 3>() = skew(translation_) * r;
    return ad;
  }

  /// Inverse of the adjoint: `Ad(T)⁻¹ = Ad(T⁻¹)`.
  [[nodiscard]] Matrix6 adjoint_inverse() const { return inverse().adjoint(); }

  /// Exponential map: twist ξ = (ω; v) → transform.
  [[nodiscard]] static SE3T exp(const Eigen::Ref<const Vector6>& xi) {
    const Vector3 omega = xi.template head<3>();
    const Vector3 v = xi.template tail<3>();
    return SE3T(SO3::exp(omega), SO3::left_jacobian(omega) * v);
  }

  /// Logarithm map: transform → twist ξ = (ω; v).
  [[nodiscard]] Vector6 log() const {
    const Vector3 omega = rotation_.log();
    Vector6 xi;
    xi.template head<3>() = omega;
    xi.template tail<3>() = SO3::left_jacobian_inverse(omega) * translation_;
    return xi;
  }

  // SE(3) Jacobians (6×6).  In angular-first ordering:
  //   Jl(ξ)   = [[ Jl_SO3(ω), 0 ], [ Q(v,ω), Jl_SO3(ω) ]]
  //   Jl(ξ)⁻¹ = [[ Jl⁻¹, 0 ], [ -Jl⁻¹ Q Jl⁻¹, Jl⁻¹ ]]
  // Jr(ξ) = Jl(-ξ).

  /// Left Jacobian Jl(ξ) on SE(3) (6×6).
  [[nodiscard]] static Matrix6 left_jacobian(const Eigen::Ref<const Vector6>& xi) {
    const Vector3 omega = xi.template head<3>();
    const Vector3 v = xi.template tail<3>();
    const Matrix3 jl = SO3::left_jacobian(omega);
    Matrix6 j = Matrix6::Zero();
    j.template topLeftCorner<3, 3>() = jl;
    j.template bottomRightCorner<3, 3>() = jl;
    j.template bottomLeftCorner<3, 3>() = left_jacobian_q(omega, v);
    return j;
  }

  /// Right Jacobian Jr(ξ) = Jl(-ξ).
  [[nodiscard]] static Matrix6 right_jacobian(const Eigen::Ref<const Vector6>& xi) {
    return left_jacobian(Vector6(-xi));
  }

  /// Inverse of the left Jacobian.
  [[nodiscard]] static Matrix6 left_jacobian_inverse(const Eigen::Ref<const Vector6>& xi) {
    const Vector3 omega = xi.template head<3>();
    const Vector3 v = xi.template tail<3>();
    const Matrix3 jl_inv = SO3::left_jacobian_inverse(omega);
    Matrix6 j = Matrix6::Zero();
    j.template topLeftCorner<3, 3>() = jl_inv;
    j.template bottomRightCorner<3, 3>() = jl_inv;
    j.template bottomLeftCorner<3, 3>() = -jl_inv * left_jacobian_q(omega, v) * jl_inv;
    return j;
  }

  /// Inverse of the right Jacobian.
  [[nodiscard]] static Matrix6 right_jacobian_inverse(const Eigen::Ref<const Vector6>& xi) {
    return left_jacobian_inverse(Vector6(-xi));
  }

 private:
  /// Barfoot's Q(v, ω) block (angular-first), the off-diagonal of Jl(ξ).
  [[nodiscard]] static Matrix3 left_jacobian_q(const Eigen::Ref<const Vector3>& omega,
                                               const Eigen::Ref<const Vector3>& v) {
    using std::cos;
    using std::sin;
    using std::sqrt;
    const Matrix3 w = skew(omega);  // φ^
    const Matrix3 p = skew(v);      // ρ^
    const Scalar theta2 = omega.squaredNorm();

    // Coefficients (with Taylor fallbacks below kSmallAngle, where the closed
    // forms cancel). Leading values: c1→1/6, c2→1/24, c3→1/120.
    Scalar c1(0);  // (θ − sinθ)/θ³
    Scalar c2(0);  // (θ² + 2cosθ − 2)/(2θ⁴)
    Scalar c3(0);  // (2θ − 3sinθ + θcosθ)/(2θ⁵)
    if (theta2 > kSmallAngle * kSmallAngle) {
      const Scalar theta = sqrt(theta2);
      const Scalar s = sin(theta);
      const Scalar c = cos(theta);
      const Scalar t3 = theta2 * theta;
      c1 = (theta - s) / t3;
      c2 = (theta2 + 2 * c - 2) / (2 * theta2 * theta2);
      c3 = (2 * theta - 3 * s + theta * c) / (2 * t3 * theta2);
    } else {
      c1 = Scalar(1) / Scalar(6) - theta2 / Scalar(120);
      c2 = Scalar(1) / Scalar(24) - theta2 / Scalar(720);
      c3 = Scalar(1) / Scalar(120);
    }

    const Matrix3 wp = w * p;
    const Matrix3 pw = p * w;
    return Scalar(0.5) * p + c1 * (wp + pw + w * p * w) + c2 * (w * wp + pw * w - 3 * w * p * w) +
           c3 * (w * (w * p * w) + (w * wp) * w);
  }

  SO3 rotation_;
  Vector3 translation_;
};

/// The double-precision transform the public API and algorithms use.
using SE3 = SE3T<double>;

}  // namespace tinyspatial

#endif  // TINYSPATIAL_LIEGROUP_SE3_HPP
