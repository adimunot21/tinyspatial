/// \file so3.hpp
/// \brief SO(3): the group of 3-D rotations, stored as a unit quaternion.
///
/// Conventions (see CLAUDE.md §15 and the course chapter 04):
///   - Rotations are stored as a Hamilton quaternion normalised to w ≥ 0, so a
///     rotation has a unique representative (q and -q are the same rotation).
///   - exp/log use the right-trivialised tangent: `log()` returns the rotation
///     vector ω (axis · angle) such that `SO3::exp(ω)` reproduces the rotation.
///   - The log is computed from the quaternion as ω = 2·asin(‖q.vec‖)/‖q.vec‖ ·
///     q.vec, which stays well-conditioned at θ → π (unlike the matrix-trace
///     acos formula). Cross-check: Pinocchio's `quaternion::log3`.
#ifndef TINYSPATIAL_LIEGROUP_SO3_HPP
#define TINYSPATIAL_LIEGROUP_SO3_HPP

#include <algorithm>
#include <cmath>

#include "tinyspatial/core/types.hpp"

namespace tinyspatial {

/// Below this angle (rad) we switch exp/log/Jacobian coefficients to their
/// Taylor expansions, where the closed forms lose precision to cancellation.
inline constexpr Scalar kSmallAngle = 1e-3;

/// Hat operator: maps a 3-vector to the skew-symmetric matrix `[v]_×` such that
/// `[v]_× w = v × w`.
[[nodiscard]] inline Matrix3 skew(const Eigen::Ref<const Vector3>& v) {
  Matrix3 m;
  // clang-format off
  m <<      Scalar(0), -v.z(),      v.y(),
            v.z(),      Scalar(0), -v.x(),
           -v.y(),      v.x(),      Scalar(0);
  // clang-format on
  return m;
}

/// Vee operator: inverse of skew() for a skew-symmetric matrix.
[[nodiscard]] inline Vector3 unskew(const Eigen::Ref<const Matrix3>& m) {
  return Vector3(m(2, 1), m(0, 2), m(1, 0));
}

/// A 3-D rotation.
class SO3 {
 public:
  /// Identity rotation.
  SO3() : quat_(Quaternion::Identity()) {}

  /// Construct from a quaternion (normalised internally; need not be unit).
  explicit SO3(const Quaternion& q) : quat_(canonical(q)) {}

  /// Construct from a rotation matrix (assumed orthonormal; \pre det ≈ +1).
  explicit SO3(const Eigen::Ref<const Matrix3>& rotation)
      : quat_(canonical(Quaternion(rotation))) {}

  /// The identity rotation.
  [[nodiscard]] static SO3 identity() { return SO3(); }

  /// The underlying unit quaternion (w ≥ 0).
  [[nodiscard]] const Quaternion& quaternion() const { return quat_; }

  /// The equivalent 3×3 rotation matrix.
  [[nodiscard]] Matrix3 matrix() const { return quat_.toRotationMatrix(); }

  /// Group composition: `(*this) ∘ rhs`.
  [[nodiscard]] SO3 operator*(const SO3& rhs) const { return SO3(quat_ * rhs.quat_); }

  /// Group inverse (the transpose / conjugate rotation).
  [[nodiscard]] SO3 inverse() const { return SO3(quat_.conjugate()); }

  /// Rotate a point: returns `R · p`.
  [[nodiscard]] Vector3 act(const Eigen::Ref<const Vector3>& p) const { return quat_ * p; }

  /// Exponential map: rotation vector ω (axis·angle) → rotation.
  [[nodiscard]] static SO3 exp(const Eigen::Ref<const Vector3>& omega);

  /// Logarithm map: rotation → rotation vector ω, with ‖ω‖ ∈ [0, π].
  [[nodiscard]] Vector3 log() const;

  /// Left Jacobian Jl(ω): relates a left-trivialised tangent perturbation to
  /// the change in the rotation vector. `Jl(ω) = Jr(ω)ᵀ = Jr(-ω)`.
  [[nodiscard]] static Matrix3 left_jacobian(const Eigen::Ref<const Vector3>& omega);
  /// Right Jacobian Jr(ω).
  [[nodiscard]] static Matrix3 right_jacobian(const Eigen::Ref<const Vector3>& omega);
  /// Inverse of the left Jacobian.
  [[nodiscard]] static Matrix3 left_jacobian_inverse(const Eigen::Ref<const Vector3>& omega);
  /// Inverse of the right Jacobian.
  [[nodiscard]] static Matrix3 right_jacobian_inverse(const Eigen::Ref<const Vector3>& omega);

 private:
  /// Normalise to a unit quaternion with w ≥ 0 (unique representative, §15).
  [[nodiscard]] static Quaternion canonical(Quaternion q) {
    q.normalize();
    if (q.w() < Scalar(0)) {
      q.coeffs() *= Scalar(-1);
    }
    return q;
  }

  Quaternion quat_;
};

// ---------------------------------------------------------------------------
// exp / log
// ---------------------------------------------------------------------------

inline SO3 SO3::exp(const Eigen::Ref<const Vector3>& omega) {
  const Scalar theta2 = omega.squaredNorm();
  const Scalar theta = std::sqrt(theta2);
  Quaternion q;
  if (theta > kSmallAngle) {
    const Scalar half = Scalar(0.5) * theta;
    q.w() = std::cos(half);
    q.vec() = (std::sin(half) / theta) * omega;  // sin(θ/2)/θ · ω
  } else {
    // sin(θ/2)/θ = 1/2 − θ²/48 + …  ;  cos(θ/2) = 1 − θ²/8 + …
    q.w() = Scalar(1) - theta2 / Scalar(8);
    q.vec() = (Scalar(0.5) - theta2 / Scalar(48)) * omega;
  }
  return SO3(q);
}

inline Vector3 SO3::log() const {
  // q is canonical (unit, w ≥ 0), so n = ‖vec‖ = sin(θ/2) and θ = 2·asin(n).
  const Vector3 vec = quat_.vec();
  const Scalar n = std::min(vec.norm(), Scalar(1));  // clamp float drift
  Scalar factor = Scalar(0);
  if (n > kSmallAngle) {
    factor = Scalar(2) * std::asin(n) / n;  // θ / sin(θ/2)
  } else {
    // 2·asin(n)/n = 2·(1 + n²/6 + …)
    factor = Scalar(2) * (Scalar(1) + n * n / Scalar(6));
  }
  return factor * vec;
}

// ---------------------------------------------------------------------------
// Jacobians.  With S = [ω]_× and θ = ‖ω‖:
//   Jr = I − A·S + B·S² ,  Jl = I + A·S + B·S²
//   A = (1 − cosθ)/θ²        B = (θ − sinθ)/θ³
//   Jr⁻¹ = I + ½·S + C·S² ,  Jl⁻¹ = I − ½·S + C·S²
//   C = 1/θ² − (1 + cosθ)/(2θ·sinθ)
// ---------------------------------------------------------------------------

inline Matrix3 SO3::right_jacobian(const Eigen::Ref<const Vector3>& omega) {
  const Scalar theta2 = omega.squaredNorm();
  const Matrix3 s = skew(omega);
  Scalar a = Scalar(0);  // (1 − cosθ)/θ²
  Scalar b = Scalar(0);  // (θ − sinθ)/θ³
  if (theta2 > kSmallAngle * kSmallAngle) {
    const Scalar theta = std::sqrt(theta2);
    a = (Scalar(1) - std::cos(theta)) / theta2;
    b = (theta - std::sin(theta)) / (theta2 * theta);
  } else {
    a = Scalar(0.5) - theta2 / Scalar(24);
    b = Scalar(1) / Scalar(6) - theta2 / Scalar(120);
  }
  return Matrix3::Identity() - a * s + b * (s * s);
}

inline Matrix3 SO3::left_jacobian(const Eigen::Ref<const Vector3>& omega) {
  // Jl(ω) = Jr(ω)ᵀ; since Sᵀ = −S this flips the sign of the S term only.
  const Scalar theta2 = omega.squaredNorm();
  const Matrix3 s = skew(omega);
  Scalar a = Scalar(0);
  Scalar b = Scalar(0);
  if (theta2 > kSmallAngle * kSmallAngle) {
    const Scalar theta = std::sqrt(theta2);
    a = (Scalar(1) - std::cos(theta)) / theta2;
    b = (theta - std::sin(theta)) / (theta2 * theta);
  } else {
    a = Scalar(0.5) - theta2 / Scalar(24);
    b = Scalar(1) / Scalar(6) - theta2 / Scalar(120);
  }
  return Matrix3::Identity() + a * s + b * (s * s);
}

inline Matrix3 SO3::right_jacobian_inverse(const Eigen::Ref<const Vector3>& omega) {
  const Scalar theta2 = omega.squaredNorm();
  const Matrix3 s = skew(omega);
  Scalar c = Scalar(0);  // 1/θ² − (1 + cosθ)/(2θ·sinθ)
  if (theta2 > kSmallAngle * kSmallAngle) {
    const Scalar theta = std::sqrt(theta2);
    c = Scalar(1) / theta2 - (Scalar(1) + std::cos(theta)) / (Scalar(2) * theta * std::sin(theta));
  } else {
    c = Scalar(1) / Scalar(12) + theta2 / Scalar(720);
  }
  return Matrix3::Identity() + Scalar(0.5) * s + c * (s * s);
}

inline Matrix3 SO3::left_jacobian_inverse(const Eigen::Ref<const Vector3>& omega) {
  const Scalar theta2 = omega.squaredNorm();
  const Matrix3 s = skew(omega);
  Scalar c = Scalar(0);
  if (theta2 > kSmallAngle * kSmallAngle) {
    const Scalar theta = std::sqrt(theta2);
    c = Scalar(1) / theta2 - (Scalar(1) + std::cos(theta)) / (Scalar(2) * theta * std::sin(theta));
  } else {
    c = Scalar(1) / Scalar(12) + theta2 / Scalar(720);
  }
  return Matrix3::Identity() - Scalar(0.5) * s + c * (s * s);
}

}  // namespace tinyspatial

#endif  // TINYSPATIAL_LIEGROUP_SO3_HPP
