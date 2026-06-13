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
///
/// The class is templated on the scalar (`SO3T<S>`), with `SO3 = SO3T<double>`.
/// `double` is the type the public API and all algorithms are written against;
/// instantiating on `Jet<N>` (core/jet.hpp) makes every rotation operation
/// differentiable. Two rules keep that working: transcendentals are called
/// through `using std::fn; fn(x)` so ADL finds the `Jet` overload, and every
/// branch predicate compares the scalar value only (a `Jet` carries its
/// derivative through whichever branch its value selects).
#ifndef TINYSPATIAL_LIEGROUP_SO3_HPP
#define TINYSPATIAL_LIEGROUP_SO3_HPP

#include <algorithm>
#include <cmath>

#include "tinyspatial/core/types.hpp"

namespace tinyspatial {

/// Below this angle (rad) we switch exp/log/Jacobian coefficients to their
/// Taylor expansions, where the closed forms lose precision to cancellation.
/// A plain `double`: branch predicates compare it against the scalar value.
inline constexpr double kSmallAngle = 1e-3;

/// Hat operator: maps a 3-vector to the skew-symmetric matrix `[v]_×` such that
/// `[v]_× w = v × w`. Deduces the scalar from the argument so it works for both
/// `double` and an autodiff `Jet`.
template <typename Derived>
[[nodiscard]] Eigen::Matrix<typename Derived::Scalar, 3, 3> skew(
    const Eigen::MatrixBase<Derived>& v) {
  using S = typename Derived::Scalar;
  Eigen::Matrix<S, 3, 3> m;
  // clang-format off
  m <<   S(0), -v.z(),  v.y(),
        v.z(),   S(0), -v.x(),
       -v.y(),  v.x(),   S(0);
  // clang-format on
  return m;
}

/// Vee operator: inverse of skew() for a skew-symmetric matrix.
template <typename Derived>
[[nodiscard]] Eigen::Matrix<typename Derived::Scalar, 3, 1> unskew(
    const Eigen::MatrixBase<Derived>& m) {
  return Eigen::Matrix<typename Derived::Scalar, 3, 1>(m(2, 1), m(0, 2), m(1, 0));
}

/// A 3-D rotation, templated on the scalar type.
template <typename S>
class SO3T {
 public:
  using Scalar = S;
  using Vector3 = typename Types<S>::Vector3;
  using Matrix3 = typename Types<S>::Matrix3;
  using Quaternion = typename Types<S>::Quaternion;

  /// Identity rotation.
  SO3T() : quat_(Quaternion::Identity()) {}

  /// Construct from a quaternion (normalised internally; need not be unit).
  explicit SO3T(const Quaternion& q) : quat_(canonical(q)) {}

  /// Construct from a rotation matrix (assumed orthonormal; \pre det ≈ +1).
  explicit SO3T(const Eigen::Ref<const Matrix3>& rotation)
      : quat_(canonical(Quaternion(rotation))) {}

  /// The identity rotation.
  [[nodiscard]] static SO3T identity() { return SO3T(); }

  /// The underlying unit quaternion (w ≥ 0).
  [[nodiscard]] const Quaternion& quaternion() const { return quat_; }

  /// The equivalent 3×3 rotation matrix.
  [[nodiscard]] Matrix3 matrix() const { return quat_.toRotationMatrix(); }

  /// Cast to another scalar type (e.g. lift a constant rotation into an
  /// autodiff scalar with zero derivative).
  template <typename S2>
  [[nodiscard]] SO3T<S2> cast() const {
    return SO3T<S2>(typename Types<S2>::Quaternion(quat_.coeffs().template cast<S2>()));
  }

  /// Group composition: `(*this) ∘ rhs`.
  [[nodiscard]] SO3T operator*(const SO3T& rhs) const { return SO3T(quat_ * rhs.quat_); }

  /// Group inverse (the transpose / conjugate rotation).
  [[nodiscard]] SO3T inverse() const { return SO3T(quat_.conjugate()); }

  /// Rotate a point: returns `R · p`.
  [[nodiscard]] Vector3 act(const Eigen::Ref<const Vector3>& p) const { return quat_ * p; }

  /// Exponential map: rotation vector ω (axis·angle) → rotation.
  [[nodiscard]] static SO3T exp(const Eigen::Ref<const Vector3>& omega) {
    using std::cos;
    using std::sin;
    using std::sqrt;
    const Scalar theta2 = omega.squaredNorm();
    const Scalar theta = sqrt(theta2);
    Quaternion q;
    if (theta > kSmallAngle) {
      const Scalar half = Scalar(0.5) * theta;
      q.w() = cos(half);
      q.vec() = (sin(half) / theta) * omega;  // sin(θ/2)/θ · ω
    } else {
      // sin(θ/2)/θ = 1/2 − θ²/48 + … ;  cos(θ/2) = 1 − θ²/8 + …
      q.w() = Scalar(1) - theta2 / Scalar(8);
      q.vec() = (Scalar(0.5) - theta2 / Scalar(48)) * omega;
    }
    return SO3T(q);
  }

  /// Logarithm map: rotation → rotation vector ω, with ‖ω‖ ∈ [0, π].
  [[nodiscard]] Vector3 log() const {
    using std::asin;
    using std::min;
    // q is canonical (unit, w ≥ 0), so n = ‖vec‖ = sin(θ/2) and θ = 2·asin(n).
    const Vector3 vec = quat_.vec();
    const Scalar n = min(Scalar(vec.norm()), Scalar(1));  // clamp float drift
    Scalar factor(0);
    if (n > kSmallAngle) {
      factor = Scalar(2) * asin(n) / n;  // θ / sin(θ/2)
    } else {
      // 2·asin(n)/n = 2·(1 + n²/6 + …)
      factor = Scalar(2) * (Scalar(1) + n * n / Scalar(6));
    }
    return factor * vec;
  }

  // Jacobians.  With S = [ω]_× and θ = ‖ω‖:
  //   Jr = I − A·S + B·S² ,  Jl = I + A·S + B·S²
  //   A = (1 − cosθ)/θ²        B = (θ − sinθ)/θ³
  //   Jr⁻¹ = I + ½·S + C·S² ,  Jl⁻¹ = I − ½·S + C·S²
  //   C = 1/θ² − (1 + cosθ)/(2θ·sinθ)

  /// Left Jacobian Jl(ω). `Jl(ω) = Jr(ω)ᵀ = Jr(-ω)`.
  [[nodiscard]] static Matrix3 left_jacobian(const Eigen::Ref<const Vector3>& omega) {
    const Matrix3 s = skew(omega);
    Scalar a(0), b(0);
    ab_coeffs(omega.squaredNorm(), a, b);
    // Jl(ω) = Jr(ω)ᵀ; since Sᵀ = −S this flips the sign of the S term only.
    return Matrix3::Identity() + a * s + b * (s * s);
  }

  /// Right Jacobian Jr(ω).
  [[nodiscard]] static Matrix3 right_jacobian(const Eigen::Ref<const Vector3>& omega) {
    const Matrix3 s = skew(omega);
    Scalar a(0), b(0);
    ab_coeffs(omega.squaredNorm(), a, b);
    return Matrix3::Identity() - a * s + b * (s * s);
  }

  /// Inverse of the left Jacobian.
  [[nodiscard]] static Matrix3 left_jacobian_inverse(const Eigen::Ref<const Vector3>& omega) {
    const Matrix3 s = skew(omega);
    return Matrix3::Identity() - Scalar(0.5) * s + c_coeff(omega.squaredNorm()) * (s * s);
  }

  /// Inverse of the right Jacobian.
  [[nodiscard]] static Matrix3 right_jacobian_inverse(const Eigen::Ref<const Vector3>& omega) {
    const Matrix3 s = skew(omega);
    return Matrix3::Identity() + Scalar(0.5) * s + c_coeff(omega.squaredNorm()) * (s * s);
  }

 private:
  /// Normalise to a unit quaternion with w ≥ 0 (unique representative, §15).
  [[nodiscard]] static Quaternion canonical(Quaternion q) {
    q.normalize();
    if (q.w() < Scalar(0)) {
      q.coeffs() *= Scalar(-1);
    }
    return q;
  }

  /// Shared A/B coefficients for the left/right Jacobians.
  static void ab_coeffs(const Scalar& theta2, Scalar& a, Scalar& b) {
    using std::cos;
    using std::sin;
    using std::sqrt;
    if (theta2 > kSmallAngle * kSmallAngle) {
      const Scalar theta = sqrt(theta2);
      a = (Scalar(1) - cos(theta)) / theta2;
      b = (theta - sin(theta)) / (theta2 * theta);
    } else {
      a = Scalar(0.5) - theta2 / Scalar(24);
      b = Scalar(1) / Scalar(6) - theta2 / Scalar(120);
    }
  }

  /// Shared C coefficient for the inverse Jacobians.
  [[nodiscard]] static Scalar c_coeff(const Scalar& theta2) {
    using std::cos;
    using std::sin;
    using std::sqrt;
    if (theta2 > kSmallAngle * kSmallAngle) {
      const Scalar theta = sqrt(theta2);
      return Scalar(1) / theta2 - (Scalar(1) + cos(theta)) / (Scalar(2) * theta * sin(theta));
    }
    return Scalar(1) / Scalar(12) + theta2 / Scalar(720);
  }

  Quaternion quat_;
};

/// The double-precision rotation the public API and algorithms use.
using SO3 = SO3T<double>;

}  // namespace tinyspatial

#endif  // TINYSPATIAL_LIEGROUP_SO3_HPP
