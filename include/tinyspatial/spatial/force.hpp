/// \file force.hpp
/// \brief `Force`: a spatial force (wrench) in se(3)*, stored angular-first.
///
/// Distinct from `Motion` at the type level. Wrenches transform under the dual
/// adjoint `Ad_T⁻ᵀ`, not the adjoint itself — getting that wrong is a classic
/// silent bug, and the typed distinction makes it a compile error instead.
#ifndef TINYSPATIAL_SPATIAL_FORCE_HPP
#define TINYSPATIAL_SPATIAL_FORCE_HPP

#include "tinyspatial/core/types.hpp"
#include "tinyspatial/liegroup/se3.hpp"

namespace tinyspatial {

/// A spatial force (wrench). Indices 0..2 are angular (torque/moment),
/// 3..5 are linear (force).
class Force {
 public:
  /// Zero wrench.
  Force() : data_(Vector6::Zero()) {}

  /// Wrap an existing 6-vector. \pre `v` is already in (τ; f) ordering.
  explicit Force(const Eigen::Ref<const Vector6>& v) : data_(v) {}

  /// Build from explicit moment / linear-force parts.
  Force(const Eigen::Ref<const Vector3>& moment, const Eigen::Ref<const Vector3>& linear) {
    data_.head<3>() = moment;
    data_.tail<3>() = linear;
  }

  /// The zero wrench.
  [[nodiscard]] static Force zero() { return Force(); }

  /// The moment (angular) part.
  [[nodiscard]] Vector3 angular() const { return data_.head<3>(); }
  /// The linear-force part.
  [[nodiscard]] Vector3 linear() const { return data_.tail<3>(); }
  /// The underlying 6-vector in (τ; f) ordering.
  [[nodiscard]] const Vector6& vector() const { return data_; }

  [[nodiscard]] Force operator+(const Force& rhs) const { return Force(data_ + rhs.data_); }
  [[nodiscard]] Force operator-(const Force& rhs) const { return Force(data_ - rhs.data_); }
  [[nodiscard]] Force operator-() const { return Force(-data_); }
  [[nodiscard]] Force operator*(Scalar k) const { return Force(data_ * k); }
  Force& operator+=(const Force& rhs) {
    data_ += rhs.data_;
    return *this;
  }
  Force& operator-=(const Force& rhs) {
    data_ -= rhs.data_;
    return *this;
  }

 private:
  Vector6 data_;
};

[[nodiscard]] inline Force operator*(Scalar k, const Force& f) {
  return f * k;
}

/// SE(3) action on a wrench: `F' = Ad_T⁻ᵀ · F`. The dual (force) adjoint in
/// angular-first ordering is `[[R, [t]_× R], [0, R]]`.
[[nodiscard]] inline Force operator*(const SE3& t, const Force& f) {
  const Matrix3 r = t.rotation().matrix();
  Matrix6 dual = Matrix6::Zero();
  dual.topLeftCorner<3, 3>() = r;
  dual.bottomRightCorner<3, 3>() = r;
  dual.topRightCorner<3, 3>() = skew(t.translation()) * r;
  return Force(dual * f.vector());
}

}  // namespace tinyspatial

#endif  // TINYSPATIAL_SPATIAL_FORCE_HPP
