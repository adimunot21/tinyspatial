/// \file motion.hpp
/// \brief `Motion`: a spatial velocity (twist) in se(3), stored angular-first.
///
/// Distinct from `Force` at the type level so a wrench can never be silently
/// added to a velocity. Both are 6-vectors; their *meaning* differs (twists are
/// covariant under the group adjoint, wrenches under its dual). See the course
/// chapter 05 for why we bother to type them.
#ifndef TINYSPATIAL_SPATIAL_MOTION_HPP
#define TINYSPATIAL_SPATIAL_MOTION_HPP

#include "tinyspatial/core/types.hpp"
#include "tinyspatial/liegroup/se3.hpp"

namespace tinyspatial {

/// A spatial motion (twist). Indices 0..2 are angular ω, 3..5 are linear v.
class Motion {
 public:
  /// Zero motion (the additive identity).
  Motion() : data_(Vector6::Zero()) {}

  /// Wrap an existing 6-vector. \pre `v` is already in (ω; v) ordering.
  explicit Motion(const Eigen::Ref<const Vector6>& v) : data_(v) {}

  /// Build from explicit angular / linear parts.
  Motion(const Eigen::Ref<const Vector3>& angular, const Eigen::Ref<const Vector3>& linear) {
    data_.head<3>() = angular;
    data_.tail<3>() = linear;
  }

  /// The zero motion.
  [[nodiscard]] static Motion zero() { return Motion(); }

  [[nodiscard]] Vector3 angular() const { return data_.head<3>(); }
  [[nodiscard]] Vector3 linear() const { return data_.tail<3>(); }
  /// The underlying 6-vector in (ω; v) ordering.
  [[nodiscard]] const Vector6& vector() const { return data_; }

  [[nodiscard]] Motion operator+(const Motion& rhs) const { return Motion(data_ + rhs.data_); }
  [[nodiscard]] Motion operator-(const Motion& rhs) const { return Motion(data_ - rhs.data_); }
  [[nodiscard]] Motion operator-() const { return Motion(-data_); }
  [[nodiscard]] Motion operator*(Scalar k) const { return Motion(data_ * k); }
  Motion& operator+=(const Motion& rhs) {
    data_ += rhs.data_;
    return *this;
  }
  Motion& operator-=(const Motion& rhs) {
    data_ -= rhs.data_;
    return *this;
  }

 private:
  Vector6 data_;
};

/// Left scalar multiplication.
[[nodiscard]] inline Motion operator*(Scalar k, const Motion& m) {
  return m * k;
}

/// SE(3) action on a motion: `M' = Ad_T · M`.
[[nodiscard]] inline Motion operator*(const SE3& t, const Motion& m) {
  return Motion(t.adjoint() * m.vector());
}

}  // namespace tinyspatial

#endif  // TINYSPATIAL_SPATIAL_MOTION_HPP
