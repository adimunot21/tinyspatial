/// \file motion.hpp
/// \brief `Motion`: a spatial velocity (twist) in se(3), stored angular-first.
///
/// Distinct from `Force` at the type level so a wrench can never be silently
/// added to a velocity. Both are 6-vectors; their *meaning* differs (twists are
/// covariant under the group adjoint, wrenches under its dual). See the course
/// chapter 05 for why we bother to type them.
///
/// Templated on the scalar (`MotionT<S>`, with `Motion = MotionT<double>`) so it
/// can carry an autodiff `Jet`; the body is pure arithmetic, no transcendentals.
#ifndef TINYSPATIAL_SPATIAL_MOTION_HPP
#define TINYSPATIAL_SPATIAL_MOTION_HPP

#include "tinyspatial/core/types.hpp"
#include "tinyspatial/liegroup/se3.hpp"

namespace tinyspatial {

/// A spatial motion (twist). Indices 0..2 are angular ω, 3..5 are linear v.
template <typename S>
class MotionT {
 public:
  using Scalar = S;
  using Vector3 = typename Types<S>::Vector3;
  using Vector6 = typename Types<S>::Vector6;
  using Matrix3 = typename Types<S>::Matrix3;

  /// Zero motion (the additive identity).
  MotionT() : data_(Vector6::Zero()) {}

  /// Wrap an existing 6-vector. \pre `v` is already in (ω; v) ordering.
  explicit MotionT(const Eigen::Ref<const Vector6>& v) : data_(v) {}

  /// Build from explicit angular / linear parts.
  MotionT(const Eigen::Ref<const Vector3>& angular, const Eigen::Ref<const Vector3>& linear) {
    data_.template head<3>() = angular;
    data_.template tail<3>() = linear;
  }

  /// The zero motion.
  [[nodiscard]] static MotionT zero() { return MotionT(); }

  [[nodiscard]] Vector3 angular() const { return data_.template head<3>(); }
  [[nodiscard]] Vector3 linear() const { return data_.template tail<3>(); }
  /// The underlying 6-vector in (ω; v) ordering.
  [[nodiscard]] const Vector6& vector() const { return data_; }

  [[nodiscard]] MotionT operator+(const MotionT& rhs) const { return MotionT(data_ + rhs.data_); }
  [[nodiscard]] MotionT operator-(const MotionT& rhs) const { return MotionT(data_ - rhs.data_); }
  [[nodiscard]] MotionT operator-() const { return MotionT(-data_); }
  [[nodiscard]] MotionT operator*(const S& k) const { return MotionT(data_ * k); }
  MotionT& operator+=(const MotionT& rhs) {
    data_ += rhs.data_;
    return *this;
  }
  MotionT& operator-=(const MotionT& rhs) {
    data_ -= rhs.data_;
    return *this;
  }

 private:
  Vector6 data_;
};

/// The double-precision twist used by the public API and algorithms.
using Motion = MotionT<double>;

/// Left scalar multiplication.
template <typename S>
[[nodiscard]] MotionT<S> operator*(const S& k, const MotionT<S>& m) {
  return m * k;
}

/// SE(3) action on a motion: `M' = Ad_T · M`.
///
/// Inline expansion of `Ad_T = [[R, 0], [[t]_x R, R]]`:
///   new_w = R * w
///   new_v = R * v + t × new_w
/// 24 ops vs constructing a 6×6 adjoint and multiplying (72 ops + temporary).
template <typename S>
[[nodiscard]] MotionT<S> operator*(const SE3T<S>& t, const MotionT<S>& m) {
  const typename Types<S>::Matrix3 r = t.rotation().matrix();
  const typename Types<S>::Vector3 new_w = r * m.angular();
  const typename Types<S>::Vector3 new_v = r * m.linear() + t.translation().cross(new_w);
  return MotionT<S>(new_w, new_v);
}

}  // namespace tinyspatial

#endif  // TINYSPATIAL_SPATIAL_MOTION_HPP
