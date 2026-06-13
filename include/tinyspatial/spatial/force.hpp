/// \file force.hpp
/// \brief `Force`: a spatial force (wrench) in se(3)*, stored angular-first.
///
/// Distinct from `Motion` at the type level. Wrenches transform under the dual
/// adjoint `Ad_T⁻ᵀ`, not the adjoint itself — getting that wrong is a classic
/// silent bug, and the typed distinction makes it a compile error instead.
///
/// Templated on the scalar (`ForceT<S>`, with `Force = ForceT<double>`).
#ifndef TINYSPATIAL_SPATIAL_FORCE_HPP
#define TINYSPATIAL_SPATIAL_FORCE_HPP

#include "tinyspatial/core/types.hpp"
#include "tinyspatial/liegroup/se3.hpp"

namespace tinyspatial {

/// A spatial force (wrench). Indices 0..2 are angular (torque/moment),
/// 3..5 are linear (force).
template <typename S>
class ForceT {
 public:
  using Scalar = S;
  using Vector3 = typename Types<S>::Vector3;
  using Vector6 = typename Types<S>::Vector6;

  /// Zero wrench.
  ForceT() : data_(Vector6::Zero()) {}

  /// Wrap an existing 6-vector. \pre `v` is already in (τ; f) ordering.
  explicit ForceT(const Eigen::Ref<const Vector6>& v) : data_(v) {}

  /// Build from explicit moment / linear-force parts.
  ForceT(const Eigen::Ref<const Vector3>& moment, const Eigen::Ref<const Vector3>& linear) {
    data_.template head<3>() = moment;
    data_.template tail<3>() = linear;
  }

  /// The zero wrench.
  [[nodiscard]] static ForceT zero() { return ForceT(); }

  /// The moment (angular) part.
  [[nodiscard]] Vector3 angular() const { return data_.template head<3>(); }
  /// The linear-force part.
  [[nodiscard]] Vector3 linear() const { return data_.template tail<3>(); }
  /// The underlying 6-vector in (τ; f) ordering.
  [[nodiscard]] const Vector6& vector() const { return data_; }

  [[nodiscard]] ForceT operator+(const ForceT& rhs) const { return ForceT(data_ + rhs.data_); }
  [[nodiscard]] ForceT operator-(const ForceT& rhs) const { return ForceT(data_ - rhs.data_); }
  [[nodiscard]] ForceT operator-() const { return ForceT(-data_); }
  [[nodiscard]] ForceT operator*(const S& k) const { return ForceT(data_ * k); }
  ForceT& operator+=(const ForceT& rhs) {
    data_ += rhs.data_;
    return *this;
  }
  ForceT& operator-=(const ForceT& rhs) {
    data_ -= rhs.data_;
    return *this;
  }

 private:
  Vector6 data_;
};

/// The double-precision wrench used by the public API and algorithms.
using Force = ForceT<double>;

template <typename S>
[[nodiscard]] ForceT<S> operator*(const S& k, const ForceT<S>& f) {
  return f * k;
}

/// SE(3) action on a wrench: `F' = Ad_T⁻ᵀ · F`. The dual (force) adjoint in
/// angular-first ordering is `[[R, [t]_× R], [0, R]]`.
///
/// Inline expansion:
///   new_f_linear  = R · f_linear           (lower-right block)
///   new_f_torque  = R · f_angular + t × new_f_linear
/// 24 ops vs constructing the 6×6 dual adjoint and multiplying (~80 ops).
template <typename S>
[[nodiscard]] ForceT<S> operator*(const SE3T<S>& t, const ForceT<S>& f) {
  const typename Types<S>::Matrix3 r = t.rotation().matrix();
  const typename Types<S>::Vector3 new_linear = r * f.linear();
  const typename Types<S>::Vector3 new_angular =
      r * f.angular() + t.translation().cross(new_linear);
  return ForceT<S>(new_angular, new_linear);
}

}  // namespace tinyspatial

#endif  // TINYSPATIAL_SPATIAL_FORCE_HPP
