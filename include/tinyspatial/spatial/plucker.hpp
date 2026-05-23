/// \file plucker.hpp
/// \brief Plücker transforms — the 6×6 matrices that transport spatial
/// quantities between frames.
///
/// They are *the same* objects as the SE(3) adjoint and its dual; this file
/// just gives them their Featherstone names so the dynamics chapters can speak
/// the textbook's language. For a motion vector, the Plücker transform is
/// `Ad_T`; for a force, it is the dual `Ad_T⁻ᵀ`.
#ifndef TINYSPATIAL_SPATIAL_PLUCKER_HPP
#define TINYSPATIAL_SPATIAL_PLUCKER_HPP

#include "tinyspatial/core/types.hpp"
#include "tinyspatial/liegroup/se3.hpp"
#include "tinyspatial/liegroup/so3.hpp"

namespace tinyspatial {

/// The motion Plücker transform `X = Ad_T` (angular-first).
[[nodiscard]] inline Matrix6 motion_plucker(const SE3& t) {
  return t.adjoint();
}

/// The force Plücker transform `X* = Ad_T⁻ᵀ` (angular-first). With
/// `T = (R, t)`, this is `[[R, [t]_× R], [0, R]]`.
[[nodiscard]] inline Matrix6 force_plucker(const SE3& t) {
  const Matrix3 r = t.rotation().matrix();
  Matrix6 x = Matrix6::Zero();
  x.topLeftCorner<3, 3>() = r;
  x.bottomRightCorner<3, 3>() = r;
  x.topRightCorner<3, 3>() = skew(t.translation()) * r;
  return x;
}

}  // namespace tinyspatial

#endif  // TINYSPATIAL_SPATIAL_PLUCKER_HPP
