/// \file cross.hpp
/// \brief Spatial cross-product operators (Featherstone's `×` and `×*`).
///
/// These are the Lie-algebra adjoint of se(3): for a motion vector m,
/// `cross_motion(m)` is `ad_m`, and `d/dt Ad(exp(t·m))|₀ = cross_motion(m)`.
/// All operators use the ANGULAR-FIRST convention (ω on top, v below), to match
/// SE3::adjoint() (CLAUDE.md §5). They live here, beside the Lie groups, because
/// they are defined directly in terms of the SE(3) adjoint structure.
#ifndef TINYSPATIAL_SPATIAL_CROSS_HPP
#define TINYSPATIAL_SPATIAL_CROSS_HPP

#include "tinyspatial/core/types.hpp"
#include "tinyspatial/liegroup/so3.hpp"

namespace tinyspatial {

/// Motion cross product `m ×` (Featherstone). With m = (ω; v):
/// `[[ [ω]_×, 0 ], [ [v]_×, [ω]_× ]]`. Acts on motion (twist) vectors.
[[nodiscard]] inline Matrix6 cross_motion(const Eigen::Ref<const Vector6>& m) {
  const Matrix3 w = skew(m.head<3>());
  const Matrix3 vx = skew(m.tail<3>());
  Matrix6 x = Matrix6::Zero();
  x.topLeftCorner<3, 3>() = w;
  x.bottomRightCorner<3, 3>() = w;
  x.bottomLeftCorner<3, 3>() = vx;
  return x;
}

/// Force cross product `m ×*` (Featherstone), equal to `-(m ×)ᵀ`. With
/// m = (ω; v): `[[ [ω]_×, [v]_× ], [ 0, [ω]_× ]]`. Acts on force (wrench)
/// vectors.
[[nodiscard]] inline Matrix6 cross_force(const Eigen::Ref<const Vector6>& m) {
  const Matrix3 w = skew(m.head<3>());
  const Matrix3 vx = skew(m.tail<3>());
  Matrix6 x = Matrix6::Zero();
  x.topLeftCorner<3, 3>() = w;
  x.bottomRightCorner<3, 3>() = w;
  x.topRightCorner<3, 3>() = vx;
  return x;
}

}  // namespace tinyspatial

#endif  // TINYSPATIAL_SPATIAL_CROSS_HPP
