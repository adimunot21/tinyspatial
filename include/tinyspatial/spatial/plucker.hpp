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

/// Apply the force Plücker transform `X* = Ad_T⁻ᵀ` to a 6×N matrix without
/// building the 6×6 X* first. With X* = [[R, [t]×R], [0, R]] acting on
/// `m = [[m_top], [m_bot]]`:
///
///   new_top = R · m_top + t × (R · m_bot)
///   new_bot = R · m_bot
///
/// Saves ~36 ops + the 6×6 zero-fill compared to `force_plucker(t) * m`.
/// Used in CRBA's parent-chain walk where `m` is the joint-subspace
/// force matrix (6 × nv_i, typically 6×1).
[[nodiscard]] inline Matrix6X force_plucker_apply_matrix(const SE3& t,
                                                         const Eigen::Ref<const Matrix6X>& m) {
  const Matrix3 r = t.rotation().matrix();
  const Vector3& tr = t.translation();
  Matrix6X out(6, m.cols());
  out.bottomRows<3>().noalias() = r * m.bottomRows<3>();
  out.topRows<3>().noalias() = r * m.topRows<3>();
  // Add t × (R · m_bot) row by row.
  for (Eigen::Index k = 0; k < m.cols(); ++k) {
    out.template block<3, 1>(0, k) += tr.cross(out.template block<3, 1>(3, k));
  }
  return out;
}

}  // namespace tinyspatial

#endif  // TINYSPATIAL_SPATIAL_PLUCKER_HPP
