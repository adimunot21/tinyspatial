/// \file plucker.hpp
/// \brief Plücker transforms — the 6×6 matrices that transport spatial
/// quantities between frames.
///
/// They are *the same* objects as the SE(3) adjoint and its dual; this file
/// just gives them their Featherstone names so the dynamics chapters can speak
/// the textbook's language. For a motion vector, the Plücker transform is
/// `Ad_T`; for a force, it is the dual `Ad_T⁻ᵀ`.
///
/// Templated on the scalar so CRBA/ABA can be differentiated; the `double`
/// aliases are the default instantiation.
#ifndef TINYSPATIAL_SPATIAL_PLUCKER_HPP
#define TINYSPATIAL_SPATIAL_PLUCKER_HPP

#include "tinyspatial/core/types.hpp"
#include "tinyspatial/liegroup/se3.hpp"
#include "tinyspatial/liegroup/so3.hpp"

namespace tinyspatial {

/// The motion Plücker transform `X = Ad_T` (angular-first).
template <typename S>
[[nodiscard]] typename Types<S>::Matrix6 motion_plucker(const SE3T<S>& t) {
  return t.adjoint();
}

/// The force Plücker transform `X* = Ad_T⁻ᵀ` (angular-first). With
/// `T = (R, t)`, this is `[[R, [t]_× R], [0, R]]`.
template <typename S>
[[nodiscard]] typename Types<S>::Matrix6 force_plucker(const SE3T<S>& t) {
  using Matrix3 = typename Types<S>::Matrix3;
  using Matrix6 = typename Types<S>::Matrix6;
  const Matrix3 r = t.rotation().matrix();
  Matrix6 x = Matrix6::Zero();
  x.template topLeftCorner<3, 3>() = r;
  x.template bottomRightCorner<3, 3>() = r;
  x.template topRightCorner<3, 3>() = skew(t.translation()) * r;
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
template <typename S>
[[nodiscard]] typename Types<S>::Matrix6X force_plucker_apply_matrix(
    const SE3T<S>& t, const Eigen::Ref<const typename Types<S>::Matrix6X>& m) {
  using Matrix3 = typename Types<S>::Matrix3;
  using Vector3 = typename Types<S>::Vector3;
  using Matrix6X = typename Types<S>::Matrix6X;
  const Matrix3 r = t.rotation().matrix();
  const Vector3& tr = t.translation();
  Matrix6X out(6, m.cols());
  out.template bottomRows<3>().noalias() = r * m.template bottomRows<3>();
  out.template topRows<3>().noalias() = r * m.template topRows<3>();
  // Add t × (R · m_bot) row by row.
  for (Eigen::Index k = 0; k < m.cols(); ++k) {
    out.template block<3, 1>(0, k) += tr.cross(out.template block<3, 1>(3, k));
  }
  return out;
}

}  // namespace tinyspatial

#endif  // TINYSPATIAL_SPATIAL_PLUCKER_HPP
