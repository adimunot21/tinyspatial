/// \file cross.hpp
/// \brief Spatial cross-product operators (Featherstone's `×` and `×*`).
///
/// These are the Lie-algebra adjoint of se(3): for a motion vector m,
/// `cross_motion(m)` is `ad_m`, and `d/dt Ad(exp(t·m))|₀ = cross_motion(m)`.
/// All operators use the ANGULAR-FIRST convention (ω on top, v below), to match
/// SE3::adjoint() (CLAUDE.md §5). They live here, beside the Lie groups, because
/// they are defined directly in terms of the SE(3) adjoint structure.
///
/// `cross_motion`/`cross_force` deduce their scalar from the argument (so they
/// serve both `double` and an autodiff `Jet`); the typed `cross(...)` overloads
/// are templated on the scalar (`MotionT<S>` / `ForceT<S>`).
#ifndef TINYSPATIAL_SPATIAL_CROSS_HPP
#define TINYSPATIAL_SPATIAL_CROSS_HPP

#include "tinyspatial/core/types.hpp"
#include "tinyspatial/liegroup/so3.hpp"
#include "tinyspatial/spatial/force.hpp"
#include "tinyspatial/spatial/motion.hpp"

namespace tinyspatial {

/// Motion cross product `m ×` (Featherstone). With m = (ω; v):
/// `[[ [ω]_×, 0 ], [ [v]_×, [ω]_× ]]`. Acts on motion (twist) vectors.
template <typename Derived>
[[nodiscard]] Eigen::Matrix<typename Derived::Scalar, 6, 6> cross_motion(
    const Eigen::MatrixBase<Derived>& m) {
  using S = typename Derived::Scalar;
  const Eigen::Matrix<S, 3, 3> w = skew(m.template head<3>());
  const Eigen::Matrix<S, 3, 3> vx = skew(m.template tail<3>());
  Eigen::Matrix<S, 6, 6> x = Eigen::Matrix<S, 6, 6>::Zero();
  x.template topLeftCorner<3, 3>() = w;
  x.template bottomRightCorner<3, 3>() = w;
  x.template bottomLeftCorner<3, 3>() = vx;
  return x;
}

/// Force cross product `m ×*` (Featherstone), equal to `-(m ×)ᵀ`. With
/// m = (ω; v): `[[ [ω]_×, [v]_× ], [ 0, [ω]_× ]]`. Acts on force (wrench)
/// vectors.
template <typename Derived>
[[nodiscard]] Eigen::Matrix<typename Derived::Scalar, 6, 6> cross_force(
    const Eigen::MatrixBase<Derived>& m) {
  using S = typename Derived::Scalar;
  const Eigen::Matrix<S, 3, 3> w = skew(m.template head<3>());
  const Eigen::Matrix<S, 3, 3> vx = skew(m.template tail<3>());
  Eigen::Matrix<S, 6, 6> x = Eigen::Matrix<S, 6, 6>::Zero();
  x.template topLeftCorner<3, 3>() = w;
  x.template bottomRightCorner<3, 3>() = w;
  x.template topRightCorner<3, 3>() = vx;
  return x;
}

/// Typed motion-on-motion cross product: returns the Lie bracket
/// `[v, m] = v × m` as a `Motion`. Antisymmetric: `cross(a, b) = -cross(b, a)`.
template <typename S>
[[nodiscard]] MotionT<S> cross(const MotionT<S>& v, const MotionT<S>& m) {
  return MotionT<S>(cross_motion(v.vector()) * m.vector());
}

/// Typed motion-on-force cross product: returns `v ×* f` as a `Force`.
template <typename S>
[[nodiscard]] ForceT<S> cross(const MotionT<S>& v, const ForceT<S>& f) {
  return ForceT<S>(cross_force(v.vector()) * f.vector());
}

}  // namespace tinyspatial

#endif  // TINYSPATIAL_SPATIAL_CROSS_HPP
