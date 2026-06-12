/// \file inertia.hpp
/// \brief `SpatialInertia`: a rigid body's mass distribution, in se(3)-space.
///
/// Stored *separably* — mass, COM, and the rotational inertia about COM — so
/// that an SE(3) transform moves each piece cleanly and the stored form never
/// accumulates symmetrisation drift the way a 6×6 matrix would. The 6×6
/// spatial inertia matrix is built on demand via `matrix6()`.
///
/// In angular-first ordering, with `c× = skew(COM)`:
///
///   I_spatial = [[ Ī + m·(‖c‖²·I − c·cᵀ),   m·c× ],
///                [ m·c×ᵀ,                    m·I  ]]
///
/// where Ī is the inertia about the COM (`inertia_com()`). The off-diagonal
/// blocks differ by a sign (c× = −c×ᵀ) so the full matrix is symmetric.
///
/// Templated on the scalar (`SpatialInertiaT<S>`, `SpatialInertia = …<double>`).
#ifndef TINYSPATIAL_SPATIAL_INERTIA_HPP
#define TINYSPATIAL_SPATIAL_INERTIA_HPP

#include "tinyspatial/core/types.hpp"
#include "tinyspatial/liegroup/se3.hpp"
#include "tinyspatial/spatial/force.hpp"
#include "tinyspatial/spatial/motion.hpp"

namespace tinyspatial {

/// The spatial inertia of a rigid body, expressed at the body-frame origin.
template <typename S>
class SpatialInertiaT {
 public:
  using Scalar = S;
  using Vector3 = typename Types<S>::Vector3;
  using Matrix3 = typename Types<S>::Matrix3;
  using Matrix6 = typename Types<S>::Matrix6;

  /// Zero mass (the additive identity for the composite-body algorithm).
  SpatialInertiaT() : mass_(0), com_(Vector3::Zero()), inertia_com_(Matrix3::Zero()) {}

  /// Build from explicit parts. \pre `inertia_com` is symmetric PSD, `mass ≥ 0`.
  SpatialInertiaT(S mass, const Eigen::Ref<const Vector3>& com,
                  const Eigen::Ref<const Matrix3>& inertia_com)
      : mass_(mass), com_(com), inertia_com_(inertia_com) {}

  /// The zero-mass element.
  [[nodiscard]] static SpatialInertiaT zero() { return SpatialInertiaT(); }

  [[nodiscard]] S mass() const { return mass_; }
  [[nodiscard]] const Vector3& com() const { return com_; }
  /// Rotational inertia tensor about the centre of mass.
  [[nodiscard]] const Matrix3& inertia_com() const { return inertia_com_; }

  /// Cast the (constant) inertia to another scalar type — used to lift a
  /// double-precision model into an autodiff scalar with zero derivative.
  template <typename S2>
  [[nodiscard]] SpatialInertiaT<S2> cast() const {
    return SpatialInertiaT<S2>(S2(mass_), com_.template cast<S2>(),
                               inertia_com_.template cast<S2>());
  }

  /// Rotational inertia about the body-frame origin (parallel-axis theorem).
  /// Ī_O = Ī_com + m·(‖c‖² I − c·cᵀ) = Ī_com − m·c× c×.
  [[nodiscard]] Matrix3 inertia_origin() const {
    const Matrix3 cx = skew(com_);
    return inertia_com_ - mass_ * cx * cx;
  }

  /// The full 6×6 spatial inertia matrix at the body-frame origin.
  [[nodiscard]] Matrix6 matrix6() const {
    const Matrix3 cx = skew(com_);
    Matrix6 i = Matrix6::Zero();
    i.template topLeftCorner<3, 3>() = inertia_origin();
    i.template topRightCorner<3, 3>() = mass_ * cx;
    i.template bottomLeftCorner<3, 3>() = mass_ * cx.transpose();
    i.template bottomRightCorner<3, 3>() = mass_ * Matrix3::Identity();
    return i;
  }

  /// Addition (composite-body inertia): valid when both inertias are expressed
  /// in the same frame. mass adds; COM is the mass-weighted average; the
  /// rotational inertia of the composite about the new COM follows from the
  /// parallel-axis theorem applied to each constituent.
  [[nodiscard]] SpatialInertiaT operator+(const SpatialInertiaT& rhs) const {
    const S m = mass_ + rhs.mass_;
    if (m == S(0)) {
      return SpatialInertiaT();
    }
    const Vector3 c = (mass_ * com_ + rhs.mass_ * rhs.com_) / m;
    // Parallel-axis: shift each Ī_com to the composite COM, then add.
    const Vector3 d_a = com_ - c;
    const Vector3 d_b = rhs.com_ - c;
    const Matrix3 sa = skew(d_a);
    const Matrix3 sb = skew(d_b);
    const Matrix3 i_total = inertia_com_ - mass_ * sa * sa + rhs.inertia_com_ - rhs.mass_ * sb * sb;
    return SpatialInertiaT(m, c, i_total);
  }

 private:
  S mass_;
  Vector3 com_;
  Matrix3 inertia_com_;
};

/// The double-precision spatial inertia used by the public API and algorithms.
using SpatialInertia = SpatialInertiaT<double>;

/// Inertia acting on a motion gives a momentum (a force): `F = I · M`.
///
/// Inline expansion of the angular-first spatial-inertia form
/// `[[I_O, m·c×], [-m·c×, m·I]]`:
///   new_angular = I_O · ω + m · (c × v)
///   new_linear  = m · (v − c × ω)
/// where `I_O = I_com − m·c×·c×` (parallel-axis shift to origin).
/// Saves both the 6×6 matrix construction and the matrix-vector multiply
/// against the dense form (~80 ops vs ~30 ops).
template <typename S>
[[nodiscard]] ForceT<S> operator*(const SpatialInertiaT<S>& i, const MotionT<S>& m) {
  using Vector3 = typename Types<S>::Vector3;
  const Vector3& c = i.com();
  const S mass = i.mass();
  const Vector3 c_cross_v = c.cross(m.linear());
  const Vector3 c_cross_w = c.cross(m.angular());
  const Vector3 new_angular = i.inertia_origin() * m.angular() + mass * c_cross_v;
  const Vector3 new_linear = mass * (m.linear() - c_cross_w);
  return ForceT<S>(new_angular, new_linear);
}

/// SE(3) action on a spatial inertia. With `T = (R, t)` taking body-frame
/// coordinates to parent-frame: the mass is invariant, the COM moves by
/// `c' = R c + t`, and the rotational inertia about COM rotates by
/// `Ī' = R Ī Rᵀ`. Equivalent to the congruence `X⁻ᵀ I X⁻¹` on the 6×6 form,
/// but expressed in the storage parameters to avoid drift.
template <typename S>
[[nodiscard]] SpatialInertiaT<S> operator*(const SE3T<S>& t, const SpatialInertiaT<S>& i) {
  const typename Types<S>::Matrix3 r = t.rotation().matrix();
  return SpatialInertiaT<S>(i.mass(), t.act(i.com()), r * i.inertia_com() * r.transpose());
}

}  // namespace tinyspatial

#endif  // TINYSPATIAL_SPATIAL_INERTIA_HPP
