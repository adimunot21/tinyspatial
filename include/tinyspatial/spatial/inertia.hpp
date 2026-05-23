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
#ifndef TINYSPATIAL_SPATIAL_INERTIA_HPP
#define TINYSPATIAL_SPATIAL_INERTIA_HPP

#include "tinyspatial/core/types.hpp"
#include "tinyspatial/liegroup/se3.hpp"
#include "tinyspatial/spatial/force.hpp"
#include "tinyspatial/spatial/motion.hpp"

namespace tinyspatial {

/// The spatial inertia of a rigid body, expressed at the body-frame origin.
class SpatialInertia {
 public:
  /// Zero mass (the additive identity for the composite-body algorithm).
  SpatialInertia() : mass_(0), com_(Vector3::Zero()), inertia_com_(Matrix3::Zero()) {}

  /// Build from explicit parts. \pre `inertia_com` is symmetric PSD, `mass ≥ 0`.
  SpatialInertia(Scalar mass, const Eigen::Ref<const Vector3>& com,
                 const Eigen::Ref<const Matrix3>& inertia_com)
      : mass_(mass), com_(com), inertia_com_(inertia_com) {}

  /// The zero-mass element.
  [[nodiscard]] static SpatialInertia zero() { return SpatialInertia(); }

  [[nodiscard]] Scalar mass() const { return mass_; }
  [[nodiscard]] const Vector3& com() const { return com_; }
  /// Rotational inertia tensor about the centre of mass.
  [[nodiscard]] const Matrix3& inertia_com() const { return inertia_com_; }

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
    i.topLeftCorner<3, 3>() = inertia_origin();
    i.topRightCorner<3, 3>() = mass_ * cx;
    i.bottomLeftCorner<3, 3>() = mass_ * cx.transpose();
    i.bottomRightCorner<3, 3>() = mass_ * Matrix3::Identity();
    return i;
  }

  /// Addition (composite-body inertia): valid when both inertias are expressed
  /// in the same frame. mass adds; COM is the mass-weighted average; the
  /// rotational inertia of the composite about the new COM follows from the
  /// parallel-axis theorem applied to each constituent.
  [[nodiscard]] SpatialInertia operator+(const SpatialInertia& rhs) const {
    const Scalar m = mass_ + rhs.mass_;
    if (m == Scalar(0)) {
      return SpatialInertia();
    }
    const Vector3 c = (mass_ * com_ + rhs.mass_ * rhs.com_) / m;
    // Parallel-axis: shift each Ī_com to the composite COM, then add.
    const Vector3 d_a = com_ - c;
    const Vector3 d_b = rhs.com_ - c;
    const Matrix3 sa = skew(d_a);
    const Matrix3 sb = skew(d_b);
    const Matrix3 i_total = inertia_com_ - mass_ * sa * sa + rhs.inertia_com_ - rhs.mass_ * sb * sb;
    return SpatialInertia(m, c, i_total);
  }

 private:
  Scalar mass_;
  Vector3 com_;
  Matrix3 inertia_com_;
};

/// Inertia acting on a motion gives a momentum (a force): `F = I · M`.
[[nodiscard]] inline Force operator*(const SpatialInertia& i, const Motion& m) {
  return Force(i.matrix6() * m.vector());
}

/// SE(3) action on a spatial inertia. With `T = (R, t)` taking body-frame
/// coordinates to parent-frame: the mass is invariant, the COM moves by
/// `c' = R c + t`, and the rotational inertia about COM rotates by
/// `Ī' = R Ī Rᵀ`. Equivalent to the congruence `X⁻ᵀ I X⁻¹` on the 6×6 form,
/// but expressed in the storage parameters to avoid drift.
[[nodiscard]] inline SpatialInertia operator*(const SE3& t, const SpatialInertia& i) {
  const Matrix3 r = t.rotation().matrix();
  return SpatialInertia(i.mass(), t.act(i.com()), r * i.inertia_com() * r.transpose());
}

}  // namespace tinyspatial

#endif  // TINYSPATIAL_SPATIAL_INERTIA_HPP
