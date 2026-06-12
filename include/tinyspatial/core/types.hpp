/// \file types.hpp
/// \brief Numeric type aliases used throughout tinyspatial.
///
/// The library is written against a single scalar type, `double`, exposed
/// through the concrete aliases below (`Vector3`, `Matrix6`, …). Those aliases
/// are what every header uses, and they are unchanged.
///
/// Underneath, the same algebra is also available templated on the scalar via
/// `Types<S>`. This is the seam that lets the algorithms run on an autodiff
/// dual type (`core/jet.hpp`) without a second implementation: `double` is just
/// `Types<double>`. Library code keeps naming the concrete aliases for
/// readability (CLAUDE.md §7); the templated layer is opt-in.
#ifndef TINYSPATIAL_CORE_TYPES_HPP
#define TINYSPATIAL_CORE_TYPES_HPP

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace tinyspatial {

/// The whole fixed/dynamic algebra, parameterised on the scalar type `S`.
///
/// Spatial 6-vectors are ANGULAR-FIRST: indices 0..2 are angular (ω), 3..5 are
/// linear (v). This follows Featherstone (2008) and diverges from Pinocchio's
/// linear-first ordering — the validation suite converts at the boundary
/// (CLAUDE.md §5, §11, §15).
template <typename S>
struct Types {
  using Scalar = S;

  using Vector3 = Eigen::Matrix<S, 3, 1>;
  using Matrix3 = Eigen::Matrix<S, 3, 3>;
  /// 4×4 homogeneous transform matrix (SE(3) in homogeneous coordinates).
  using Matrix4 = Eigen::Matrix<S, 4, 4>;
  using Vector6 = Eigen::Matrix<S, 6, 1>;
  using Matrix6 = Eigen::Matrix<S, 6, 6>;
  /// Hamilton quaternion; tinyspatial normalises rotations to `w ≥ 0` (§15).
  using Quaternion = Eigen::Quaternion<S>;

  // Dynamic sizes — used only at the kinematic-tree boundary (configuration
  // vectors, Jacobians).
  using VectorX = Eigen::Matrix<S, Eigen::Dynamic, 1>;
  using MatrixX = Eigen::Matrix<S, Eigen::Dynamic, Eigen::Dynamic>;
  /// Spatial Jacobian: 6 rows (angular-first twist), `model.nv()` columns.
  using Matrix6X = Eigen::Matrix<S, 6, Eigen::Dynamic>;
};

// --- The concrete double-precision algebra the library is written against. ---
// These are exactly the types used before the templated layer existed; every
// header names these. `Types<double>::Vector3` is identical to the old
// `Eigen::Matrix<Scalar, 3, 1>`, so nothing downstream changes.

/// The one scalar type the library's public API is written against.
using Scalar = double;

using Vector3 = Types<Scalar>::Vector3;
using Matrix3 = Types<Scalar>::Matrix3;
using Matrix4 = Types<Scalar>::Matrix4;
using Vector6 = Types<Scalar>::Vector6;
using Matrix6 = Types<Scalar>::Matrix6;
using Quaternion = Types<Scalar>::Quaternion;
using VectorX = Types<Scalar>::VectorX;
using MatrixX = Types<Scalar>::MatrixX;
using Matrix6X = Types<Scalar>::Matrix6X;

}  // namespace tinyspatial

#endif  // TINYSPATIAL_CORE_TYPES_HPP
