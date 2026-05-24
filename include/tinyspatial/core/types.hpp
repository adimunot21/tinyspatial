/// \file types.hpp
/// \brief Numeric type aliases used throughout tinyspatial.
///
/// We commit to a concrete scalar type (`double`) rather than templating on
/// Scalar. This keeps the headers readable for the course audience and keeps
/// compile times down; templating for autodiff/codegen is a deliberate future
/// refactor (see PROJECT_PLAN Phases 6 and 13).
#ifndef TINYSPATIAL_CORE_TYPES_HPP
#define TINYSPATIAL_CORE_TYPES_HPP

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace tinyspatial {

/// The one scalar type for the whole library.
using Scalar = double;

// Fixed-size algebra. Prefer these wherever dimensions are known at compile
// time (CLAUDE.md §7); dynamic sizes appear only at the kinematic-tree boundary.
using Vector3 = Eigen::Matrix<Scalar, 3, 1>;
using Matrix3 = Eigen::Matrix<Scalar, 3, 3>;

/// 4×4 homogeneous transform matrix (SE(3) in homogeneous coordinates).
using Matrix4 = Eigen::Matrix<Scalar, 4, 4>;

/// Spatial 6-vector. ANGULAR-FIRST: indices 0..2 are angular (ω), 3..5 are
/// linear (v). This follows Featherstone (2008) and diverges from Pinocchio's
/// linear-first ordering — the Pinocchio validation suite converts at the
/// boundary (CLAUDE.md §5, §11, §15).
using Vector6 = Eigen::Matrix<Scalar, 6, 1>;
using Matrix6 = Eigen::Matrix<Scalar, 6, 6>;

/// Hamilton quaternion; tinyspatial normalises rotations to `w ≥ 0` (§15).
using Quaternion = Eigen::Quaternion<Scalar>;

// Dynamic sizes — used at the kinematic-tree boundary (configuration vectors,
// Jacobians) from Phase 3 onward.
using VectorX = Eigen::Matrix<Scalar, Eigen::Dynamic, 1>;
using MatrixX = Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic>;
/// Spatial Jacobian: 6 rows (angular-first twist), `model.nv()` columns.
using Matrix6X = Eigen::Matrix<Scalar, 6, Eigen::Dynamic>;

}  // namespace tinyspatial

#endif  // TINYSPATIAL_CORE_TYPES_HPP
