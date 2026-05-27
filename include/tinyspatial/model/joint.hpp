/// \file joint.hpp
/// \brief Joint variants: revolute, prismatic, fixed, floating.
///
/// Each joint is a small value-type holding only its geometric data (e.g. an
/// axis). The four variants are unified by `std::variant`; algorithms dispatch
/// via `std::visit` at compile time, so there's no virtual-call overhead in
/// the per-joint loop. The configuration `q` and velocity `v` of a joint are
/// stored in flat vectors in `Model::idx_q` / `idx_v`; this file is just the
/// per-joint contract.
#ifndef TINYSPATIAL_MODEL_JOINT_HPP
#define TINYSPATIAL_MODEL_JOINT_HPP

#include <variant>

#include "tinyspatial/core/types.hpp"
#include "tinyspatial/liegroup/se3.hpp"
#include "tinyspatial/liegroup/so3.hpp"

namespace tinyspatial {

/// A 1-DOF revolute (hinge) joint: rotates by `q` radians about `axis`.
/// `axis` is expressed in the joint's local frame and is assumed unit-norm.
struct JointRevolute {
  Vector3 axis;

  [[nodiscard]] int nq() const { return 1; }
  [[nodiscard]] int nv() const { return 1; }
  [[nodiscard]] SE3 transform(Scalar q) const { return SE3(SO3::exp(axis * q), Vector3::Zero()); }
};

/// A 1-DOF prismatic (slider) joint: translates by `q` along `axis`.
struct JointPrismatic {
  Vector3 axis;

  [[nodiscard]] int nq() const { return 1; }
  [[nodiscard]] int nv() const { return 1; }
  [[nodiscard]] SE3 transform(Scalar q) const { return SE3(SO3::identity(), axis * q); }
};

/// A 0-DOF rigid attachment. Used for welded URDF links.
struct JointFixed {
  [[nodiscard]] int nq() const { return 0; }
  [[nodiscard]] int nv() const { return 0; }
  [[nodiscard]] SE3 transform() const { return SE3::identity(); }
};

/// A 6-DOF free joint (e.g. the base of a humanoid). Configuration layout
/// follows Pinocchio: `q = (t_x, t_y, t_z, q_x, q_y, q_z, q_w)` — translation
/// first, then unit quaternion. Velocity is a standard angular-first twist.
struct JointFloating {
  [[nodiscard]] int nq() const { return 7; }
  [[nodiscard]] int nv() const { return 6; }
  [[nodiscard]] SE3 transform(const Eigen::Ref<const Eigen::Matrix<Scalar, 7, 1>>& q) const {
    // Eigen quaternion constructor takes (w, x, y, z); the SO3 ctor normalises.
    const Quaternion quat(q(6), q(3), q(4), q(5));
    return SE3(SO3(quat), q.head<3>());
  }
};

/// Tagged union over the four joint types.
using Joint = std::variant<JointFixed, JointRevolute, JointPrismatic, JointFloating>;

/// Number of configuration coordinates for a joint.
[[nodiscard]] inline int nq(const Joint& j) {
  return std::visit([](const auto& x) { return x.nq(); }, j);
}

/// Number of velocity / tangent coordinates for a joint.
[[nodiscard]] inline int nv(const Joint& j) {
  return std::visit([](const auto& x) { return x.nv(); }, j);
}

/// Joint motion subspace `S_j` as a 6 × nv(j) matrix in the joint's body
/// frame (angular-first ordering). For revolute joints column 0 is
/// `(axis; 0)`; for prismatic, `(0; axis)`; for fixed, an empty 6×0 matrix;
/// for floating, the 6×6 identity. Pure function of the joint variant —
/// can be cached at `Model::add_joint` time so the hot path (CRBA, RNEA
/// inward pass) avoids both the `std::visit` and the per-call heap
/// allocation for the dynamic-sized matrix.
[[nodiscard]] inline Matrix6X joint_motion_subspace(const Joint& j) {
  return std::visit(
      [](const auto& jj) -> Matrix6X {
        using JT = std::decay_t<decltype(jj)>;
        if constexpr (std::is_same_v<JT, JointRevolute>) {
          Matrix6X s(6, 1);
          s.setZero();
          s.template block<3, 1>(0, 0) = jj.axis;
          return s;
        } else if constexpr (std::is_same_v<JT, JointPrismatic>) {
          Matrix6X s(6, 1);
          s.setZero();
          s.template block<3, 1>(3, 0) = jj.axis;
          return s;
        } else if constexpr (std::is_same_v<JT, JointFloating>) {
          Matrix6X s(6, 6);
          s.setIdentity();
          return s;
        } else {  // JointFixed
          return Matrix6X(6, 0);
        }
      },
      j);
}

/// Joint's local SE(3) transform, given the slice of `q` belonging to this joint.
/// `q_slice` must have `nq(j)` rows.
[[nodiscard]] inline SE3 joint_transform(const Joint& j, const Eigen::Ref<const VectorX>& q_slice) {
  return std::visit(
      [&](const auto& x) -> SE3 {
        using T = std::decay_t<decltype(x)>;
        if constexpr (std::is_same_v<T, JointFixed>) {
          return x.transform();
        } else if constexpr (std::is_same_v<T, JointFloating>) {
          // head<7>() returns an Eigen expression; convert to a fixed-size buffer.
          Eigen::Matrix<Scalar, 7, 1> q7;
          for (int i = 0; i < 7; ++i) {
            q7(i) = q_slice(i);
          }
          return x.transform(q7);
        } else {
          return x.transform(q_slice(0));
        }
      },
      j);
}

}  // namespace tinyspatial

#endif  // TINYSPATIAL_MODEL_JOINT_HPP
