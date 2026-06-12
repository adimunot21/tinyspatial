/// \file joint.hpp
/// \brief Joint variants: revolute, prismatic, fixed, floating.
///
/// Each joint is a small value-type holding only its geometric data (e.g. an
/// axis). The four variants are unified by `std::variant`; algorithms dispatch
/// via `std::visit` at compile time, so there's no virtual-call overhead in
/// the per-joint loop. The configuration `q` and velocity `v` of a joint are
/// stored in flat vectors in `Model::idx_q` / `idx_v`; this file is just the
/// per-joint contract.
///
/// Each variant is templated on the scalar (`JointRevoluteT<S>`, …) with the
/// concrete `double` types aliased (`JointRevolute`, `Joint`, …). Instantiating
/// on an autodiff `Jet` makes `transform()` differentiable w.r.t. `q`; the
/// joint geometry (axes) is constant data lifted with `cast<S>()`.
#ifndef TINYSPATIAL_MODEL_JOINT_HPP
#define TINYSPATIAL_MODEL_JOINT_HPP

#include <type_traits>
#include <variant>

#include "tinyspatial/core/types.hpp"
#include "tinyspatial/liegroup/se3.hpp"
#include "tinyspatial/liegroup/so3.hpp"

namespace tinyspatial {

/// A 1-DOF revolute (hinge) joint: rotates by `q` radians about `axis`.
/// `axis` is expressed in the joint's local frame and is assumed unit-norm.
template <typename S>
struct JointRevoluteT {
  using Vector3 = typename Types<S>::Vector3;
  Vector3 axis;

  [[nodiscard]] int nq() const { return 1; }
  [[nodiscard]] int nv() const { return 1; }
  [[nodiscard]] SE3T<S> transform(const S& q) const {
    return SE3T<S>(SO3T<S>::exp(axis * q), Vector3::Zero());
  }
  template <typename S2>
  [[nodiscard]] JointRevoluteT<S2> cast() const {
    return JointRevoluteT<S2>{axis.template cast<S2>()};
  }
};

/// A 1-DOF prismatic (slider) joint: translates by `q` along `axis`.
template <typename S>
struct JointPrismaticT {
  using Vector3 = typename Types<S>::Vector3;
  Vector3 axis;

  [[nodiscard]] int nq() const { return 1; }
  [[nodiscard]] int nv() const { return 1; }
  [[nodiscard]] SE3T<S> transform(const S& q) const {
    return SE3T<S>(SO3T<S>::identity(), axis * q);
  }
  template <typename S2>
  [[nodiscard]] JointPrismaticT<S2> cast() const {
    return JointPrismaticT<S2>{axis.template cast<S2>()};
  }
};

/// A 0-DOF rigid attachment. Used for welded URDF links.
template <typename S>
struct JointFixedT {
  [[nodiscard]] int nq() const { return 0; }
  [[nodiscard]] int nv() const { return 0; }
  [[nodiscard]] SE3T<S> transform() const { return SE3T<S>::identity(); }
  template <typename S2>
  [[nodiscard]] JointFixedT<S2> cast() const {
    return JointFixedT<S2>{};
  }
};

/// A 6-DOF free joint (e.g. the base of a humanoid). Configuration layout
/// follows Pinocchio: `q = (t_x, t_y, t_z, q_x, q_y, q_z, q_w)` — translation
/// first, then unit quaternion. Velocity is a standard angular-first twist.
template <typename S>
struct JointFloatingT {
  using Quaternion = typename Types<S>::Quaternion;

  [[nodiscard]] int nq() const { return 7; }
  [[nodiscard]] int nv() const { return 6; }
  [[nodiscard]] SE3T<S> transform(const Eigen::Ref<const Eigen::Matrix<S, 7, 1>>& q) const {
    // Eigen quaternion constructor takes (w, x, y, z); the SO3 ctor normalises.
    const Quaternion quat(q(6), q(3), q(4), q(5));
    return SE3T<S>(SO3T<S>(quat), q.template head<3>());
  }
  template <typename S2>
  [[nodiscard]] JointFloatingT<S2> cast() const {
    return JointFloatingT<S2>{};
  }
};

/// Tagged union over the four joint types, templated on the scalar.
template <typename S>
using JointT =
    std::variant<JointFixedT<S>, JointRevoluteT<S>, JointPrismaticT<S>, JointFloatingT<S>>;

// The double-precision joint types used by the public API, URDF loader, and
// algorithms.
using JointFixed = JointFixedT<double>;
using JointRevolute = JointRevoluteT<double>;
using JointPrismatic = JointPrismaticT<double>;
using JointFloating = JointFloatingT<double>;
using Joint = JointT<double>;

/// Number of configuration coordinates for a joint.
template <typename S>
[[nodiscard]] int nq(const JointT<S>& j) {
  return std::visit([](const auto& x) { return x.nq(); }, j);
}

/// Number of velocity / tangent coordinates for a joint.
template <typename S>
[[nodiscard]] int nv(const JointT<S>& j) {
  return std::visit([](const auto& x) { return x.nv(); }, j);
}

/// Lift a double-precision joint to another scalar (zero-derivative constant).
template <typename S2>
[[nodiscard]] JointT<S2> joint_cast(const Joint& j) {
  return std::visit([](const auto& x) -> JointT<S2> { return x.template cast<S2>(); }, j);
}

/// Joint motion subspace `S_j` as a 6 × nv(j) matrix in the joint's body
/// frame (angular-first ordering). For revolute joints column 0 is
/// `(axis; 0)`; for prismatic, `(0; axis)`; for fixed, an empty 6×0 matrix;
/// for floating, the 6×6 identity. Pure function of the joint variant —
/// can be cached at `Model::add_joint` time so the hot path (CRBA, RNEA
/// inward pass) avoids both the `std::visit` and the per-call heap
/// allocation for the dynamic-sized matrix.
template <typename S>
[[nodiscard]] typename Types<S>::Matrix6X joint_motion_subspace(const JointT<S>& j) {
  using Matrix6X = typename Types<S>::Matrix6X;
  return std::visit(
      [](const auto& jj) -> Matrix6X {
        using JT = std::decay_t<decltype(jj)>;
        if constexpr (std::is_same_v<JT, JointRevoluteT<S>>) {
          Matrix6X s(6, 1);
          s.setZero();
          s.template block<3, 1>(0, 0) = jj.axis;
          return s;
        } else if constexpr (std::is_same_v<JT, JointPrismaticT<S>>) {
          Matrix6X s(6, 1);
          s.setZero();
          s.template block<3, 1>(3, 0) = jj.axis;
          return s;
        } else if constexpr (std::is_same_v<JT, JointFloatingT<S>>) {
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
template <typename S>
[[nodiscard]] SE3T<S> joint_transform(const JointT<S>& j,
                                      const Eigen::Ref<const typename Types<S>::VectorX>& q_slice) {
  return std::visit(
      [&](const auto& x) -> SE3T<S> {
        using T = std::decay_t<decltype(x)>;
        if constexpr (std::is_same_v<T, JointFixedT<S>>) {
          return x.transform();
        } else if constexpr (std::is_same_v<T, JointFloatingT<S>>) {
          // head<7>() returns an Eigen expression; convert to a fixed-size buffer.
          Eigen::Matrix<S, 7, 1> q7;
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
