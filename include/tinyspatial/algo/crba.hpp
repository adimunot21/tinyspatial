/// \file crba.hpp
/// \brief Composite Rigid Body Algorithm — joint-space inertia matrix `M(q)`.
///
/// Given a configuration `q`, fills the upper triangle of `M` such that
///
///   τ = M(q) · q̈ + C(q, q̇) q̇ + g(q)
///
/// (CRBA computes `M` only; the bias term is RNEA at q̈ = 0.) The lower
/// triangle is mirrored from the upper at the end so the returned matrix is
/// symmetric.
///
/// Two phases over a topologically-ordered tree:
///
///   1. **Composite inertia (leaves → root).** For each joint `i` in reverse
///      order, accumulate its composite inertia into its parent's slot:
///          ic[parent[i]] += pose_in_parent[i] · ic[i]
///      i.e. transport `ic[i]` from body i's frame into its parent's frame and
///      add. After the sweep, `ic[i]` is the spatial inertia of the entire
///      subtree rooted at `i`, expressed in body `i`'s frame.
///
///   2. **Fill M (one joint at a time, walk up the parent chain).** For joint
///      `i` with motion subspace `S_i` (6 × nv_i in body i's frame):
///        F = ic[i] · S_i                                (6 × nv_i)
///        M[i, i] = S_iᵀ · F
///      Then walk up `j = parent[i]` while transporting `F` into successive
///      parent frames via the *force* Plücker transform:
///        F = X*(pose_in_parent[k]) · F                 for the joint we stepped through
///        M[j, i] = S_jᵀ · F                            (nv_j × nv_i block)
///      and finally mirror to fill the symmetric lower triangle.
///
/// Featherstone (2008) §6.2. Body-fixed, angular-first.
#ifndef TINYSPATIAL_ALGO_CRBA_HPP
#define TINYSPATIAL_ALGO_CRBA_HPP

#include <type_traits>
#include <variant>
#include <vector>

#include "tinyspatial/algo/forward_kinematics.hpp"
#include "tinyspatial/core/types.hpp"
#include "tinyspatial/model/joint.hpp"
#include "tinyspatial/model/model.hpp"
#include "tinyspatial/spatial/inertia.hpp"
#include "tinyspatial/spatial/plucker.hpp"

namespace tinyspatial {

namespace detail {

/// Joint motion subspace `S_j` as a 6 × nv(j) matrix in the joint's body
/// frame. Angular-first ordering throughout.
[[nodiscard]] inline Eigen::Matrix<Scalar, 6, Eigen::Dynamic> joint_motion_subspace(
    const Joint& j) {
  return std::visit(
      [](const auto& jj) -> Eigen::Matrix<Scalar, 6, Eigen::Dynamic> {
        using JT = std::decay_t<decltype(jj)>;
        if constexpr (std::is_same_v<JT, JointRevolute>) {
          Eigen::Matrix<Scalar, 6, Eigen::Dynamic> s(6, 1);
          s.setZero();
          s.template block<3, 1>(0, 0) = jj.axis;
          return s;
        } else if constexpr (std::is_same_v<JT, JointPrismatic>) {
          Eigen::Matrix<Scalar, 6, Eigen::Dynamic> s(6, 1);
          s.setZero();
          s.template block<3, 1>(3, 0) = jj.axis;
          return s;
        } else if constexpr (std::is_same_v<JT, JointFloating>) {
          Eigen::Matrix<Scalar, 6, Eigen::Dynamic> s(6, 6);
          s.setIdentity();
          return s;
        } else {  // JointFixed
          return Eigen::Matrix<Scalar, 6, Eigen::Dynamic>(6, 0);
        }
      },
      j);
}

}  // namespace detail

/// Composite Rigid Body Algorithm. Writes the full (symmetric) joint-space
/// inertia matrix `M(q)` into `M`.
///
/// \pre `q.size() == model.nq()`. `M` is `model.nv() × model.nv()`.
/// `data` is sized to `model.njoints()`.
inline void crba(const Model& model, Data& data, const Eigen::Ref<const VectorX>& q,
                 Eigen::Ref<MatrixX> m_out) {
  forward_kinematics(model, data, q);

  const int njoints = model.njoints();

  // Phase 1: composite inertias, leaves → root.
  std::vector<SpatialInertia> ic = model.inertia;
  for (int i = njoints - 1; i > 0; --i) {
    const int p = model.parent[i];
    if (p >= 0) {
      ic[p] = ic[p] + (data.pose_in_parent[i] * ic[i]);
    }
  }

  m_out.setZero();

  // Phase 2: for each joint i, compute its diagonal block then walk up the
  // parent chain to fill the cross-coupling blocks.
  for (int i = 0; i < njoints; ++i) {
    const Eigen::Matrix<Scalar, 6, Eigen::Dynamic> s_i =
        detail::joint_motion_subspace(model.joints[i]);
    const int nv_i = static_cast<int>(s_i.cols());
    if (nv_i == 0) {
      continue;  // Fixed joint contributes nothing to M, but still transports.
    }

    // F = ic[i] · S_i  (6 × nv_i), in body i's frame.
    Eigen::Matrix<Scalar, 6, Eigen::Dynamic> f = ic[i].matrix6() * s_i;

    // Diagonal block: M[i, i] = S_iᵀ F.
    m_out.block(model.idx_v[i], model.idx_v[i], nv_i, nv_i) = s_i.transpose() * f;

    // Walk up the parent chain, transporting F into each ancestor's frame
    // via the force Plücker transform of the joint we just stepped through.
    int child = i;
    int j = model.parent[i];
    while (j >= 0) {
      // F now lives in `child`'s frame; transport into `j`'s frame.
      f = force_plucker(data.pose_in_parent[child]) * f;
      const Eigen::Matrix<Scalar, 6, Eigen::Dynamic> s_j =
          detail::joint_motion_subspace(model.joints[j]);
      const int nv_j = static_cast<int>(s_j.cols());
      if (nv_j > 0) {
        const Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> block = s_j.transpose() * f;
        // Pinocchio stores M with rows = parent index of the pair (lower of
        // the two), cols = the joint we started from. We follow the convention
        // M[ancestor, descendant] = S_ancestorᵀ · F (transported), and mirror
        // to fill the symmetric counterpart.
        m_out.block(model.idx_v[j], model.idx_v[i], nv_j, nv_i) = block;
        m_out.block(model.idx_v[i], model.idx_v[j], nv_i, nv_j) = block.transpose();
      }
      child = j;
      j = model.parent[j];
    }
  }
}

}  // namespace tinyspatial

#endif  // TINYSPATIAL_ALGO_CRBA_HPP
