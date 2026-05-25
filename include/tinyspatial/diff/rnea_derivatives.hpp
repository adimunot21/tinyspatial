/// \file rnea_derivatives.hpp
/// \brief Analytical partial derivatives of RNEA: `∂τ/∂q`, `∂τ/∂v`, `∂τ/∂a`.
///
/// Implements the body-fixed, angular-first version of the recursive
/// derivative algorithm in Carpentier & Mansard, *Analytical Derivatives of
/// Rigid Body Dynamics Algorithms* (RSS 2018, §IV).
///
/// Two key identities make this O(N²) rather than O(N³):
///
///   1. **Cross-motion antisymmetry**: `cross_motion(x) · y = -cross_motion(y) · x`
///      (proven in chapter 05). This turns the per-column work
///      `cross_motion(dv_dq[i].col(k)) · (S_i v_slice_i)` into a single
///      matrix product `-cross_motion(S_i v_slice_i) · dv_dq[i]`.
///
///   2. **Force-cross-motion duality**: for fixed force `f`,
///      `cross_force(m) · f` is linear in `m` and equals
///      `Phi(f) · m`, with
///      `Phi(f) = [[-[τ]_×, -[v_f]_×]; [-[v_f]_×, 0]]`. We compute
///      `Phi(I_i v[i])` once per joint and apply it to `dv_d*[i]` as a
///      matrix product.
///
/// The single-DOF derivative formulas (revolute / prismatic / fixed; the
/// fixture URDFs use only these) are:
///
///   ∂v[i]/∂q.col(k) = if k in joint i's slice: -[S_col]_× · v_parent_in_i
///                     elif k ancestor:          X_{pi} · ∂v[p]/∂q.col(k)
///                     else:                      0
///
///   ∂v[i]/∂v.col(k) = if k in joint i's slice:  S_col
///                     elif k ancestor:           X_{pi} · ∂v[p]/∂v.col(k)
///                     else:                      0
///
///   ∂a[i]/∂q, ∂a[i]/∂v: same structure with the extra
///                       `cross_motion(∂v[i]/∂*.col(k)) · (S_i v_slice_i)`
///                       term plus, for own-slice columns, the gravity-
///                       coupling `-[S_col]_× · a_parent_in_i`.
///
/// The backward pass mirrors RNEA's inward pass, with one extra term at
/// joint i's own columns: `Y_{ip} · cross_force(S_col) · data.f[i]`,
/// which is the derivative of the dual-Plücker transport w.r.t. the
/// joint's own configuration.
///
/// **∂τ/∂a ≡ M(q)** as a corollary (since `∂a[i]/∂a = J_i` and `v[i]` is
/// `a`-independent). The unit tests check this directly against CRBA.
#ifndef TINYSPATIAL_DIFF_RNEA_DERIVATIVES_HPP
#define TINYSPATIAL_DIFF_RNEA_DERIVATIVES_HPP

#include <type_traits>
#include <variant>
#include <vector>

#include "tinyspatial/algo/forward_kinematics.hpp"
#include "tinyspatial/algo/rnea.hpp"  // detail::joint_subspace_motion, joint_subspace_project
#include "tinyspatial/core/types.hpp"
#include "tinyspatial/model/joint.hpp"
#include "tinyspatial/model/model.hpp"
#include "tinyspatial/spatial/cross.hpp"
#include "tinyspatial/spatial/force.hpp"
#include "tinyspatial/spatial/inertia.hpp"
#include "tinyspatial/spatial/motion.hpp"
#include "tinyspatial/spatial/plucker.hpp"

namespace tinyspatial {

namespace detail {

/// Motion subspace `S_i` of a joint as a 6 × nv(j) matrix. Same as CRBA's
/// `joint_motion_subspace` but duplicated here to avoid a `crba.hpp` include
/// pulling in unrelated machinery.
[[nodiscard]] inline Eigen::Matrix<Scalar, 6, Eigen::Dynamic> rnea_deriv_joint_subspace(
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

/// `Phi(f)` such that `Phi(f) · m == cross_force(m) · f` for any motion m.
/// For `f = (τ; v_f)`: `Phi = [[-[τ]_×, -[v_f]_×]; [-[v_f]_×, 0]]`.
[[nodiscard]] inline Matrix6 force_cross_motion_phi(const Eigen::Ref<const Vector6>& f) {
  const Matrix3 tau_x = skew(f.head<3>());
  const Matrix3 vf_x = skew(f.tail<3>());
  Matrix6 phi = Matrix6::Zero();
  phi.topLeftCorner<3, 3>() = -tau_x;
  phi.topRightCorner<3, 3>() = -vf_x;
  phi.bottomLeftCorner<3, 3>() = -vf_x;
  return phi;
}

}  // namespace detail

/// Analytical partials of RNEA. Writes `∂τ/∂q`, `∂τ/∂v`, `∂τ/∂a` as
/// `nv × nv` matrices.
///
/// \pre `q.size() == model.nq()`, `v.size() == a.size() == model.nv()`.
/// `dtau_dq`, `dtau_dv`, `dtau_da` are `model.nv() × model.nv()`.
/// `data` is sized to `model.njoints()`.
/// `gravity` is in the world frame; default `(0, 0, −9.81)`.
inline void rnea_derivatives(const Model& model, Data& data, const Eigen::Ref<const VectorX>& q,
                             const Eigen::Ref<const VectorX>& v, const Eigen::Ref<const VectorX>& a,
                             Eigen::Ref<MatrixX> dtau_dq, Eigen::Ref<MatrixX> dtau_dv,
                             Eigen::Ref<MatrixX> dtau_da,
                             const Vector3& gravity = Vector3(0, 0, -9.81)) {
  forward_kinematics(model, data, q);

  const int njoints = model.njoints();
  const int nv_t = model.nv();

  // Per-body 6×nv derivative matrices. dv_dv[i] is the per-joint Jacobian
  // J_i; we also use it as da_da[i] later (∂a[i]/∂a = J_i).
  std::vector<Matrix6X> dv_dq(njoints, Matrix6X::Zero(6, nv_t));
  std::vector<Matrix6X> dv_dv(njoints, Matrix6X::Zero(6, nv_t));
  std::vector<Matrix6X> da_dq(njoints, Matrix6X::Zero(6, nv_t));
  std::vector<Matrix6X> da_dv(njoints, Matrix6X::Zero(6, nv_t));

  // ---- Forward pass: standard RNEA outward + per-body derivative recurrences.
  const Motion neg_gravity_world(Vector3::Zero(), -gravity);

  for (int i = 0; i < njoints; ++i) {
    const Joint& j = model.joints[i];
    const int nv_i = nv(j);
    const int idx_v_i = model.idx_v[i];
    const Eigen::Matrix<Scalar, 6, Eigen::Dynamic> s_i = detail::rnea_deriv_joint_subspace(j);
    const SE3 i_from_parent = data.pose_in_parent[i].inverse();
    const Matrix6 x_pi = i_from_parent.adjoint();  // motion adjoint, parent → body i.

    // Joint contribution motions.
    const Motion v_j = detail::joint_subspace_motion(j, v.segment(idx_v_i, nv_i));
    const Motion a_j = detail::joint_subspace_motion(j, a.segment(idx_v_i, nv_i));

    // v_parent_in_i and a_parent_in_i (the second carries gravity for the root).
    Motion v_parent_in_i;
    Motion a_parent_in_i;
    if (model.parent[i] == -1) {
      v_parent_in_i = Motion::zero();
      a_parent_in_i = i_from_parent * neg_gravity_world;
    } else {
      v_parent_in_i = i_from_parent * data.v[model.parent[i]];
      a_parent_in_i = i_from_parent * data.a[model.parent[i]];

      // Inherit derivative columns from the parent (X_pi · parent's matrices).
      // For columns in joint i's own slice the parent has zeros, so the
      // inheritance leaves them zero; the special formulas below overwrite.
      dv_dq[i].noalias() = x_pi * dv_dq[model.parent[i]];
      dv_dv[i].noalias() = x_pi * dv_dv[model.parent[i]];
      da_dq[i].noalias() = x_pi * da_dq[model.parent[i]];
      da_dv[i].noalias() = x_pi * da_dv[model.parent[i]];
    }

    // Standard outward kinematics.
    data.v[i] = v_parent_in_i + v_j;
    data.a[i] = a_parent_in_i + a_j + cross(data.v[i], v_j);

    // Net body wrench (same as RNEA): I·a + v ×* (I·v).
    const Force inertia_a = model.inertia[i] * data.a[i];
    const Force inertia_v = model.inertia[i] * data.v[i];
    data.f[i] = inertia_a + cross(data.v[i], inertia_v);

    // Joint i's own slice: overwrite the special derivative formulas.
    const Vector6 v_par_vec = v_parent_in_i.vector();
    const Vector6 a_par_vec = a_parent_in_i.vector();
    for (int k_local = 0; k_local < nv_i; ++k_local) {
      const int col = idx_v_i + k_local;
      const Vector6 s_col = s_i.col(k_local);
      const Matrix6 ad_s = cross_motion(s_col);

      // ∂v[i]/∂q.col = -[S_col]_× · v_parent_in_i.
      dv_dq[i].col(col) = -ad_s * v_par_vec;
      // ∂v[i]/∂v.col = S_col.
      dv_dv[i].col(col) = s_col;
      // ∂a[i]/∂q.col = -[S_col]_× · a_parent_in_i  (the rest comes from the
      // universal cross-coupling subtraction below).
      da_dq[i].col(col) = -ad_s * a_par_vec;
      // ∂a[i]/∂v.col = cross_motion(v[i]) · S_col + (universal term added below).
      da_dv[i].col(col) = cross_motion(data.v[i].vector()) * s_col;
    }

    // Universal coupling term: cross_motion(dv_d*.col(k)) · (S_i v_slice_i)
    //   = -cross_motion(S_i v_slice_i) · dv_d*.col(k)    (antisymmetry).
    // For joint i's own slice this completes the formula; for ancestor cols
    // this *is* the formula (after the X_pi inheritance, which already added
    // the X_pi · ∂a[p]/∂* term).
    const Matrix6 m_sv = cross_motion(v_j.vector());
    da_dq[i].noalias() -= m_sv * dv_dq[i];
    da_dv[i].noalias() -= m_sv * dv_dv[i];
  }

  // ---- Backward pass: per-body wrench derivatives + projection.
  std::vector<Matrix6X> df_dq(njoints, Matrix6X::Zero(6, nv_t));
  std::vector<Matrix6X> df_dv(njoints, Matrix6X::Zero(6, nv_t));
  std::vector<Matrix6X> df_da(njoints, Matrix6X::Zero(6, nv_t));

  // Initialize df_d* from the body's own f = I·a + v ×* (I·v):
  //   df_dq = I·da_dq + Phi(I·v)·dv_dq + cross_force(v)·I·dv_dq
  //   df_dv = I·da_dv + Phi(I·v)·dv_dv + cross_force(v)·I·dv_dv
  //   df_da = I·dv_dv                       (∂a/∂a = J = dv_dv; v ⊥ a)
  for (int i = 0; i < njoints; ++i) {
    const Matrix6 i_mat = model.inertia[i].matrix6();
    const Vector6 iv = i_mat * data.v[i].vector();
    const Matrix6 phi_iv = detail::force_cross_motion_phi(iv);
    const Matrix6 cf_v = cross_force(data.v[i].vector());
    const Matrix6 vel_coupling = phi_iv + cf_v * i_mat;
    df_dq[i].noalias() = i_mat * da_dq[i] + vel_coupling * dv_dq[i];
    df_dv[i].noalias() = i_mat * da_dv[i] + vel_coupling * dv_dv[i];
    df_da[i].noalias() = i_mat * dv_dv[i];
  }

  dtau_dq.setZero();
  dtau_dv.setZero();
  dtau_da.setZero();

  for (int i = njoints - 1; i >= 0; --i) {
    const Joint& j = model.joints[i];
    const int nv_i = nv(j);
    const int idx_v_i = model.idx_v[i];
    const Eigen::Matrix<Scalar, 6, Eigen::Dynamic> s_i = detail::rnea_deriv_joint_subspace(j);

    if (nv_i > 0) {
      // Project tau-slice derivatives.
      dtau_dq.middleRows(idx_v_i, nv_i).noalias() = s_i.transpose() * df_dq[i];
      dtau_dv.middleRows(idx_v_i, nv_i).noalias() = s_i.transpose() * df_dv[i];
      dtau_da.middleRows(idx_v_i, nv_i).noalias() = s_i.transpose() * df_da[i];
    }

    if (model.parent[i] != -1) {
      const Matrix6 y_ip = force_plucker(data.pose_in_parent[i]);
      // Standard wrench transport (mirrors RNEA inward).
      data.f[model.parent[i]] += data.pose_in_parent[i] * data.f[i];

      // Inherited derivative columns: F[parent] += Y_ip · F[i].
      df_dq[model.parent[i]].noalias() += y_ip * df_dq[i];
      df_dv[model.parent[i]].noalias() += y_ip * df_dv[i];
      df_da[model.parent[i]].noalias() += y_ip * df_da[i];

      // Extra ∂Y_ip/∂q.col(k) · F[i] term at joint i's own columns.
      // ∂Y_ip/∂q_slice_i_k = Y_ip · cross_force(S_i.col(k_local)).
      for (int k_local = 0; k_local < nv_i; ++k_local) {
        const Vector6 s_col = s_i.col(k_local);
        const int col = idx_v_i + k_local;
        df_dq[model.parent[i]].col(col).noalias() +=
            y_ip * (cross_force(s_col) * data.f[i].vector());
      }
    }
  }
}

}  // namespace tinyspatial

#endif  // TINYSPATIAL_DIFF_RNEA_DERIVATIVES_HPP
