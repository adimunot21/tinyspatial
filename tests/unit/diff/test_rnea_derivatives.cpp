#include <filesystem>
#include <random>
#include <string>

#include "tinyspatial/algo/crba.hpp"
#include "tinyspatial/algo/rnea.hpp"
#include "tinyspatial/diff/rnea_derivatives.hpp"
#include "tinyspatial/urdf/urdf_loader.hpp"

#include <gtest/gtest.h>

namespace tinyspatial {
namespace {

constexpr const char* kFixtureDir = TINYSPATIAL_TEST_FIXTURE_DIR;

std::string fixture(const std::string& name) {
  return (std::filesystem::path(kFixtureDir) / name).string();
}

/// Build a column-FD reference for ∂τ/∂x via central difference at step eps.
MatrixX fd_column_partials(const Model& m, const VectorX& q, const VectorX& v, const VectorX& a,
                           const Vector3& gravity, int which_input, Scalar eps) {
  // which_input: 0 = perturb q, 1 = perturb v, 2 = perturb a.
  const int n = m.nv();
  MatrixX out(n, n);
  Data d_plus(m);
  Data d_minus(m);
  VectorX tau_plus(n);
  VectorX tau_minus(n);

  for (int k = 0; k < n; ++k) {
    VectorX q_p = q;
    VectorX v_p = v;
    VectorX a_p = a;
    VectorX q_m = q;
    VectorX v_m = v;
    VectorX a_m = a;
    if (which_input == 0) {
      q_p(k) += eps;
      q_m(k) -= eps;
    } else if (which_input == 1) {
      v_p(k) += eps;
      v_m(k) -= eps;
    } else {
      a_p(k) += eps;
      a_m(k) -= eps;
    }
    rnea(m, d_plus, q_p, v_p, a_p, tau_plus, gravity);
    rnea(m, d_minus, q_m, v_m, a_m, tau_minus, gravity);
    out.col(k) = (tau_plus - tau_minus) / (2.0 * eps);
  }
  return out;
}

class RneaDerivativesTest : public ::testing::TestWithParam<std::string> {};

TEST_P(RneaDerivativesTest, MatchesCentralFiniteDifference) {
  // For random (q, v, a), analytical partials must agree with central FD to
  // ~1e-6 (FD precision floor at eps=1e-6).
  const Model m = build_model_from_urdf_file(fixture(GetParam()));
  Data d(m);
  const Vector3 gravity(0, 0, -9.81);
  std::mt19937 gen(7919);
  std::uniform_real_distribution<Scalar> uni(-1.0, 1.0);

  for (int trial = 0; trial < 3; ++trial) {
    VectorX q(m.nq());
    for (int i = 0; i < m.nq(); ++i) {
      q(i) = uni(gen);
    }
    VectorX vv(m.nv());
    VectorX aa(m.nv());
    for (int i = 0; i < m.nv(); ++i) {
      vv(i) = uni(gen);
      aa(i) = uni(gen);
    }

    MatrixX dtau_dq(m.nv(), m.nv());
    MatrixX dtau_dv(m.nv(), m.nv());
    MatrixX dtau_da(m.nv(), m.nv());
    rnea_derivatives(m, d, q, vv, aa, dtau_dq, dtau_dv, dtau_da, gravity);

    constexpr Scalar kEps = 1e-6;
    const MatrixX fd_q = fd_column_partials(m, q, vv, aa, gravity, 0, kEps);
    const MatrixX fd_v = fd_column_partials(m, q, vv, aa, gravity, 1, kEps);
    const MatrixX fd_a = fd_column_partials(m, q, vv, aa, gravity, 2, kEps);

    EXPECT_LE((dtau_dq - fd_q).cwiseAbs().maxCoeff(), 1e-6) << "dtau/dq mismatch vs FD";
    EXPECT_LE((dtau_dv - fd_v).cwiseAbs().maxCoeff(), 1e-6) << "dtau/dv mismatch vs FD";
    EXPECT_LE((dtau_da - fd_a).cwiseAbs().maxCoeff(), 1e-6) << "dtau/da mismatch vs FD";
  }
}

TEST_P(RneaDerivativesTest, DtauDaEqualsCrbaMassMatrix) {
  // ∂τ/∂a ≡ M(q) is a structural identity. Check to machine precision.
  const Model m = build_model_from_urdf_file(fixture(GetParam()));
  Data d(m);
  std::mt19937 gen(11);
  std::uniform_real_distribution<Scalar> uni(-1.2, 1.2);
  VectorX q(m.nq());
  for (int i = 0; i < m.nq(); ++i) {
    q(i) = uni(gen);
  }
  VectorX vv(m.nv());
  VectorX aa(m.nv());
  for (int i = 0; i < m.nv(); ++i) {
    vv(i) = uni(gen);
    aa(i) = uni(gen);
  }

  MatrixX dtau_dq(m.nv(), m.nv());
  MatrixX dtau_dv(m.nv(), m.nv());
  MatrixX dtau_da(m.nv(), m.nv());
  rnea_derivatives(m, d, q, vv, aa, dtau_dq, dtau_dv, dtau_da);

  MatrixX big_m(m.nv(), m.nv());
  crba(m, d, q, big_m);

  // ∂τ/∂a is structurally symmetric because it equals M; the analytical
  // pass however only writes one direction (S^T df_da), so check both.
  const Scalar diff = (dtau_da - big_m).cwiseAbs().maxCoeff();
  EXPECT_LE(diff, 1e-10) << "∂τ/∂a should equal M(q) (from CRBA)";
}

TEST_P(RneaDerivativesTest, DtauDaIsSymmetric) {
  // M(q) symmetric → ∂τ/∂a symmetric.
  const Model m = build_model_from_urdf_file(fixture(GetParam()));
  Data d(m);
  std::mt19937 gen(42);
  std::uniform_real_distribution<Scalar> uni(-0.8, 0.8);
  VectorX q(m.nq());
  for (int i = 0; i < m.nq(); ++i) {
    q(i) = uni(gen);
  }
  VectorX vv(m.nv());
  VectorX aa(m.nv());
  for (int i = 0; i < m.nv(); ++i) {
    vv(i) = uni(gen);
    aa(i) = uni(gen);
  }
  MatrixX dtau_dq(m.nv(), m.nv());
  MatrixX dtau_dv(m.nv(), m.nv());
  MatrixX dtau_da(m.nv(), m.nv());
  rnea_derivatives(m, d, q, vv, aa, dtau_dq, dtau_dv, dtau_da);

  EXPECT_LE((dtau_da - dtau_da.transpose()).cwiseAbs().maxCoeff(), 1e-10);
}

INSTANTIATE_TEST_SUITE_P(AllFixtures, RneaDerivativesTest,
                         ::testing::Values("simple_arm.urdf", "franka_fr3.urdf", "ur5e.urdf",
                                           "so_arm101.urdf"));

}  // namespace
}  // namespace tinyspatial
