/// Differentiable CRBA / ABA (Phase 2, P2.5) — completes the algorithm suite.
///
/// Two checks:
///   - CRBA on `Jet` reproduces the `double` mass matrix (templatization is
///     value-correct), and
///   - ABA, seeded on `τ`, yields ∂q̈/∂τ — which must equal `M(q)⁻¹`, since
///     `q̈ = M⁻¹(τ − h)`. That ties forward dynamics (ABA) back to the mass
///     matrix (CRBA) through autodiff, closing the dynamics loop:
///     RNEA gives M (= ∂τ/∂a), ABA gives M⁻¹ (= ∂q̈/∂τ).
#include <filesystem>
#include <random>
#include <string>

#include "tinyspatial/algo/aba.hpp"
#include "tinyspatial/algo/crba.hpp"
#include "tinyspatial/core/jet.hpp"
#include "tinyspatial/urdf/urdf_loader.hpp"

#include <gtest/gtest.h>

namespace tinyspatial {
namespace {

constexpr const char* kFixtureDir = TINYSPATIAL_TEST_FIXTURE_DIR;

std::string fixture(const std::string& name) {
  return (std::filesystem::path(kFixtureDir) / name).string();
}

template <int N>
void check_dynamics_ad(const std::string& robot) {
  const Model m = build_model_from_urdf_file(fixture(robot));
  ASSERT_EQ(m.nv(), N);

  std::mt19937 gen(57);
  std::uniform_real_distribution<double> uni(-1.0, 1.0);
  VectorX q(m.nq()), v(m.nv()), tau(m.nv());
  for (int i = 0; i < m.nq(); ++i) {
    q(i) = uni(gen);
  }
  for (int i = 0; i < m.nv(); ++i) {
    v(i) = uni(gen);
    tau(i) = uni(gen);
  }
  const Vector3 gravity(0, 0, -9.81);

  // Double oracles.
  Data d(m);
  MatrixX mass_matrix(m.nv(), m.nv());
  crba(m, d, q, mass_matrix);
  VectorX qdd_double(m.nv());
  aba(m, d, q, v, tau, qdd_double, gravity);

  using J = Jet<N>;
  const ModelT<J> mj = model_cast<J>(m);
  auto constant = [](const VectorX& x) {
    typename Types<J>::VectorX xj(x.size());
    for (int k = 0; k < x.size(); ++k) {
      xj(k) = J(x(k));
    }
    return xj;
  };

  // 1) CRBA<Jet> value reproduces the double mass matrix.
  {
    DataT<J> dj(mj);
    typename Types<J>::MatrixX mj_out(m.nv(), m.nv());
    crba(mj, dj, constant(q), mj_out);
    double diff = 0.0;
    for (int r = 0; r < m.nv(); ++r) {
      for (int c = 0; c < m.nv(); ++c) {
        diff = std::max(diff, std::abs(mj_out(r, c).a - mass_matrix(r, c)));
      }
    }
    EXPECT_LE(diff, 1e-9) << robot << " CRBA<Jet> value vs double";
  }

  // 2) ABA<Jet> with τ seeded: value matches double, and ∂q̈/∂τ == M(q)⁻¹.
  {
    DataT<J> dj(mj);
    typename Types<J>::VectorX qj = constant(q);
    typename Types<J>::VectorX vj = constant(v);
    typename Types<J>::VectorX tauj(m.nv());
    for (int k = 0; k < m.nv(); ++k) {
      tauj(k) = J(tau(k), k);  // seed τ as the independent variables
    }
    typename Types<J>::VectorX qddj(m.nv());
    const typename Types<J>::Vector3 gj(J(0.0), J(0.0), J(-9.81));
    aba(mj, dj, qj, vj, tauj, qddj, gj);

    double vdiff = 0.0;
    MatrixX dqdd_dtau(m.nv(), N);
    for (int r = 0; r < m.nv(); ++r) {
      vdiff = std::max(vdiff, std::abs(qddj(r).a - qdd_double(r)));
      for (int k = 0; k < N; ++k) {
        dqdd_dtau(r, k) = qddj(r).v[k];
      }
    }
    EXPECT_LE(vdiff, 1e-9) << robot << " ABA<Jet> value vs double";
    const MatrixX m_inverse = mass_matrix.inverse();
    EXPECT_LE((dqdd_dtau - m_inverse).cwiseAbs().maxCoeff(), 1e-8)
        << robot << " dqdd/dtau vs M(q)^-1";
  }
}

TEST(DynamicsAd, CrbaAndAbaFranka) {
  check_dynamics_ad<7>("franka_fr3.urdf");
}
TEST(DynamicsAd, CrbaAndAbaUr5e) {
  check_dynamics_ad<6>("ur5e.urdf");
}

}  // namespace
}  // namespace tinyspatial
