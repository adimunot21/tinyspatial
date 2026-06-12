/// The dynamics payoff (Phase 2, P2.4): inverse dynamics is differentiable, and
/// its autodiff derivatives agree with two independent analytical sources.
///
/// Seeding q / v / a as autodiff variables and running the SAME `rnea` yields
/// ∂τ/∂q, ∂τ/∂v, ∂τ/∂a. We check:
///   - ∂τ/∂q, ∂τ/∂v, ∂τ/∂a  vs  `diff/rnea_derivatives.hpp` (Carpentier–Mansard
///     analytical recursion), and
///   - ∂τ/∂a  vs  `algo/crba.hpp` M(q)  — the standard identity τ = M(q)·a + …,
///     so ∂τ/∂a ≡ M(q).
/// Three independently-derived paths (autodiff, the analytical recursion, and
/// CRBA) agreeing to ~1e-10 is the dynamics half of the differentiable-first
/// correctness story.
#include <filesystem>
#include <random>
#include <string>

#include "tinyspatial/algo/crba.hpp"
#include "tinyspatial/algo/rnea.hpp"
#include "tinyspatial/core/jet.hpp"
#include "tinyspatial/diff/rnea_derivatives.hpp"
#include "tinyspatial/urdf/urdf_loader.hpp"

#include <gtest/gtest.h>

namespace tinyspatial {
namespace {

constexpr const char* kFixtureDir = TINYSPATIAL_TEST_FIXTURE_DIR;

std::string fixture(const std::string& name) {
  return (std::filesystem::path(kFixtureDir) / name).string();
}

// `N` is the robot's nv (compile-time Jet width). Fixtures here are all-revolute.
template <int N>
void check_rnea_ad(const std::string& robot) {
  const Model m = build_model_from_urdf_file(fixture(robot));
  ASSERT_EQ(m.nv(), N);

  std::mt19937 gen(31);
  std::uniform_real_distribution<double> uni(-1.0, 1.0);
  VectorX q(m.nq()), v(m.nv()), a(m.nv());
  for (int i = 0; i < m.nq(); ++i) {
    q(i) = uni(gen);
  }
  for (int i = 0; i < m.nv(); ++i) {
    v(i) = uni(gen);
    a(i) = uni(gen);
  }
  const Vector3 gravity(0, 0, -9.81);

  // Analytical oracles (double): rnea_derivatives + CRBA mass matrix.
  Data d(m);
  MatrixX dtau_dq(m.nv(), m.nv()), dtau_dv(m.nv(), m.nv()), dtau_da(m.nv(), m.nv());
  rnea_derivatives(m, d, q, v, a, dtau_dq, dtau_dv, dtau_da, gravity);
  MatrixX mass_matrix(m.nv(), m.nv());
  crba(m, d, q, mass_matrix);

  // AD path: rnea<Jet>, seeding one of (q, v, a) as independent variables.
  using J = Jet<N>;
  const ModelT<J> mj = model_cast<J>(m);
  auto seeded = [](const VectorX& x, bool seed) {
    typename Types<J>::VectorX xj(x.size());
    for (int k = 0; k < x.size(); ++k) {
      xj(k) = seed ? J(x(k), k) : J(x(k));
    }
    return xj;
  };
  auto ad_jacobian = [&](int wrt) {  // 0 = q, 1 = v, 2 = a
    DataT<J> dj(mj);
    typename Types<J>::VectorX qj = seeded(q, wrt == 0);
    typename Types<J>::VectorX vj = seeded(v, wrt == 1);
    typename Types<J>::VectorX aj = seeded(a, wrt == 2);
    typename Types<J>::VectorX tauj(m.nv());
    const typename Types<J>::Vector3 gj(J(0.0), J(0.0), J(-9.81));
    rnea(mj, dj, qj, vj, aj, tauj, gj);
    MatrixX jac(m.nv(), N);
    for (int r = 0; r < m.nv(); ++r) {
      for (int k = 0; k < N; ++k) {
        jac(r, k) = tauj(r).v[k];
      }
    }
    return jac;
  };

  const MatrixX ad_dq = ad_jacobian(0);
  const MatrixX ad_dv = ad_jacobian(1);
  const MatrixX ad_da = ad_jacobian(2);

  EXPECT_LE((ad_dq - dtau_dq).cwiseAbs().maxCoeff(), 1e-9) << robot << " dtau/dq";
  EXPECT_LE((ad_dv - dtau_dv).cwiseAbs().maxCoeff(), 1e-9) << robot << " dtau/dv";
  EXPECT_LE((ad_da - dtau_da).cwiseAbs().maxCoeff(), 1e-9) << robot << " dtau/da vs analytical";
  // The triangulating check: autodiff ∂τ/∂a equals the CRBA mass matrix.
  EXPECT_LE((ad_da - mass_matrix).cwiseAbs().maxCoeff(), 1e-9) << robot << " dtau/da vs CRBA M(q)";
}

TEST(RneaAd, MatchesAnalyticalAndCrbaFranka) {
  check_rnea_ad<7>("franka_fr3.urdf");
}
TEST(RneaAd, MatchesAnalyticalAndCrbaUr5e) {
  check_rnea_ad<6>("ur5e.urdf");
}

}  // namespace
}  // namespace tinyspatial
