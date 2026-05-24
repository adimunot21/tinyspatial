#include <filesystem>
#include <random>
#include <string>

#include "tinyspatial/algo/crba.hpp"
#include "tinyspatial/algo/rnea.hpp"
#include "tinyspatial/urdf/urdf_loader.hpp"

#include <Eigen/Eigenvalues>

#include <gtest/gtest.h>

namespace tinyspatial {
namespace {

constexpr const char* kFixtureDir = TINYSPATIAL_TEST_FIXTURE_DIR;

std::string fixture(const std::string& name) {
  return (std::filesystem::path(kFixtureDir) / name).string();
}

TEST(Crba, MassMatrixIsSymmetricAndPSD) {
  // For every fixture and a few random configurations, M(q) must be symmetric
  // and positive definite (a non-zero-mass rigid system has no zero-energy
  // motion direction).
  const std::vector<std::string> urdfs = {"simple_arm.urdf", "franka_fr3.urdf", "ur5e.urdf",
                                          "so_arm101.urdf"};
  std::mt19937 gen(7);
  std::uniform_real_distribution<Scalar> uni(-1.5, 1.5);
  for (const auto& name : urdfs) {
    const Model m = build_model_from_urdf_file(fixture(name));
    Data d(m);
    MatrixX big_m(m.nv(), m.nv());
    for (int trial = 0; trial < 5; ++trial) {
      VectorX q(m.nq());
      for (int i = 0; i < m.nq(); ++i) {
        q(i) = uni(gen);
      }
      crba(m, d, q, big_m);
      // Symmetry to machine precision.
      const Scalar asym = (big_m - big_m.transpose()).cwiseAbs().maxCoeff();
      EXPECT_LE(asym, 1e-12) << "asymmetric M for " << name;
      // Positive definite — smallest eigenvalue strictly above zero.
      const Eigen::SelfAdjointEigenSolver<MatrixX> es(big_m);
      EXPECT_GT(es.eigenvalues().minCoeff(), 1e-9) << "non-PD M for " << name;
    }
  }
}

TEST(Crba, ConsistentWithRneaUnitVectors) {
  // Property: τ = M(q) q̈ + h(q, q̇)  ⇒  column j of M is RNEA(q, 0, e_j) − h(q, 0)
  // where h(q, 0) is RNEA(q, 0, 0) (the gravity-bias torque).
  // Validated on every fixture.
  const std::vector<std::string> urdfs = {"simple_arm.urdf", "franka_fr3.urdf", "ur5e.urdf",
                                          "so_arm101.urdf"};
  std::mt19937 gen(11);
  std::uniform_real_distribution<Scalar> uni(-1.2, 1.2);
  for (const auto& name : urdfs) {
    const Model m = build_model_from_urdf_file(fixture(name));
    Data d(m);
    VectorX q(m.nq());
    for (int i = 0; i < m.nq(); ++i) {
      q(i) = uni(gen);
    }
    const VectorX zero_v = VectorX::Zero(m.nv());
    // Bias h(q, 0): no gravity here so we can isolate inertial coupling.
    VectorX h(m.nv());
    rnea(m, d, q, zero_v, zero_v, h, Vector3::Zero());

    MatrixX big_m(m.nv(), m.nv());
    crba(m, d, q, big_m);

    MatrixX m_from_rnea(m.nv(), m.nv());
    for (int j = 0; j < m.nv(); ++j) {
      VectorX e_j = VectorX::Zero(m.nv());
      e_j(j) = 1.0;
      VectorX tau_col(m.nv());
      rnea(m, d, q, zero_v, e_j, tau_col, Vector3::Zero());
      m_from_rnea.col(j) = tau_col - h;
    }
    const Scalar diff = (big_m - m_from_rnea).cwiseAbs().maxCoeff();
    EXPECT_LE(diff, 1e-10) << "CRBA disagrees with RNEA on " << name;
  }
}

TEST(Crba, SimpleArmHandComputed) {
  // simple_arm at q=0: two unit point-mass links along x. The body-frame
  // inertia tensor for each is the parallel-axis from a 0.1 m radius shell
  // at COM=(0,0,0) of its own link. With z-axis rotations about joints
  // at (0,0,0) and (1,0,0), M(0) ≈ [[I11, I12], [I12, I22]] where
  //   I22 = J_link2_z(about joint 2) = J̄ + m·||COM_link2||² (= J̄ + 0)
  //   I11 = J̄_1 + J̄_2 + m_1·1² + m_2·(2² + 0) where the 2² comes from
  //          link 2's COM at distance 2 from joint 1.
  //   I12 = J̄_2 + m_2·(1² + 0)·... (cross block of the composite).
  // We just sanity-check shapes & diagonal positivity here; the rigorous
  // numerical check lives in the Pinocchio cross-validation.
  const Model m = build_model_from_urdf_file(fixture("simple_arm.urdf"));
  Data d(m);
  const VectorX q = VectorX::Zero(m.nq());
  MatrixX big_m(m.nv(), m.nv());
  crba(m, d, q, big_m);
  ASSERT_EQ(big_m.rows(), 2);
  ASSERT_EQ(big_m.cols(), 2);
  EXPECT_GT(big_m(0, 0), big_m(1, 1));  // outboard joint has less inertia behind it
  EXPECT_NEAR(big_m(0, 1), big_m(1, 0), 1e-12);
  EXPECT_GT(big_m(0, 0), 0.0);
  EXPECT_GT(big_m(1, 1), 0.0);
}

}  // namespace
}  // namespace tinyspatial
