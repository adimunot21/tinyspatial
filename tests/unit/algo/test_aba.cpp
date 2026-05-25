#include <filesystem>
#include <random>
#include <string>

#include "tinyspatial/algo/aba.hpp"
#include "tinyspatial/algo/crba.hpp"
#include "tinyspatial/algo/rnea.hpp"
#include "tinyspatial/urdf/urdf_loader.hpp"

#include <gtest/gtest.h>

namespace tinyspatial {
namespace {

constexpr const char* kFixtureDir = TINYSPATIAL_TEST_FIXTURE_DIR;

std::string fixture(const std::string& name) {
  return (std::filesystem::path(kFixtureDir) / name).string();
}

TEST(Aba, InverseOfRnea) {
  // The strongest property test: for any (q, v, a),
  //   τ = RNEA(q, v, a)  ⇒  ABA(q, v, τ) ≈ a
  // Exercised on every fixture, with gravity on.
  const std::vector<std::string> urdfs = {"simple_arm.urdf", "franka_fr3.urdf", "ur5e.urdf",
                                          "so_arm101.urdf"};
  std::mt19937 gen(31);
  std::uniform_real_distribution<Scalar> uni(-1.2, 1.2);
  const Vector3 gravity(0, 0, -9.81);
  for (const auto& name : urdfs) {
    const Model m = build_model_from_urdf_file(fixture(name));
    Data d(m);
    for (int trial = 0; trial < 5; ++trial) {
      VectorX q(m.nq());
      for (int i = 0; i < m.nq(); ++i) {
        q(i) = uni(gen);
      }
      VectorX v(m.nv());
      VectorX a(m.nv());
      for (int i = 0; i < m.nv(); ++i) {
        v(i) = uni(gen);
        a(i) = uni(gen);
      }
      VectorX tau(m.nv());
      rnea(m, d, q, v, a, tau, gravity);
      VectorX qdd(m.nv());
      aba(m, d, q, v, tau, qdd, gravity);
      const Scalar diff = (a - qdd).cwiseAbs().maxCoeff();
      EXPECT_LE(diff, 1e-9) << "ABA != RNEA^{-1} on " << name;
    }
  }
}

TEST(Aba, ConsistentWithCrbaAndRnea) {
  // Decoupling: τ = M(q) q̈ + h(q, q̇)  with  h = RNEA(q, v, 0).
  // So q̈ = M(q)⁻¹ · (τ − h). ABA should agree.
  const std::vector<std::string> urdfs = {"simple_arm.urdf", "franka_fr3.urdf", "ur5e.urdf",
                                          "so_arm101.urdf"};
  std::mt19937 gen(53);
  std::uniform_real_distribution<Scalar> uni(-1.1, 1.1);
  const Vector3 gravity(0, 0, -9.81);
  for (const auto& name : urdfs) {
    const Model m = build_model_from_urdf_file(fixture(name));
    Data d(m);
    VectorX q(m.nq());
    for (int i = 0; i < m.nq(); ++i) {
      q(i) = uni(gen);
    }
    VectorX v(m.nv());
    VectorX tau(m.nv());
    for (int i = 0; i < m.nv(); ++i) {
      v(i) = uni(gen);
      tau(i) = uni(gen);
    }

    // Bias h(q, v) (zero accel, with gravity).
    VectorX h(m.nv());
    rnea(m, d, q, v, VectorX::Zero(m.nv()), h, gravity);

    MatrixX big_m(m.nv(), m.nv());
    crba(m, d, q, big_m);

    const VectorX qdd_expected = big_m.ldlt().solve(tau - h);

    VectorX qdd_aba(m.nv());
    aba(m, d, q, v, tau, qdd_aba, gravity);

    const Scalar diff = (qdd_expected - qdd_aba).cwiseAbs().maxCoeff();
    EXPECT_LE(diff, 1e-9) << "ABA disagrees with M⁻¹(τ − h) on " << name;
  }
}

TEST(Aba, ZeroTorqueZeroVelocityNoGravityGivesZeroAccel) {
  // Static equilibrium in zero-g with no joint torques: nothing accelerates.
  const Model m = build_model_from_urdf_file(fixture("franka_fr3.urdf"));
  Data d(m);
  const VectorX q = VectorX::Zero(m.nq());
  const VectorX v = VectorX::Zero(m.nv());
  const VectorX tau = VectorX::Zero(m.nv());
  VectorX qdd(m.nv());
  aba(m, d, q, v, tau, qdd, Vector3::Zero());
  EXPECT_LE(qdd.cwiseAbs().maxCoeff(), 1e-12);
}

}  // namespace
}  // namespace tinyspatial
