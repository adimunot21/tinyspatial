#include <filesystem>
#include <random>
#include <string>

#include "tinyspatial/algo/rnea.hpp"
#include "tinyspatial/urdf/urdf_loader.hpp"

#include <gtest/gtest.h>

namespace tinyspatial {
namespace {

constexpr const char* kFixtureDir = TINYSPATIAL_TEST_FIXTURE_DIR;

std::string fixture(const std::string& name) {
  return (std::filesystem::path(kFixtureDir) / name).string();
}

TEST(Rnea, NoGravityNoMotionGivesZeroTorque) {
  // With no gravity and v=a=0 there are no forces to balance — all joints
  // must have zero torque.
  const Model m = build_model_from_urdf_file(fixture("simple_arm.urdf"));
  Data d(m);
  const VectorX zero = VectorX::Zero(m.nq());
  VectorX tau(m.nv());
  rnea(m, d, zero, zero, zero, tau, /*gravity=*/Vector3::Zero());
  EXPECT_LE(tau.cwiseAbs().maxCoeff(), 1e-12);
}

TEST(Rnea, HandComputedGravityCompensation) {
  // simple_arm at q=0: link 1 mass (1 kg) at world (1,0,0); link 2 mass (1 kg)
  // at world (2,0,0). With gravity in -y, the static-balance torques about
  // each z-axis are:
  //   joint 1: r1 × m1·g + r2 × m2·g, z-component = -(1)(−9.81) − (2)(−9.81) = 29.43
  //   joint 2: (r2 − r1) × m2·g = (1)(9.81) = 9.81
  // RNEA returns the joint torques required to hold q̈=0 against gravity, so
  // these are the positive numbers above.
  const Model m = build_model_from_urdf_file(fixture("simple_arm.urdf"));
  Data d(m);
  const VectorX zero = VectorX::Zero(m.nq());
  VectorX tau(m.nv());
  rnea(m, d, zero, zero, zero, tau, /*gravity=*/Vector3(0, -9.81, 0));
  EXPECT_NEAR(tau(0), 29.43, 1e-10);
  EXPECT_NEAR(tau(1), 9.81, 1e-10);
}

TEST(Rnea, AccelerationProducesCoupledTorque) {
  // At q=v=0 (no gravity) with q̈ = (1, 0): only joint 1 accelerates. The
  // angular momentum of both link masses changes, contributing to τ. With
  // each mass at unit distance from its joint, the torque ≥ 0.
  const Model m = build_model_from_urdf_file(fixture("simple_arm.urdf"));
  Data d(m);
  VectorX q = VectorX::Zero(m.nq());
  VectorX v = VectorX::Zero(m.nv());
  VectorX a(m.nv());
  a << 1.0, 0.0;
  VectorX tau(m.nv());
  rnea(m, d, q, v, a, tau, /*gravity=*/Vector3::Zero());
  // Joint 1 has both link masses on its outboard side; joint 2 has only
  // link 2's. So τ(0) must exceed τ(1).
  EXPECT_GT(tau(0), tau(1));
  EXPECT_GT(tau(0), 0.0);
}

class FixtureRneaTest : public ::testing::TestWithParam<std::string> {};

TEST_P(FixtureRneaTest, RandomInputsDontCrashAndStaySane) {
  const Model m = build_model_from_urdf_file(fixture(GetParam()));
  Data d(m);
  std::mt19937 gen(53);
  std::uniform_real_distribution<Scalar> uni(-1.5, 1.5);
  for (int trial = 0; trial < 30; ++trial) {
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
    EXPECT_NO_THROW(rnea(m, d, q, v, a, tau));
    EXPECT_TRUE(tau.allFinite());
  }
}

INSTANTIATE_TEST_SUITE_P(AllFixtures, FixtureRneaTest,
                         ::testing::Values("simple_arm.urdf", "franka_fr3.urdf", "ur5e.urdf",
                                           "so_arm101.urdf"));

}  // namespace
}  // namespace tinyspatial
