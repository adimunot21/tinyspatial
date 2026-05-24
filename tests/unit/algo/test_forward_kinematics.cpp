#include <cmath>
#include <filesystem>
#include <random>
#include <string>

#include "tinyspatial/algo/forward_kinematics.hpp"
#include "tinyspatial/urdf/urdf_loader.hpp"

#include <gtest/gtest.h>

namespace tinyspatial {
namespace {

constexpr Scalar kPi = 3.14159265358979323846;
constexpr const char* kFixtureDir = TINYSPATIAL_TEST_FIXTURE_DIR;

std::string fixture(const std::string& name) {
  return (std::filesystem::path(kFixtureDir) / name).string();
}

TEST(ForwardKinematics, AtQZeroMatchesPlacementChain) {
  const Model m = build_model_from_urdf_file(fixture("simple_arm.urdf"));
  Data d(m);
  forward_kinematics(m, d, VectorX::Zero(m.nq()));

  // joint1: root, no rotation, no translation.
  EXPECT_TRUE(d.pose_in_world[0].matrix().isApprox(Matrix4::Identity()));
  // joint2: translated +1 along x, no rotation.
  EXPECT_TRUE(d.pose_in_world[1].translation().isApprox(Vector3(1, 0, 0)));
  EXPECT_TRUE(d.pose_in_world[1].rotation().matrix().isApprox(Matrix3::Identity()));
}

TEST(ForwardKinematics, HandComputedSimpleArm) {
  // simple_arm at q = (pi/2, 0).
  // joint1 rotates 90° about z. World pose at joint 1: rot_z(pi/2), at origin.
  // joint2 placement is +x by 1. World pose at joint 2: rot_z(pi/2) translated
  // by rot_z(pi/2) * (1,0,0) = (0, 1, 0).
  const Model m = build_model_from_urdf_file(fixture("simple_arm.urdf"));
  Data d(m);
  VectorX q(2);
  q << kPi / 2, 0.0;
  forward_kinematics(m, d, q);

  const SO3 expected_rot = SO3::exp(Vector3(0, 0, kPi / 2));
  EXPECT_TRUE(d.pose_in_world[0].rotation().matrix().isApprox(expected_rot.matrix()));
  EXPECT_TRUE(d.pose_in_world[0].translation().isApprox(Vector3::Zero()));
  EXPECT_TRUE(d.pose_in_world[1].rotation().matrix().isApprox(expected_rot.matrix()));
  EXPECT_LE((d.pose_in_world[1].translation() - Vector3(0, 1, 0)).cwiseAbs().maxCoeff(), 1e-12);
}

TEST(ForwardKinematics, PoseInParentIsLocalContribution) {
  // pose_in_parent[i] should equal placement[i] * joint_transform(joints[i], q).
  // pose_in_world chain is the cumulative product of pose_in_parent.
  const Model m = build_model_from_urdf_file(fixture("simple_arm.urdf"));
  Data d(m);
  VectorX q(2);
  q << 0.3, -0.7;
  forward_kinematics(m, d, q);

  for (int i = 0; i < m.njoints(); ++i) {
    const VectorX q_slice = q.segment(m.idx_q[i], nq(m.joints[i]));
    const SE3 expected_local = m.placement[i] * joint_transform(m.joints[i], q_slice);
    EXPECT_TRUE(d.pose_in_parent[i].matrix().isApprox(expected_local.matrix()));

    const SE3 expected_world =
        (m.parent[i] == -1) ? expected_local : d.pose_in_world[m.parent[i]] * expected_local;
    EXPECT_TRUE(d.pose_in_world[i].matrix().isApprox(expected_world.matrix()));
  }
}

class FixtureFkTest : public ::testing::TestWithParam<std::string> {};

TEST_P(FixtureFkTest, RandomConfigsDontCrashAndStaySane) {
  const Model m = build_model_from_urdf_file(fixture(GetParam()));
  Data d(m);
  std::mt19937 gen(31);
  std::uniform_real_distribution<Scalar> uni(-1.5, 1.5);

  for (int trial = 0; trial < 50; ++trial) {
    VectorX q(m.nq());
    for (int i = 0; i < m.nq(); ++i) {
      q(i) = uni(gen);
    }
    // Floating joints: the quaternion slice (indices 3..6 in their q-block)
    // must be unit-norm for `JointFloating::transform` to give a valid pose.
    for (int j = 0; j < m.njoints(); ++j) {
      if (std::holds_alternative<JointFloating>(m.joints[j])) {
        const int start = m.idx_q[j] + 3;
        Vector3 v(uni(gen), uni(gen), uni(gen));
        const Scalar n = std::sqrt(v.squaredNorm() + 1);  // a quaternion (x,y,z,w)
        q.segment<3>(start) = v / n;
        q(start + 3) = 1.0 / n;
      }
    }
    EXPECT_NO_THROW(forward_kinematics(m, d, q));
    // Rotation matrices should remain orthonormal under composition.
    for (int i = 0; i < m.njoints(); ++i) {
      const Matrix3 r = d.pose_in_world[i].rotation().matrix();
      EXPECT_LE((r.transpose() * r - Matrix3::Identity()).cwiseAbs().maxCoeff(), 1e-10);
    }
  }
}

INSTANTIATE_TEST_SUITE_P(AllFixtures, FixtureFkTest,
                         ::testing::Values("franka_fr3.urdf", "ur5e.urdf", "so_arm101.urdf"));

}  // namespace
}  // namespace tinyspatial
