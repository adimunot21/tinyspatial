// Joint-frame placements at q=0 against a hand-computed reference for each
// fixture URDF. The placement of joint i in its parent's body frame is exactly
// the `<origin>` element of the URDF joint; the world-frame pose of each joint
// at q=0 falls out of chaining placements along `Model::parent`.
#include <cmath>
#include <filesystem>
#include <ranges>
#include <string>
#include <vector>

#include "tinyspatial/urdf/urdf_loader.hpp"

#include <gtest/gtest.h>

namespace tinyspatial {
namespace {

constexpr const char* kFixtureDir = TINYSPATIAL_TEST_FIXTURE_DIR;

std::string fixture(const std::string& name) {
  return (std::filesystem::path(kFixtureDir) / name).string();
}

constexpr Scalar kPi = 3.14159265358979323846;

// Chain placements along the parent[] array to get each joint's world pose
// at q = 0 (no joint variable contributions, just the static <origin> data).
SE3 world_pose_at_q_zero(const Model& m, int joint_idx) {
  SE3 result = SE3::identity();
  // Walk from root down to `joint_idx`.
  std::vector<int> chain;
  for (int j = joint_idx; j != -1; j = m.parent[j]) {
    chain.push_back(j);
  }
  for (const int j : std::ranges::reverse_view(chain)) {
    result = result * m.placement[j];
  }
  return result;
}

TEST(UrdfPlacements, SimpleArmJointPositionsAtQZero) {
  const Model m = build_model_from_urdf_file(fixture("simple_arm.urdf"));
  // joint1 sits at the world origin; joint2 sits 1 m down the +x axis
  // (placement[1] = (1, 0, 0), no rotation).
  EXPECT_TRUE(world_pose_at_q_zero(m, 0).translation().isApprox(Vector3::Zero()));
  EXPECT_TRUE(world_pose_at_q_zero(m, 1).translation().isApprox(Vector3(1, 0, 0)));
  EXPECT_TRUE(world_pose_at_q_zero(m, 0).rotation().matrix().isApprox(Matrix3::Identity()));
  EXPECT_TRUE(world_pose_at_q_zero(m, 1).rotation().matrix().isApprox(Matrix3::Identity()));
}

TEST(UrdfPlacements, FrankaFr3FirstJointHasExpectedZOffset) {
  const Model m = build_model_from_urdf_file(fixture("franka_fr3.urdf"));
  // fr3_joint1 placement: (0, 0, 0.333), no rotation.
  EXPECT_TRUE(m.placement[0].translation().isApprox(Vector3(0, 0, 0.333)));
  EXPECT_TRUE(m.placement[0].rotation().matrix().isApprox(Matrix3::Identity()));

  // fr3_joint2 placement: (0, 0, 0) with rpy = (-pi/2, 0, 0). The URDF stores
  // the angle as the truncated literal 1.5707963 (~1e-7 from exact pi/2), so
  // we compare with a matching tolerance instead of Eigen's default 1e-12.
  EXPECT_LE(m.placement[1].translation().cwiseAbs().maxCoeff(), 1e-12);
  const SO3 expected_r2(Eigen::AngleAxis<Scalar>(-kPi / 2, Vector3::UnitX()).toRotationMatrix());
  EXPECT_LE((m.placement[1].rotation().matrix() - expected_r2.matrix()).cwiseAbs().maxCoeff(),
            1e-6);
}

TEST(UrdfPlacements, Ur5eShoulderAndUpperArmPlacements) {
  const Model m = build_model_from_urdf_file(fixture("ur5e.urdf"));
  // shoulder_pan_joint origin: (0, 0, 0.1625), no rotation.
  EXPECT_TRUE(m.placement[0].translation().isApprox(Vector3(0, 0, 0.1625)));
  EXPECT_TRUE(m.placement[0].rotation().matrix().isApprox(Matrix3::Identity()));

  // shoulder_lift_joint origin: (0, 0, 0) with rpy = (0, pi/2, 0). URDF stores
  // the truncated literal 1.5707963, so compare at 1e-6 instead of default.
  EXPECT_LE(m.placement[1].translation().cwiseAbs().maxCoeff(), 1e-12);
  const SO3 expected_lift(Eigen::AngleAxis<Scalar>(kPi / 2, Vector3::UnitY()).toRotationMatrix());
  EXPECT_LE((m.placement[1].rotation().matrix() - expected_lift.matrix()).cwiseAbs().maxCoeff(),
            1e-6);

  // elbow_joint origin: (-0.425, 0, 0), no rotation.
  EXPECT_TRUE(m.placement[2].translation().isApprox(Vector3(-0.425, 0, 0)));
}

TEST(UrdfPlacements, SoArm101CumulativeReachAtQZero) {
  const Model m = build_model_from_urdf_file(fixture("so_arm101.urdf"));
  // The arm is vertical at q=0; cumulative +z offsets are
  // 0.04 (base→shoulder) + 0.08 (shoulder→upper_arm) + 0.116 (upper_arm→forearm)
  // + 0.116 (forearm→wrist_pitch) + 0.06 (wrist_pitch→wrist_roll)
  // + 0.05 (wrist_roll→gripper_palm) = 0.462 m.
  const SE3 ee = world_pose_at_q_zero(m, m.njoints() - 1);
  EXPECT_NEAR(ee.translation().z(), 0.462, 1e-12);
  EXPECT_NEAR(ee.translation().x(), 0.0, 1e-12);
  EXPECT_NEAR(ee.translation().y(), 0.0, 1e-12);
  EXPECT_TRUE(ee.rotation().matrix().isApprox(Matrix3::Identity()));
}

}  // namespace
}  // namespace tinyspatial
