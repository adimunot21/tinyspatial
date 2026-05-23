#include <filesystem>
#include <string>
#include <variant>

#include "tinyspatial/urdf/urdf_loader.hpp"

#include <gtest/gtest.h>

namespace tinyspatial {
namespace {

// Absolute path to the data/robots/ directory; injected by CMake.
constexpr const char* kFixtureDir = TINYSPATIAL_TEST_FIXTURE_DIR;

std::string fixture(const std::string& name) {
  return (std::filesystem::path(kFixtureDir) / name).string();
}

TEST(UrdfLoader, InlineMinimalRobot) {
  // The smallest URDF that should load: one link, no joints.
  const std::string xml = R"(<?xml version="1.0"?>
<robot name="empty">
  <link name="root"/>
</robot>)";
  const Model m = build_model_from_urdf(xml);
  EXPECT_EQ(m.name, "empty");
  EXPECT_EQ(m.njoints(), 0);
  EXPECT_EQ(m.nq(), 0);
}

TEST(UrdfLoader, SimpleArmRoundTrip) {
  const Model m = build_model_from_urdf_file(fixture("simple_arm.urdf"));
  EXPECT_EQ(m.name, "simple_arm");
  EXPECT_EQ(m.njoints(), 2);
  EXPECT_EQ(m.nq(), 2);
  EXPECT_EQ(m.nv(), 2);
  EXPECT_EQ(m.parent[0], -1);
  EXPECT_EQ(m.parent[1], 0);
  EXPECT_EQ(m.joint_names[0], "joint1");
  EXPECT_EQ(m.joint_names[1], "joint2");
  EXPECT_EQ(m.link_names[0], "link1");
  EXPECT_EQ(m.link_names[1], "link2");
  // joint2's placement is +x by 1 (the length of link1).
  EXPECT_TRUE(m.placement[1].translation().isApprox(Vector3(1.0, 0.0, 0.0)));
  // The link inertia carries the point-mass-at-tip parameters.
  EXPECT_DOUBLE_EQ(m.inertia[0].mass(), 1.0);
  EXPECT_TRUE(m.inertia[0].com().isApprox(Vector3(1.0, 0.0, 0.0)));
}

TEST(UrdfLoader, FrankaFr3HasSevenDof) {
  const Model m = build_model_from_urdf_file(fixture("franka_fr3.urdf"));
  EXPECT_EQ(m.name, "franka_fr3");
  EXPECT_EQ(m.njoints(), 7);
  EXPECT_EQ(m.nq(), 7);
  EXPECT_EQ(m.nv(), 7);
  // All revolute — every joint contributes 1 DOF.
  for (const Joint& j : m.joints) {
    EXPECT_TRUE(std::holds_alternative<JointRevolute>(j));
  }
  // The chain is a single serial line: parent[i] == i-1 for i ≥ 1.
  for (int i = 1; i < m.njoints(); ++i) {
    EXPECT_EQ(m.parent[i], i - 1);
  }
}

TEST(UrdfLoader, Ur5eHasSixDof) {
  const Model m = build_model_from_urdf_file(fixture("ur5e.urdf"));
  EXPECT_EQ(m.name, "ur5e");
  EXPECT_EQ(m.njoints(), 6);
  EXPECT_EQ(m.nq(), 6);
  EXPECT_EQ(m.nv(), 6);
  EXPECT_EQ(m.find_joint("elbow_joint"), 2);
}

TEST(UrdfLoader, SoArm101HasMixedJoints) {
  const Model m = build_model_from_urdf_file(fixture("so_arm101.urdf"));
  EXPECT_EQ(m.name, "so_arm101");
  // 5 revolute joints + 1 fixed gripper joint.
  EXPECT_EQ(m.njoints(), 6);
  EXPECT_EQ(m.nq(), 5);  // fixed joint contributes 0
  EXPECT_EQ(m.nv(), 5);
  EXPECT_TRUE(std::holds_alternative<JointFixed>(m.joints.back()));
}

TEST(UrdfLoader, RejectsXmlGarbage) {
  EXPECT_THROW(build_model_from_urdf("not xml at all <<<"), UrdfParseError);
}

TEST(UrdfLoader, RejectsMissingRobot) {
  EXPECT_THROW(build_model_from_urdf("<other/>"), UrdfParseError);
}

TEST(UrdfLoader, RejectsUnknownJointType) {
  const std::string xml = R"(<?xml version="1.0"?>
<robot name="bad"><link name="a"/><link name="b"/>
<joint name="j" type="screw">
  <parent link="a"/><child link="b"/>
</joint></robot>)";
  EXPECT_THROW(build_model_from_urdf(xml), UrdfParseError);
}

TEST(UrdfLoader, RejectsUnknownChildLink) {
  const std::string xml = R"(<?xml version="1.0"?>
<robot name="dangling"><link name="a"/>
<joint name="j" type="fixed">
  <parent link="a"/><child link="missing"/>
</joint></robot>)";
  EXPECT_THROW(build_model_from_urdf(xml), UrdfParseError);
}

TEST(UrdfLoader, RejectsCycle) {
  // Two joints forming a cycle a→b→a have no root link.
  const std::string xml = R"(<?xml version="1.0"?>
<robot name="cycle"><link name="a"/><link name="b"/>
<joint name="ab" type="fixed"><parent link="a"/><child link="b"/></joint>
<joint name="ba" type="fixed"><parent link="b"/><child link="a"/></joint>
</robot>)";
  EXPECT_THROW(build_model_from_urdf(xml), UrdfParseError);
}

TEST(UrdfLoader, IgnoresVisualAndCollision) {
  // Visual/collision tags must be silently skipped, not error.
  const std::string xml = R"(<?xml version="1.0"?>
<robot name="vc"><link name="a">
  <visual><origin xyz="0 0 0"/><geometry><box size="1 1 1"/></geometry></visual>
  <collision><origin xyz="0 0 0"/><geometry><box size="1 1 1"/></geometry></collision>
</link></robot>)";
  EXPECT_NO_THROW(build_model_from_urdf(xml));
}

}  // namespace
}  // namespace tinyspatial
