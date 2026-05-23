#include "tinyspatial/model/model.hpp"

#include <gtest/gtest.h>

namespace tinyspatial {
namespace {

constexpr Scalar kPi = 3.14159265358979323846;

// A tiny 2-DOF planar arm in the xz plane: two revolute joints about y, with
// a unit-length link in between and a unit point mass at each end.
//
//     world ──[J0:revolute y]── link0(len=1) ──[J1:revolute y]── link1(len=1)
//                                |─ inertia: 1 kg point mass at end of link0
//                                                                  ditto link1
Model build_two_dof_arm() {
  Model m;
  m.name = "2dof_planar";
  // First joint at the world origin; its body (link 0) sits one unit along +x
  // when the joint is at q=0, with a point mass 1 kg at the link tip.
  m.add_joint("joint0", "link0", -1, JointRevolute{Vector3::UnitY()}, SE3::identity(),
              SpatialInertia(1.0, Vector3(1.0, 0.0, 0.0), Matrix3::Zero()));
  // Second joint attached at the *end* of link 0 (placement = translate +x by 1).
  m.add_joint("joint1", "link1", 0, JointRevolute{Vector3::UnitY()},
              SE3(SO3::identity(), Vector3(1.0, 0.0, 0.0)),
              SpatialInertia(1.0, Vector3(1.0, 0.0, 0.0), Matrix3::Zero()));
  return m;
}

TEST(Model, ShapeAndIndexing) {
  const Model m = build_two_dof_arm();

  EXPECT_EQ(m.njoints(), 2);
  EXPECT_EQ(m.nq(), 2);
  EXPECT_EQ(m.nv(), 2);

  EXPECT_EQ(m.joint_names[0], "joint0");
  EXPECT_EQ(m.joint_names[1], "joint1");
  EXPECT_EQ(m.parent[0], -1);
  EXPECT_EQ(m.parent[1], 0);
  EXPECT_EQ(m.idx_q[0], 0);
  EXPECT_EQ(m.idx_q[1], 1);
  EXPECT_EQ(m.idx_v[0], 0);
  EXPECT_EQ(m.idx_v[1], 1);
}

TEST(Model, FindJoint) {
  const Model m = build_two_dof_arm();
  EXPECT_EQ(m.find_joint("joint0"), 0);
  EXPECT_EQ(m.find_joint("joint1"), 1);
  EXPECT_EQ(m.find_joint("nope"), -1);
}

TEST(Model, MixedJointConfigSizes) {
  // A pathological model exercising every variant — checks idx_q / idx_v
  // bookkeeping for non-uniform joint sizes.
  Model m;
  m.add_joint("base", "root", -1, JointFloating{}, SE3::identity(), SpatialInertia::zero());
  m.add_joint("hinge", "arm", 0, JointRevolute{Vector3::UnitZ()}, SE3::identity(),
              SpatialInertia::zero());
  m.add_joint("welded", "tip", 1, JointFixed{}, SE3::identity(), SpatialInertia::zero());
  m.add_joint("slider", "ee", 2, JointPrismatic{Vector3::UnitX()}, SE3::identity(),
              SpatialInertia::zero());

  EXPECT_EQ(m.nq(), 7 + 1 + 0 + 1);  // 9
  EXPECT_EQ(m.nv(), 6 + 1 + 0 + 1);  // 8
  EXPECT_EQ(m.idx_q[0], 0);
  EXPECT_EQ(m.idx_q[1], 7);
  EXPECT_EQ(m.idx_q[2], 8);  // fixed contributes nothing, so slider starts here
  EXPECT_EQ(m.idx_q[3], 8);
  EXPECT_EQ(m.idx_v[3], 7);
}

TEST(Model, DataIsSizedToMatch) {
  const Model m = build_two_dof_arm();
  Data d(m);
  EXPECT_EQ(static_cast<int>(d.pose_in_parent.size()), m.njoints());
  EXPECT_EQ(static_cast<int>(d.pose_in_world.size()), m.njoints());
  EXPECT_EQ(static_cast<int>(d.v.size()), m.njoints());
  EXPECT_EQ(static_cast<int>(d.a.size()), m.njoints());
  EXPECT_EQ(static_cast<int>(d.f.size()), m.njoints());
  // Default-constructed values are identity / zero.
  EXPECT_TRUE(d.pose_in_world[0].matrix().isApprox(Matrix4::Identity()));
  EXPECT_EQ(d.v[0].vector(), Vector6::Zero());
}

TEST(Model, ChainedTransformsAtQZero) {
  // At q=0 the chain reduces to the joint placements alone.
  const Model m = build_two_dof_arm();
  VectorX q = VectorX::Zero(m.nq());
  // World→joint0 = placement[0] = identity. World→joint1 = placement[0] * placement[1].
  const SE3 w_to_j0 =
      m.placement[0] * joint_transform(m.joints[0], q.segment(m.idx_q[0], nq(m.joints[0])));
  const SE3 j0_to_j1 =
      m.placement[1] * joint_transform(m.joints[1], q.segment(m.idx_q[1], nq(m.joints[1])));
  const SE3 w_to_j1 = w_to_j0 * j0_to_j1;
  EXPECT_TRUE(w_to_j0.matrix().isApprox(Matrix4::Identity()));
  // Joint 1 sits one unit along +x of the world (link 0's length).
  EXPECT_TRUE(w_to_j1.translation().isApprox(Vector3(1.0, 0.0, 0.0)));
}

}  // namespace
}  // namespace tinyspatial
