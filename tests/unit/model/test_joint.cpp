#include "tinyspatial/model/joint.hpp"

#include <gtest/gtest.h>

namespace tinyspatial {
namespace {

constexpr Scalar kPi = 3.14159265358979323846;

template <typename A, typename B>
void expect_near(const A& a, const B& b, Scalar tol) {
  EXPECT_LE((a - b).cwiseAbs().maxCoeff(), tol) << "a=\n" << a << "\nb=\n" << b;
}

TEST(JointRevolute, AxisRotationAndDof) {
  const JointRevolute j{Vector3::UnitZ()};
  EXPECT_EQ(j.nq(), 1);
  EXPECT_EQ(j.nv(), 1);

  const SE3 t0 = j.transform(0.0);
  expect_near(t0.matrix(), Matrix4::Identity(), 1e-12);

  // 90° about z: a point at (1, 0, 0) maps to (0, 1, 0).
  const SE3 t90 = j.transform(kPi / 2);
  const Vector3 p(1.0, 0.0, 0.0);
  expect_near(t90.act(p), Vector3(0.0, 1.0, 0.0), 1e-12);
}

TEST(JointPrismatic, AxisTranslationAndDof) {
  const JointPrismatic j{Vector3::UnitX()};
  EXPECT_EQ(j.nq(), 1);
  EXPECT_EQ(j.nv(), 1);

  const SE3 t = j.transform(2.5);
  expect_near(t.rotation().matrix(), Matrix3::Identity(), 1e-12);
  expect_near(t.translation(), Vector3(2.5, 0.0, 0.0), 1e-12);
}

TEST(JointFixed, IdentityTransform) {
  const JointFixed j;
  EXPECT_EQ(j.nq(), 0);
  EXPECT_EQ(j.nv(), 0);
  expect_near(j.transform().matrix(), Matrix4::Identity(), 1e-12);
}

TEST(JointFloating, ConfigLayoutRoundTrip) {
  const JointFloating j;
  EXPECT_EQ(j.nq(), 7);
  EXPECT_EQ(j.nv(), 6);

  // Build q = (t, q_x, q_y, q_z, q_w) by hand from a known SE3.
  const SE3 target(SO3::exp(Vector3(0.3, -0.4, 1.1)), Vector3(1.5, -2.0, 0.5));
  const Quaternion q_target = target.rotation().quaternion();
  Eigen::Matrix<Scalar, 7, 1> q;
  q.head<3>() = target.translation();
  q(3) = q_target.x();
  q(4) = q_target.y();
  q(5) = q_target.z();
  q(6) = q_target.w();

  const SE3 t = j.transform(q);
  expect_near(t.matrix(), target.matrix(), 1e-12);
}

TEST(JointVariant, FreeFunctionDispatch) {
  // Build each variant, push into a Joint, and verify the free-function API
  // matches the typed call.
  const Joint jr = JointRevolute{Vector3::UnitY()};
  const Joint jp = JointPrismatic{Vector3::UnitZ()};
  const Joint jf = JointFixed{};
  const Joint jfl = JointFloating{};

  EXPECT_EQ(nq(jr), 1);
  EXPECT_EQ(nv(jr), 1);
  EXPECT_EQ(nq(jp), 1);
  EXPECT_EQ(nv(jp), 1);
  EXPECT_EQ(nq(jf), 0);
  EXPECT_EQ(nv(jf), 0);
  EXPECT_EQ(nq(jfl), 7);
  EXPECT_EQ(nv(jfl), 6);

  // Revolute via the variant
  VectorX qr(1);
  qr << 0.7;
  expect_near(joint_transform(jr, qr).matrix(), std::get<JointRevolute>(jr).transform(0.7).matrix(),
              1e-12);

  // Prismatic via the variant
  VectorX qp(1);
  qp << 1.5;
  expect_near(joint_transform(jp, qp).matrix(),
              std::get<JointPrismatic>(jp).transform(1.5).matrix(), 1e-12);

  // Fixed via the variant (q_slice is empty)
  VectorX qe(0);
  expect_near(joint_transform(jf, qe).matrix(), Matrix4::Identity(), 1e-12);
}

}  // namespace
}  // namespace tinyspatial
