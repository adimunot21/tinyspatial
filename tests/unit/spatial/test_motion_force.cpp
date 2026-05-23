#include <random>

#include "tinyspatial/spatial/cross.hpp"
#include "tinyspatial/spatial/force.hpp"
#include "tinyspatial/spatial/motion.hpp"

#include <gtest/gtest.h>

namespace tinyspatial {
namespace {

constexpr Scalar kTight = 1e-12;

class MotionForceTest : public ::testing::Test {
 protected:
  std::mt19937 gen_{19};
  std::uniform_real_distribution<Scalar> uni_{-1.0, 1.0};

  Vector3 random_vec() { return Vector3(uni_(gen_), uni_(gen_), uni_(gen_)); }
  Motion random_motion() { return Motion(random_vec(), random_vec()); }
  Force random_force() { return Force(random_vec(), random_vec()); }

  // A bounded-rotation SE(3) so we stay in the exp/log chart.
  SE3 random_se3() {
    Vector3 axis = random_vec();
    while (axis.norm() < 1e-6) {
      axis = random_vec();
    }
    constexpr Scalar kPi = 3.14159265358979323846;
    const Scalar angle = (uni_(gen_) * 0.5 + 0.5) * (kPi - 1e-3);
    Vector6 xi;
    xi.head<3>() = axis.normalized() * angle;
    xi.tail<3>() = random_vec();
    return SE3::exp(xi);
  }
};

TEST(MotionBasics, ConstructorsAndAccessors) {
  const Motion m;  // zero
  EXPECT_EQ(m.vector(), Vector6::Zero());

  const Vector3 w(1.0, 2.0, 3.0);
  const Vector3 v(4.0, 5.0, 6.0);
  const Motion m2(w, v);
  EXPECT_EQ(m2.angular(), w);
  EXPECT_EQ(m2.linear(), v);

  Vector6 packed;
  packed << w, v;
  EXPECT_EQ(m2.vector(), packed);
}

TEST(ForceBasics, ConstructorsAndAccessors) {
  const Vector3 tau(0.5, -1.0, 2.0);
  const Vector3 fl(7.0, 0.0, -3.0);
  const Force f(tau, fl);
  EXPECT_EQ(f.angular(), tau);
  EXPECT_EQ(f.linear(), fl);
}

TEST_F(MotionForceTest, Arithmetic) {
  const Motion a = random_motion();
  const Motion b = random_motion();
  EXPECT_TRUE((a + b).vector().isApprox(a.vector() + b.vector()));
  EXPECT_TRUE((a - b).vector().isApprox(a.vector() - b.vector()));
  EXPECT_TRUE((-a).vector().isApprox(-a.vector()));
  EXPECT_TRUE((a * 2.5).vector().isApprox(a.vector() * 2.5));
  EXPECT_TRUE((2.5 * a).vector().isApprox(a.vector() * 2.5));
  Motion c = a;
  c += b;
  EXPECT_TRUE(c.vector().isApprox(a.vector() + b.vector()));
  c -= b;
  EXPECT_TRUE(c.vector().isApprox(a.vector()));
}

TEST_F(MotionForceTest, CrossAgreesWithMatrix) {
  for (int i = 0; i < 20; ++i) {
    const Motion v = random_motion();
    const Motion m = random_motion();
    const Force f = random_force();
    EXPECT_TRUE(cross(v, m).vector().isApprox(cross_motion(v.vector()) * m.vector()));
    EXPECT_TRUE(cross(v, f).vector().isApprox(cross_force(v.vector()) * f.vector()));
  }
}

TEST_F(MotionForceTest, MotionCrossIsAntisymmetric) {
  for (int i = 0; i < 20; ++i) {
    const Motion a = random_motion();
    const Motion b = random_motion();
    EXPECT_TRUE(cross(a, b).vector().isApprox(-cross(b, a).vector()));
  }
}

TEST_F(MotionForceTest, MotionCrossJacobi) {
  // [a,[b,c]] + [b,[c,a]] + [c,[a,b]] = 0
  for (int i = 0; i < 20; ++i) {
    const Motion a = random_motion();
    const Motion b = random_motion();
    const Motion c = random_motion();
    const Vector6 lhs = cross(a, cross(b, c)).vector() + cross(b, cross(c, a)).vector() +
                        cross(c, cross(a, b)).vector();
    EXPECT_LE(lhs.cwiseAbs().maxCoeff(), 1e-12) << lhs.transpose();
  }
}

TEST_F(MotionForceTest, Se3ActsOnMotionIsAdjoint) {
  for (int i = 0; i < 20; ++i) {
    const SE3 t = random_se3();
    const Motion m = random_motion();
    EXPECT_TRUE((t * m).vector().isApprox(t.adjoint() * m.vector()));
  }
}

TEST_F(MotionForceTest, Se3MotionRoundTrip) {
  for (int i = 0; i < 20; ++i) {
    const SE3 t = random_se3();
    const Motion m = random_motion();
    const Motion round = t * (t.inverse() * m);
    EXPECT_LE((round.vector() - m.vector()).cwiseAbs().maxCoeff(), 1e-10);
  }
}

TEST_F(MotionForceTest, Se3ActsOnForceIsDualAdjoint) {
  // F' = Ad^{-T} F. Verify against the inverse-transpose of the motion adjoint.
  for (int i = 0; i < 20; ++i) {
    const SE3 t = random_se3();
    const Force f = random_force();
    const Vector6 expected = t.adjoint().transpose().inverse() * f.vector();
    EXPECT_LE(((t * f).vector() - expected).cwiseAbs().maxCoeff(), 1e-10);
  }
}

TEST_F(MotionForceTest, Se3ForceRoundTrip) {
  for (int i = 0; i < 20; ++i) {
    const SE3 t = random_se3();
    const Force f = random_force();
    const Force round = t * (t.inverse() * f);
    EXPECT_LE((round.vector() - f.vector()).cwiseAbs().maxCoeff(), 1e-10);
  }
}

TEST_F(MotionForceTest, DualityPower) {
  // Power τ·v is frame-invariant: (Ad_T m) · (Ad_T^{-T} f) = m · f.
  for (int i = 0; i < 20; ++i) {
    const SE3 t = random_se3();
    const Motion m = random_motion();
    const Force f = random_force();
    const Scalar p_world = (t * m).vector().dot((t * f).vector());
    const Scalar p_local = m.vector().dot(f.vector());
    EXPECT_NEAR(p_world, p_local, 1e-10);
  }
}

}  // namespace
}  // namespace tinyspatial
