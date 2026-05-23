#include <random>

#include "tinyspatial/liegroup/se3.hpp"
#include "tinyspatial/spatial/cross.hpp"

#include <gtest/gtest.h>

namespace tinyspatial {
namespace {

template <typename A, typename B>
void expect_near(const A& a, const B& b, Scalar tol) {
  EXPECT_LE((a - b).cwiseAbs().maxCoeff(), tol) << "a=\n" << a << "\nb=\n" << b;
}

class CrossTest : public ::testing::Test {
 protected:
  std::mt19937 gen_{11};
  std::uniform_real_distribution<Scalar> uni_{-1.0, 1.0};
  Vector6 random_motion() {
    Vector6 m;
    for (int i = 0; i < 6; ++i) {
      m(i) = uni_(gen_);
    }
    return m;
  }
};

TEST_F(CrossTest, ForceIsNegativeTransposeOfMotion) {
  for (int i = 0; i < 20; ++i) {
    const Vector6 m = random_motion();
    expect_near(cross_force(m), Matrix6(-cross_motion(m).transpose()), 1e-15);
  }
}

TEST_F(CrossTest, SelfCrossIsZero) {
  for (int i = 0; i < 20; ++i) {
    const Vector6 m = random_motion();
    EXPECT_NEAR((cross_motion(m) * m).cwiseAbs().maxCoeff(), 0.0, 1e-14);
  }
}

TEST_F(CrossTest, JacobiIdentity) {
  // ad is a Lie-algebra homomorphism: ad_{[a,b]} = [ad_a, ad_b].
  for (int i = 0; i < 20; ++i) {
    const Vector6 a = random_motion();
    const Vector6 b = random_motion();
    const Vector6 bracket = cross_motion(a) * b;  // [a, b]
    const Matrix6 lhs = cross_motion(bracket);
    const Matrix6 rhs = cross_motion(a) * cross_motion(b) - cross_motion(b) * cross_motion(a);
    expect_near(lhs, rhs, 1e-13);
  }
}

TEST_F(CrossTest, CrossMotionIsAdjointDerivative) {
  // d/dt Ad(exp(t·m))|₀ == cross_motion(m). Central difference vs the SE(3)
  // adjoint links cross.hpp and SE3::adjoint().
  const Scalar h = 1e-6;
  for (int i = 0; i < 20; ++i) {
    const Vector6 m = random_motion();
    const Matrix6 fd =
        (SE3::exp(Vector6(h * m)).adjoint() - SE3::exp(Vector6(-h * m)).adjoint()) / (2 * h);
    expect_near(fd, cross_motion(m), 1e-6);
  }
}

}  // namespace
}  // namespace tinyspatial
