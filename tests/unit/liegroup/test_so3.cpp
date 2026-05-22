#include <cmath>
#include <random>

#include "tinyspatial/liegroup/so3.hpp"

#include <gtest/gtest.h>

namespace tinyspatial {
namespace {

constexpr Scalar kPi = 3.14159265358979323846;
constexpr Scalar kTight = 1e-12;  // exp/log round-trips
constexpr Scalar kFd = 1e-7;      // finite-difference Jacobian agreement

void ExpectMatrixNear(const Matrix3& a, const Matrix3& b, Scalar tol) {
  EXPECT_LE((a - b).cwiseAbs().maxCoeff(), tol) << "a=\n" << a << "\nb=\n" << b;
}

// A reproducible source of random rotation vectors with bounded angle.
class So3Test : public ::testing::Test {
 protected:
  std::mt19937 gen_{42};
  std::uniform_real_distribution<Scalar> uni_{-1.0, 1.0};

  Vector3 RandomAxis() {
    Vector3 v(uni_(gen_), uni_(gen_), uni_(gen_));
    while (v.norm() < 1e-6) {
      v = Vector3(uni_(gen_), uni_(gen_), uni_(gen_));
    }
    return v.normalized();
  }
  Vector3 RandomOmega(Scalar max_angle) {
    const Scalar angle = (uni_(gen_) * 0.5 + 0.5) * max_angle;
    return RandomAxis() * angle;
  }

  // ∂/∂δ log(exp(ω)⁻¹ · exp(ω+δ)) |_{δ=0}  ==  right Jacobian.
  Matrix3 NumericalRightJacobian(const Vector3& w) {
    const Scalar h = 1e-5;
    const SO3 r_inv = SO3::exp(w).inverse();
    Matrix3 j;
    for (int col = 0; col < 3; ++col) {
      Vector3 d = Vector3::Zero();
      d(col) = h;
      const Vector3 fp = (r_inv * SO3::exp(w + d)).log();
      const Vector3 fm = (r_inv * SO3::exp(w - d)).log();
      j.col(col) = (fp - fm) / (2 * h);
    }
    return j;
  }

  // ∂/∂δ log(exp(ω+δ) · exp(ω)⁻¹) |_{δ=0}  ==  left Jacobian.
  Matrix3 NumericalLeftJacobian(const Vector3& w) {
    const Scalar h = 1e-5;
    const SO3 r_inv = SO3::exp(w).inverse();
    Matrix3 j;
    for (int col = 0; col < 3; ++col) {
      Vector3 d = Vector3::Zero();
      d(col) = h;
      const Vector3 fp = (SO3::exp(w + d) * r_inv).log();
      const Vector3 fm = (SO3::exp(w - d) * r_inv).log();
      j.col(col) = (fp - fm) / (2 * h);
    }
    return j;
  }
};

TEST(So3Hat, SkewUnskewRoundTrip) {
  const Vector3 v(1.0, -2.0, 3.0);
  EXPECT_TRUE(skew(v).isApprox(-skew(v).transpose()));  // antisymmetric
  EXPECT_TRUE(unskew(skew(v)).isApprox(v));
  // [v]_× w == v × w
  const Vector3 w(0.5, 0.25, -1.0);
  EXPECT_TRUE((skew(v) * w).isApprox(v.cross(w)));
}

TEST(So3Basics, IdentityAndAccessors) {
  const SO3 id = SO3::identity();
  ExpectMatrixNear(id.matrix(), Matrix3::Identity(), kTight);
  EXPECT_NEAR(id.log().norm(), 0.0, kTight);
}

TEST(So3Basics, ActMatchesMatrix) {
  const SO3 r = SO3::exp(Vector3(0.3, -0.7, 1.1));
  const Vector3 p(2.0, -1.0, 0.5);
  EXPECT_TRUE(r.act(p).isApprox(r.matrix() * p));
}

TEST_F(So3Test, ExpMatchesAngleAxis) {
  for (int i = 0; i < 50; ++i) {
    const Vector3 axis = RandomAxis();
    const Scalar angle = uni_(gen_) * kPi;  // (-π, π)
    const Matrix3 expected = Eigen::AngleAxis<Scalar>(angle, axis).toRotationMatrix();
    ExpectMatrixNear(SO3::exp(axis * angle).matrix(), expected, 1e-12);
  }
}

TEST_F(So3Test, LogExpRoundTrip) {
  for (int i = 0; i < 200; ++i) {
    const Vector3 w = RandomOmega(kPi - 1e-6);  // unique chart away from π
    const Vector3 round = SO3::exp(w).log();
    EXPECT_LE((round - w).norm(), kTight) << "w=" << w.transpose();
  }
}

TEST_F(So3Test, ExpLogRoundTrip) {
  for (int i = 0; i < 200; ++i) {
    const SO3 r = SO3::exp(RandomOmega(kPi));
    ExpectMatrixNear(SO3::exp(r.log()).matrix(), r.matrix(), kTight);
  }
}

TEST(So3Corner, SmallAngle) {
  const Vector3 w(1e-10, -2e-10, 5e-11);
  EXPECT_LE((SO3::exp(w).log() - w).norm(), kTight);
  ExpectMatrixNear(SO3::right_jacobian(w), Matrix3::Identity(), 1e-9);
  ExpectMatrixNear(SO3::left_jacobian(w), Matrix3::Identity(), 1e-9);
}

TEST(So3Corner, PiAngle) {
  // The hardest case for log: a rotation of exactly π.
  const Vector3 axes[] = {Vector3::UnitX(), Vector3::UnitY(), Vector3::UnitZ(),
                          Vector3(1, 1, 1).normalized()};
  for (const Vector3& axis : axes) {
    const SO3 r = SO3::exp(axis * kPi);
    EXPECT_NEAR(r.log().norm(), kPi, 1e-9);
    ExpectMatrixNear(SO3::exp(r.log()).matrix(), r.matrix(), 1e-9);
  }
}

TEST_F(So3Test, GroupAxioms) {
  for (int i = 0; i < 50; ++i) {
    const SO3 a = SO3::exp(RandomOmega(kPi));
    const SO3 b = SO3::exp(RandomOmega(kPi));
    const SO3 c = SO3::exp(RandomOmega(kPi));
    // Associativity.
    ExpectMatrixNear(((a * b) * c).matrix(), (a * (b * c)).matrix(), 1e-12);
    // Identity.
    ExpectMatrixNear((a * SO3::identity()).matrix(), a.matrix(), kTight);
    // Inverse.
    ExpectMatrixNear((a * a.inverse()).matrix(), Matrix3::Identity(), 1e-12);
  }
}

TEST_F(So3Test, JacobiansMatchFiniteDifference) {
  for (int i = 0; i < 50; ++i) {
    const Vector3 w = RandomOmega(2.5);  // well inside the chart
    ExpectMatrixNear(SO3::right_jacobian(w), NumericalRightJacobian(w), kFd);
    ExpectMatrixNear(SO3::left_jacobian(w), NumericalLeftJacobian(w), kFd);
  }
}

TEST_F(So3Test, JacobianInverses) {
  for (int i = 0; i < 50; ++i) {
    const Vector3 w = RandomOmega(2.5);
    ExpectMatrixNear(SO3::right_jacobian(w) * SO3::right_jacobian_inverse(w), Matrix3::Identity(),
                     1e-10);
    ExpectMatrixNear(SO3::left_jacobian(w) * SO3::left_jacobian_inverse(w), Matrix3::Identity(),
                     1e-10);
    // Identity Jl(ω) = Jr(ω)ᵀ.
    ExpectMatrixNear(SO3::left_jacobian(w), SO3::right_jacobian(w).transpose(), 1e-12);
  }
}

}  // namespace
}  // namespace tinyspatial
