#include <random>

#include "tinyspatial/liegroup/se3.hpp"

#include <gtest/gtest.h>

namespace tinyspatial {
namespace {

constexpr Scalar kPi = 3.14159265358979323846;
constexpr Scalar kTight = 1e-12;
constexpr Scalar kFd = 1e-6;

template <typename A, typename B>
void expect_near(const A& a, const B& b, Scalar tol) {
  EXPECT_LE((a - b).cwiseAbs().maxCoeff(), tol) << "a=\n" << a << "\nb=\n" << b;
}

class Se3Test : public ::testing::Test {
 protected:
  std::mt19937 gen_{7};
  std::uniform_real_distribution<Scalar> uni_{-1.0, 1.0};

  Vector3 random_vec() { return Vector3(uni_(gen_), uni_(gen_), uni_(gen_)); }

  // Twist with bounded rotation angle so we stay in the exp/log chart.
  Vector6 random_xi(Scalar max_angle) {
    Vector3 axis = random_vec();
    while (axis.norm() < 1e-6) {
      axis = random_vec();
    }
    const Scalar angle = (uni_(gen_) * 0.5 + 0.5) * max_angle;
    Vector6 xi;
    xi.head<3>() = axis.normalized() * angle;
    xi.tail<3>() = random_vec();
    return xi;
  }
  SE3 random_se3() { return SE3::exp(random_xi(kPi - 1e-3)); }

  // ∂/∂δ log(exp(ξ)⁻¹ · exp(ξ+δ)) == right Jacobian.
  static Matrix6 numerical_right_jacobian(const Vector6& xi) {
    const Scalar h = 1e-5;
    const SE3 t_inv = SE3::exp(xi).inverse();
    Matrix6 j;
    for (int col = 0; col < 6; ++col) {
      Vector6 d = Vector6::Zero();
      d(col) = h;
      const Vector6 fp = (t_inv * SE3::exp(Vector6(xi + d))).log();
      const Vector6 fm = (t_inv * SE3::exp(Vector6(xi - d))).log();
      j.col(col) = (fp - fm) / (2 * h);
    }
    return j;
  }

  // ∂/∂δ log(exp(ξ+δ) · exp(ξ)⁻¹) == left Jacobian.
  static Matrix6 numerical_left_jacobian(const Vector6& xi) {
    const Scalar h = 1e-5;
    const SE3 t_inv = SE3::exp(xi).inverse();
    Matrix6 j;
    for (int col = 0; col < 6; ++col) {
      Vector6 d = Vector6::Zero();
      d(col) = h;
      const Vector6 fp = (SE3::exp(Vector6(xi + d)) * t_inv).log();
      const Vector6 fm = (SE3::exp(Vector6(xi - d)) * t_inv).log();
      j.col(col) = (fp - fm) / (2 * h);
    }
    return j;
  }
};

TEST(Se3Basics, IdentityAndMatrix) {
  const SE3 id = SE3::identity();
  expect_near(id.matrix(), Matrix4::Identity(), kTight);

  const SE3 t(SO3::exp(Vector3(0.2, -0.5, 0.9)), Vector3(1.0, 2.0, 3.0));
  const Matrix4 m = t.matrix();
  expect_near(m.topLeftCorner<3, 3>(), t.rotation().matrix(), kTight);
  expect_near(m.topRightCorner<3, 1>(), t.translation(), kTight);
  EXPECT_DOUBLE_EQ(m(3, 3), 1.0);
  const Scalar bottom_row = m.bottomLeftCorner<1, 3>().cwiseAbs().maxCoeff();
  EXPECT_NEAR(bottom_row, 0.0, kTight);
}

TEST(Se3Basics, ActMatchesHomogeneous) {
  const SE3 t(SO3::exp(Vector3(0.3, 0.1, -0.7)), Vector3(-1.0, 0.5, 2.0));
  const Vector3 p(1.0, -2.0, 0.5);
  Eigen::Matrix<Scalar, 4, 1> ph;
  ph << p, 1.0;
  expect_near(t.act(p), (t.matrix() * ph).head<3>().eval(), kTight);
}

TEST_F(Se3Test, GroupAxioms) {
  for (int i = 0; i < 50; ++i) {
    const SE3 a = random_se3();
    const SE3 b = random_se3();
    const SE3 c = random_se3();
    expect_near(((a * b) * c).matrix(), (a * (b * c)).matrix(), 1e-12);
    expect_near((a * SE3::identity()).matrix(), a.matrix(), kTight);
    expect_near((a * a.inverse()).matrix(), Matrix4::Identity(), 1e-12);
  }
}

TEST_F(Se3Test, LogExpRoundTrip) {
  for (int i = 0; i < 200; ++i) {
    const Vector6 xi = random_xi(kPi - 1e-3);
    expect_near(SE3::exp(xi).log(), xi, kTight);
  }
}

TEST_F(Se3Test, ExpLogRoundTrip) {
  for (int i = 0; i < 200; ++i) {
    const SE3 t = random_se3();
    expect_near(SE3::exp(t.log()).matrix(), t.matrix(), kTight);
  }
}

TEST_F(Se3Test, AdjointDefiningIdentity) {
  // exp(Ad(T)·ξ) == T · exp(ξ) · T⁻¹
  for (int i = 0; i < 50; ++i) {
    const SE3 t = random_se3();
    const Vector6 xi = random_xi(1.0);
    const SE3 lhs = SE3::exp(Vector6(t.adjoint() * xi));
    const SE3 rhs = t * SE3::exp(xi) * t.inverse();
    expect_near(lhs.matrix(), rhs.matrix(), 1e-10);
  }
}

TEST_F(Se3Test, AdjointInverse) {
  for (int i = 0; i < 50; ++i) {
    const SE3 t = random_se3();
    expect_near(t.adjoint() * t.adjoint_inverse(), Matrix6::Identity(), 1e-10);
  }
}

TEST_F(Se3Test, JacobiansMatchFiniteDifference) {
  for (int i = 0; i < 50; ++i) {
    const Vector6 xi = random_xi(2.0);
    expect_near(SE3::right_jacobian(xi), numerical_right_jacobian(xi), kFd);
    expect_near(SE3::left_jacobian(xi), numerical_left_jacobian(xi), kFd);
  }
}

TEST_F(Se3Test, JacobianInverses) {
  for (int i = 0; i < 50; ++i) {
    const Vector6 xi = random_xi(2.0);
    expect_near(SE3::left_jacobian(xi) * SE3::left_jacobian_inverse(xi), Matrix6::Identity(), 1e-9);
    expect_near(SE3::right_jacobian(xi) * SE3::right_jacobian_inverse(xi), Matrix6::Identity(),
                1e-9);
  }
}

}  // namespace
}  // namespace tinyspatial
