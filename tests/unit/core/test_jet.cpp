/// Proof-of-concept gate for the Scalar-generic (differentiable) refactor.
///
/// If `Jet` propagates derivatives correctly through plain arithmetic AND
/// through Eigen's matrix and quaternion machinery, then templating the
/// library's algorithms on the scalar is mechanical. The quaternion case is the
/// riskiest Eigen interop (storage, normalise, toRotationMatrix), so it is the
/// real gate.
#include <cmath>

#include "tinyspatial/core/jet.hpp"

#include <Eigen/Geometry>

#include <gtest/gtest.h>

namespace {

using tinyspatial::Jet;

constexpr double kTol = 1e-12;

// f(x) = x^2 + 3x at x = 2  ->  value 10, derivative 2x+3 = 7.
TEST(Jet, PolynomialDerivative) {
  const Jet<1> x(2.0, 0);  // seed the one input
  const Jet<1> f = x * x + 3.0 * x;
  EXPECT_NEAR(f.a, 10.0, kTol);
  EXPECT_NEAR(f.v[0], 7.0, kTol);
}

// Reciprocal: f(x) = 1/x at x = 2 -> value 0.5, derivative -1/x^2 = -0.25.
TEST(Jet, ReciprocalDerivative) {
  const Jet<1> x(2.0, 0);
  const Jet<1> f = 1.0 / x;
  EXPECT_NEAR(f.a, 0.5, kTol);
  EXPECT_NEAR(f.v[0], -0.25, kTol);
}

// Transcendentals with known derivatives at a point.
TEST(Jet, TranscendentalDerivatives) {
  const Jet<1> x(4.0, 0);
  const Jet<1> r = sqrt(x);  // 2, 1/(2*sqrt) = 0.25
  EXPECT_NEAR(r.a, 2.0, kTol);
  EXPECT_NEAR(r.v[0], 0.25, kTol);

  const Jet<1> z(0.0, 0);
  EXPECT_NEAR(sin(z).v[0], 1.0, kTol);   // cos(0)
  EXPECT_NEAR(cos(z).v[0], 0.0, kTol);   // -sin(0)
  EXPECT_NEAR(asin(z).v[0], 1.0, kTol);  // 1/sqrt(1-0)
}

// Two independent inputs: gradient lands in the right slots.
TEST(Jet, PartialsAreIndependent) {
  const Jet<2> x(3.0, 0);
  const Jet<2> y(5.0, 1);
  const Jet<2> f = x * y;  // ∂/∂x = y = 5, ∂/∂y = x = 3
  EXPECT_NEAR(f.a, 15.0, kTol);
  EXPECT_NEAR(f.v[0], 5.0, kTol);
  EXPECT_NEAR(f.v[1], 3.0, kTol);
}

// Eigen matrix arithmetic over Jet compiles and keeps derivatives.
TEST(Jet, EigenMatrixProduct) {
  using J = Jet<1>;
  Eigen::Matrix<J, 2, 2> a;
  Eigen::Matrix<J, 2, 1> x;
  const J s(2.0, 0);  // seeded scalar appearing in the matrix
  a << s, J(1.0), J(0.0), s;
  x << J(3.0), J(4.0);
  const Eigen::Matrix<J, 2, 1> y = a * x;
  // y0 = s*3 + 4 -> value 10, d/ds = 3 ; y1 = s*4 -> value 8, d/ds = 4
  EXPECT_NEAR(y(0).a, 10.0, kTol);
  EXPECT_NEAR(y(0).v[0], 3.0, kTol);
  EXPECT_NEAR(y(1).a, 8.0, kTol);
  EXPECT_NEAR(y(1).v[0], 4.0, kTol);
}

// THE GATE: Eigen::Quaternion<Jet> through normalise + toRotationMatrix, with
// the derivative w.r.t. the rotation angle checked against the analytic dR/dθ.
TEST(Jet, EigenQuaternionRotationDerivative) {
  using J = Jet<1>;
  const double theta = 0.7;
  const J th(theta, 0);  // seed the angle
  const J half = th * 0.5;
  const J w = cos(half);
  const J z = sin(half);

  Eigen::Quaternion<J> q(w, J(0.0), J(0.0), z);  // (w, x, y, z): rotation about z
  q.normalize();
  const Eigen::Matrix<J, 3, 3> r = q.toRotationMatrix();

  // R = [[cosθ, -sinθ, 0], [sinθ, cosθ, 0], [0, 0, 1]]
  EXPECT_NEAR(r(0, 0).a, std::cos(theta), 1e-10);
  EXPECT_NEAR(r(1, 0).a, std::sin(theta), 1e-10);

  // dR/dθ: d cosθ = -sinθ, d sinθ = cosθ. This is the real proof that gradients
  // flow through Eigen's quaternion machinery, not just the values.
  EXPECT_NEAR(r(0, 0).v[0], -std::sin(theta), 1e-10);
  EXPECT_NEAR(r(1, 0).v[0], std::cos(theta), 1e-10);

  // And the result is a valid rotation (values orthonormal).
  const Eigen::Matrix<J, 3, 3> should_be_identity = r.transpose() * r;
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      EXPECT_NEAR(should_be_identity(i, j).a, i == j ? 1.0 : 0.0, 1e-10);
    }
  }
}

}  // namespace
