/// Differentiable (Jet) instantiation of the Lie groups (Phase 2, P2.1).
///
/// Proves that SO3T / SE3T run on the forward-mode autodiff scalar and produce
/// correct gradients through exp / log — i.e. through sqrt, asin, sin, cos, the
/// min-clamp, the quaternion storage, and the small-angle branch. Truths used:
///   - log(exp(ξ)) is the identity map, so its Jacobian is I.
///   - d(exp(ω)·p)/dω |_{ω=0} = -[p]_×  (the body-frame derivative at identity).
#include <cmath>

#include <Eigen/Geometry>
#include <gtest/gtest.h>

#include "tinyspatial/core/jet.hpp"
#include "tinyspatial/liegroup/se3.hpp"
#include "tinyspatial/liegroup/so3.hpp"

namespace {

using tinyspatial::Jet;
using tinyspatial::SE3T;
using tinyspatial::SO3T;

// log(exp(ω)) == ω, and its Jacobian w.r.t. ω is the 3×3 identity.
TEST(LieGroupAd, So3LogExpRoundTripJacobianIsIdentity) {
  using J = Jet<3>;
  Eigen::Matrix<J, 3, 1> omega;
  omega << J(0.3, 0), J(-0.5, 1), J(0.2, 2);  // seed each component

  const SO3T<J> r = SO3T<J>::exp(omega);
  const Eigen::Matrix<J, 3, 1> back = r.log();

  for (int i = 0; i < 3; ++i) {
    EXPECT_NEAR(back[i].a, omega[i].a, 1e-12) << "value round-trip, row " << i;
    for (int j = 0; j < 3; ++j) {
      EXPECT_NEAR(back[i].v[j], i == j ? 1.0 : 0.0, 1e-9) << "d log/d omega (" << i << "," << j << ")";
    }
  }
}

// At ω = 0 (the small-angle branch), d(R(ω)·p)/dω = -[p]_×.
TEST(LieGroupAd, So3ActDerivativeAtIdentity) {
  using J = Jet<3>;
  Eigen::Matrix<J, 3, 1> omega;
  omega << J(0.0, 0), J(0.0, 1), J(0.0, 2);  // seeded, value zero

  const Eigen::Vector3d p_val(0.1, 0.7, -0.4);
  Eigen::Matrix<J, 3, 1> p;
  p << J(p_val.x()), J(p_val.y()), J(p_val.z());  // constant point

  const Eigen::Matrix<J, 3, 1> y = SO3T<J>::exp(omega).act(p);

  Eigen::Matrix3d expected;  // -[p]_×
  expected << 0, p_val.z(), -p_val.y(), -p_val.z(), 0, p_val.x(), p_val.y(), -p_val.x(), 0;
  for (int i = 0; i < 3; ++i) {
    EXPECT_NEAR(y[i].a, p_val[i], 1e-12);  // value unchanged at identity
    for (int j = 0; j < 3; ++j) {
      EXPECT_NEAR(y[i].v[j], expected(i, j), 1e-9) << "(" << i << "," << j << ")";
    }
  }
}

// SE(3): log(exp(ξ)) == ξ with Jacobian = I_6, exercising the full SE3T<Jet>
// exp/log including Barfoot's Q block.
TEST(LieGroupAd, Se3LogExpRoundTripJacobianIsIdentity) {
  using J = Jet<6>;
  const double xi_val[6] = {0.2, -0.3, 0.15, 0.4, 0.1, -0.25};  // (ω; v)
  Eigen::Matrix<J, 6, 1> xi;
  for (int k = 0; k < 6; ++k) {
    xi[k] = J(xi_val[k], k);
  }

  const SE3T<J> t = SE3T<J>::exp(xi);
  const Eigen::Matrix<J, 6, 1> back = t.log();

  for (int i = 0; i < 6; ++i) {
    EXPECT_NEAR(back[i].a, xi_val[i], 1e-12) << "value round-trip, row " << i;
    for (int j = 0; j < 6; ++j) {
      EXPECT_NEAR(back[i].v[j], i == j ? 1.0 : 0.0, 1e-9) << "d log/d xi (" << i << "," << j << ")";
    }
  }
}

}  // namespace
