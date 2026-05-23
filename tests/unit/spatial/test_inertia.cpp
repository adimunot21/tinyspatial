#include <random>

#include "tinyspatial/spatial/inertia.hpp"
#include "tinyspatial/spatial/plucker.hpp"

#include <gtest/gtest.h>

namespace tinyspatial {
namespace {

constexpr Scalar kTight = 1e-12;

class InertiaTest : public ::testing::Test {
 protected:
  std::mt19937 gen_{29};
  std::uniform_real_distribution<Scalar> uni_{-1.0, 1.0};
  std::uniform_real_distribution<Scalar> pos_{0.1, 2.0};

  Vector3 random_vec() { return Vector3(uni_(gen_), uni_(gen_), uni_(gen_)); }

  // A non-degenerate symmetric positive-definite 3×3 inertia.
  Matrix3 random_inertia_psd() {
    Matrix3 a;
    a << pos_(gen_), uni_(gen_) * 0.1, uni_(gen_) * 0.1, uni_(gen_) * 0.1, pos_(gen_),
        uni_(gen_) * 0.1, uni_(gen_) * 0.1, uni_(gen_) * 0.1, pos_(gen_);
    // Symmetrise: a + aᵀ keeps it symmetric; with diagonal-dominant values it stays PSD.
    return (a + a.transpose()) * 0.5;
  }

  SpatialInertia random_inertia() {
    return SpatialInertia(pos_(gen_), random_vec(), random_inertia_psd());
  }

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

TEST(InertiaBasics, PointMassAtOrigin) {
  // A point mass at the body-frame origin has Ī_com = 0; full matrix is m·I_6
  // restricted to the linear block, zeros elsewhere.
  const SpatialInertia i(2.5, Vector3::Zero(), Matrix3::Zero());
  Matrix6 expected = Matrix6::Zero();
  expected.bottomRightCorner<3, 3>() = 2.5 * Matrix3::Identity();
  EXPECT_TRUE(i.matrix6().isApprox(expected));
}

TEST(InertiaBasics, PointMassOffOrigin) {
  // A point mass m at offset c: Ī_com = 0, so Ī_O = m·(‖c‖² I − c cᵀ).
  const Scalar m = 3.0;
  const Vector3 c(1.0, 0.0, 0.0);  // mass on the +x axis
  const SpatialInertia i(m, c, Matrix3::Zero());
  // The "inertia about origin" for a unit point mass at (1,0,0) about origin
  // is diag(0, 1, 1) — only spinning about y or z costs energy.
  Matrix3 expected_io;
  expected_io << 0, 0, 0, 0, m, 0, 0, 0, m;
  EXPECT_TRUE(i.inertia_origin().isApprox(expected_io));
}

TEST(InertiaBasics, SolidSphereAtOrigin) {
  // Solid sphere of mass m, radius r about its centre: Ī = (2/5) m r² I.
  // Place at origin so Ī_O = Ī_com and the matrix is block-diagonal.
  const Scalar m = 4.0;
  const Scalar r = 0.5;
  const Matrix3 i_com = (2.0 / 5.0) * m * r * r * Matrix3::Identity();
  const SpatialInertia inertia(m, Vector3::Zero(), i_com);

  Matrix6 expected = Matrix6::Zero();
  expected.topLeftCorner<3, 3>() = i_com;
  expected.bottomRightCorner<3, 3>() = m * Matrix3::Identity();
  EXPECT_TRUE(inertia.matrix6().isApprox(expected));
}

TEST_F(InertiaTest, Matrix6IsSymmetric) {
  for (int i = 0; i < 20; ++i) {
    const Matrix6 m = random_inertia().matrix6();
    EXPECT_LE((m - m.transpose()).cwiseAbs().maxCoeff(), kTight) << m;
  }
}

TEST_F(InertiaTest, KineticEnergyFormula) {
  // 2·KE = vᵀ I v should match m‖v_com‖² + ωᵀ Ī_com ω, where v_com = v + ω × c.
  for (int i = 0; i < 20; ++i) {
    const SpatialInertia inertia = random_inertia();
    const Vector3 omega = random_vec();
    const Vector3 v = random_vec();
    const Motion m(omega, v);

    const Scalar quadratic = m.vector().dot(inertia.matrix6() * m.vector());

    const Vector3 v_com = v + omega.cross(inertia.com());
    const Scalar expected =
        inertia.mass() * v_com.squaredNorm() + omega.dot(inertia.inertia_com() * omega);

    EXPECT_NEAR(quadratic, expected, 1e-10);
  }
}

TEST_F(InertiaTest, InertiaTimesMotionIsForce) {
  for (int i = 0; i < 20; ++i) {
    const SpatialInertia inertia = random_inertia();
    const Motion m(random_vec(), random_vec());
    const Force f = inertia * m;
    EXPECT_TRUE(f.vector().isApprox(inertia.matrix6() * m.vector()));
  }
}

TEST_F(InertiaTest, Se3TransformMatchesCongruence) {
  // Separable transform of I must agree with the 6×6 congruence X⁻ᵀ I X⁻¹.
  for (int i = 0; i < 20; ++i) {
    const SpatialInertia inertia = random_inertia();
    const SE3 t = random_se3();
    const Matrix6 x_inv = motion_plucker(t.inverse());
    const Matrix6 congruence = x_inv.transpose() * inertia.matrix6() * x_inv;
    const Matrix6 separable = (t * inertia).matrix6();
    EXPECT_LE((separable - congruence).cwiseAbs().maxCoeff(), 1e-9) << "diff=\n"
                                                                    << (separable - congruence);
  }
}

TEST_F(InertiaTest, CompositeInertiaMatches6x6Sum) {
  // Adding two inertias in the same frame: the matrix6 of the sum must equal
  // the sum of the matrix6s (linearity in the 6×6 representation).
  for (int i = 0; i < 20; ++i) {
    const SpatialInertia a = random_inertia();
    const SpatialInertia b = random_inertia();
    const Matrix6 sum_then_mat = (a + b).matrix6();
    const Matrix6 mat_then_sum = a.matrix6() + b.matrix6();
    EXPECT_LE((sum_then_mat - mat_then_sum).cwiseAbs().maxCoeff(), 1e-12);
  }
}

TEST_F(InertiaTest, Se3InertiaRoundTrip) {
  for (int i = 0; i < 20; ++i) {
    const SpatialInertia inertia = random_inertia();
    const SE3 t = random_se3();
    const SpatialInertia round = t.inverse() * (t * inertia);
    EXPECT_LE((round.matrix6() - inertia.matrix6()).cwiseAbs().maxCoeff(), 1e-10);
  }
}

TEST_F(InertiaTest, PluckerOfUnitTwistMatchesAdjoint) {
  // Plan-stated test: Plücker transform of a unit twist matches Ad_T · e_i.
  for (int trial = 0; trial < 5; ++trial) {
    const SE3 t = random_se3();
    const Matrix6 x = motion_plucker(t);
    const Matrix6 ad = t.adjoint();
    for (int i = 0; i < 6; ++i) {
      Vector6 e_i = Vector6::Zero();
      e_i(i) = 1.0;
      EXPECT_TRUE((x * e_i).isApprox(ad * e_i));
    }
  }
}

TEST_F(InertiaTest, ForcePluckerIsDualAdjoint) {
  for (int i = 0; i < 10; ++i) {
    const SE3 t = random_se3();
    const Matrix6 xs = force_plucker(t);
    const Matrix6 expected = t.adjoint().transpose().inverse();
    EXPECT_LE((xs - expected).cwiseAbs().maxCoeff(), 1e-10);
  }
}

}  // namespace
}  // namespace tinyspatial
