#include <filesystem>
#include <random>
#include <string>

#include "tinyspatial/algo/forward_kinematics.hpp"
#include "tinyspatial/algo/jacobian.hpp"
#include "tinyspatial/diff/fk_derivatives.hpp"
#include "tinyspatial/urdf/urdf_loader.hpp"

#include <gtest/gtest.h>

namespace tinyspatial {
namespace {

constexpr const char* kFixtureDir = TINYSPATIAL_TEST_FIXTURE_DIR;

std::string fixture(const std::string& name) {
  return (std::filesystem::path(kFixtureDir) / name).string();
}

class FkDerivativesTest : public ::testing::TestWithParam<std::string> {};

TEST_P(FkDerivativesTest, MatchesSingleJointJacobian) {
  // The collective compute_joint_jacobians() must agree, joint by joint, with
  // the existing single-joint compute_jacobian() in LOCAL frame.
  const Model m = build_model_from_urdf_file(fixture(GetParam()));
  Data d(m);
  std::mt19937 gen(101);
  std::uniform_real_distribution<Scalar> uni(-1.5, 1.5);
  for (int trial = 0; trial < 5; ++trial) {
    VectorX q(m.nq());
    for (int i = 0; i < m.nq(); ++i) {
      q(i) = uni(gen);
    }
    std::vector<Matrix6X> j_all;
    compute_joint_jacobians(m, d, q, j_all);

    forward_kinematics(m, d, q);
    Matrix6X j_single(6, m.nv());
    for (int i = 0; i < m.njoints(); ++i) {
      compute_jacobian(m, d, i, j_single, JacobianFrame::kLocal);
      const Scalar diff = (j_all[i] - j_single).cwiseAbs().maxCoeff();
      EXPECT_LE(diff, 1e-12) << "joint " << i << " disagrees with single-joint Jacobian";
    }
  }
}

TEST_P(FkDerivativesTest, AgreesWithCentralFiniteDifference) {
  // Central FD on the world pose, mapped back to a body-frame twist via the
  // Lie log: J_i.col(k) ≈ log(oMi(q-ε)⁻¹ · oMi(q+ε)) / (2ε).
  const Model m = build_model_from_urdf_file(fixture(GetParam()));
  Data d_center(m);
  Data d_plus(m);
  Data d_minus(m);

  std::mt19937 gen(202);
  std::uniform_real_distribution<Scalar> uni(-1.0, 1.0);
  VectorX q(m.nq());
  for (int i = 0; i < m.nq(); ++i) {
    q(i) = uni(gen);
  }

  std::vector<Matrix6X> j_all;
  compute_joint_jacobians(m, d_center, q, j_all);

  constexpr Scalar kEps = 1e-6;
  for (int k = 0; k < m.nv(); ++k) {
    // For our fixtures nv == nq (all revolute joints). Floating joints would
    // need the joint's own exp-map here; not present in the fixtures.
    VectorX q_plus = q;
    VectorX q_minus = q;
    q_plus(k) += kEps;
    q_minus(k) -= kEps;
    forward_kinematics(m, d_plus, q_plus);
    forward_kinematics(m, d_minus, q_minus);

    for (int i = 0; i < m.njoints(); ++i) {
      const SE3 relative = d_minus.pose_in_world[i].inverse() * d_plus.pose_in_world[i];
      const Vector6 fd_col = relative.log() / (2.0 * kEps);
      const Vector6 analytic_col = j_all[i].col(k);
      const Scalar diff = (fd_col - analytic_col).cwiseAbs().maxCoeff();
      EXPECT_LE(diff, 1e-7) << "joint " << i << " col " << k << " disagrees with FD";
    }
  }
}

INSTANTIATE_TEST_SUITE_P(AllFixtures, FkDerivativesTest,
                         ::testing::Values("simple_arm.urdf", "franka_fr3.urdf", "ur5e.urdf",
                                           "so_arm101.urdf"));

}  // namespace
}  // namespace tinyspatial
