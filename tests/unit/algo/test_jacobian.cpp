#include <filesystem>
#include <random>
#include <string>

#include "tinyspatial/algo/forward_kinematics.hpp"
#include "tinyspatial/algo/jacobian.hpp"
#include "tinyspatial/urdf/urdf_loader.hpp"

#include <gtest/gtest.h>

namespace tinyspatial {
namespace {

constexpr const char* kFixtureDir = TINYSPATIAL_TEST_FIXTURE_DIR;

std::string fixture(const std::string& name) {
  return (std::filesystem::path(kFixtureDir) / name).string();
}

// Central finite-difference Jacobian: column j is (g(q + h e_j) - g(q - h e_j)) / (2h),
// where g(q) is a 6-vector observation of the link's pose at configuration q.
template <typename Observer>
Matrix6X finite_difference_jacobian(const Model& m, int link_id, const VectorX& q, Observer g) {
  constexpr Scalar kH = 1e-6;
  Matrix6X j(6, m.nv());
  j.setZero();
  Data d(m);
  for (int col = 0; col < m.nv(); ++col) {
    VectorX dq = VectorX::Zero(m.nv());
    dq(col) = kH;
    // For revolute / prismatic / floating velocity slots, q and v have the
    // same shape on these fixtures (no quaternion integration is exercised by
    // these robots), so q_plus = q + dq is correct.
    VectorX q_plus = q + dq;
    VectorX q_minus = q - dq;
    forward_kinematics(m, d, q_plus);
    const SE3 t_plus = d.pose_in_world[link_id];
    const Vector6 g_plus = g(t_plus);
    forward_kinematics(m, d, q_minus);
    const SE3 t_minus = d.pose_in_world[link_id];
    const Vector6 g_minus = g(t_minus);
    j.col(col) = (g_plus - g_minus) / (2 * kH);
  }
  return j;
}

class JacobianFdTest : public ::testing::TestWithParam<std::string> {};

TEST_P(JacobianFdTest, LocalAgreesWithFiniteDifference) {
  // J_local(q) · δ  ≈  log( T(q)^{-1} · T(q+δ) )
  // FD: per column, (log(T(q)^{-1} · T(q+h e_j)) - log(T(q)^{-1} · T(q-h e_j))) / 2h.
  const Model m = build_model_from_urdf_file(fixture(GetParam()));
  Data d(m);
  std::mt19937 gen(43);
  std::uniform_real_distribution<Scalar> uni(-0.8, 0.8);

  VectorX q(m.nq());
  for (int i = 0; i < m.nq(); ++i) {
    q(i) = uni(gen);
  }

  const int link_id = m.njoints() - 1;
  forward_kinematics(m, d, q);
  const SE3 t_q_inv = d.pose_in_world[link_id].inverse();

  Matrix6X j_analytic(6, m.nv());
  compute_jacobian(m, d, link_id, j_analytic, JacobianFrame::kLocal);

  const Matrix6X j_fd = finite_difference_jacobian(
      m, link_id, q, [&t_q_inv](const SE3& t) -> Vector6 { return (t_q_inv * t).log(); });

  EXPECT_LE((j_analytic - j_fd).cwiseAbs().maxCoeff(), 1e-6)
      << "robot=" << GetParam() << "\nanalytic=\n"
      << j_analytic << "\nfd=\n"
      << j_fd;
}

TEST_P(JacobianFdTest, WorldAgreesWithFiniteDifference) {
  // J_world(q) · δ  ≈  log( T(q+δ) · T(q)^{-1} )
  const Model m = build_model_from_urdf_file(fixture(GetParam()));
  Data d(m);
  std::mt19937 gen(44);
  std::uniform_real_distribution<Scalar> uni(-0.8, 0.8);

  VectorX q(m.nq());
  for (int i = 0; i < m.nq(); ++i) {
    q(i) = uni(gen);
  }
  const int link_id = m.njoints() - 1;
  forward_kinematics(m, d, q);
  const SE3 t_q = d.pose_in_world[link_id];

  Matrix6X j_analytic(6, m.nv());
  compute_jacobian(m, d, link_id, j_analytic, JacobianFrame::kWorld);

  const Matrix6X j_fd = finite_difference_jacobian(
      m, link_id, q, [&t_q](const SE3& t) -> Vector6 { return (t * t_q.inverse()).log(); });

  EXPECT_LE((j_analytic - j_fd).cwiseAbs().maxCoeff(), 1e-6) << "robot=" << GetParam();
}

TEST_P(JacobianFdTest, LocalWorldAlignedMatchesRotatedLocal) {
  // J_lwa = [[R, 0], [0, R]] · J_local, where R is L's world rotation.
  const Model m = build_model_from_urdf_file(fixture(GetParam()));
  Data d(m);
  std::mt19937 gen(45);
  std::uniform_real_distribution<Scalar> uni(-0.8, 0.8);

  VectorX q(m.nq());
  for (int i = 0; i < m.nq(); ++i) {
    q(i) = uni(gen);
  }
  const int link_id = m.njoints() - 1;
  forward_kinematics(m, d, q);

  Matrix6X j_local(6, m.nv());
  Matrix6X j_lwa(6, m.nv());
  compute_jacobian(m, d, link_id, j_local, JacobianFrame::kLocal);
  compute_jacobian(m, d, link_id, j_lwa, JacobianFrame::kLocalWorldAligned);

  const Matrix3 r = d.pose_in_world[link_id].rotation().matrix();
  Matrix6 lwa_ad = Matrix6::Zero();
  lwa_ad.topLeftCorner<3, 3>() = r;
  lwa_ad.bottomRightCorner<3, 3>() = r;
  EXPECT_LE((j_lwa - lwa_ad * j_local).cwiseAbs().maxCoeff(), 1e-12);
}

INSTANTIATE_TEST_SUITE_P(AllFixtures, JacobianFdTest,
                         ::testing::Values("simple_arm.urdf", "franka_fr3.urdf", "ur5e.urdf",
                                           "so_arm101.urdf"));

TEST(JacobianShape, IsZeroOnNonAncestorJoints) {
  // For a 2-DOF serial chain, the Jacobian at link 0 has only one nonzero
  // column (joint 0's). Joint 1's column is zero — joint 1 doesn't move link 0.
  const Model m = build_model_from_urdf_file(fixture("simple_arm.urdf"));
  Data d(m);
  VectorX q(2);
  q << 0.5, 1.0;
  forward_kinematics(m, d, q);

  Matrix6X j(6, m.nv());
  compute_jacobian(m, d, /*link_id=*/0, j, JacobianFrame::kLocal);

  EXPECT_GT(j.col(0).cwiseAbs().maxCoeff(), 0.5);
  EXPECT_NEAR(j.col(1).cwiseAbs().maxCoeff(), 0.0, 1e-15);
}

}  // namespace
}  // namespace tinyspatial
