/// The Phase-2 payoff: forward kinematics is differentiable, and its autodiff
/// Jacobian agrees with the hand-written analytical Jacobian to machine
/// precision.
///
/// `model_cast<Jet>` lifts a URDF-loaded model to the autodiff scalar; seeding
/// `q` as independent `Jet` variables and running the SAME `forward_kinematics`
/// yields ∂(oMi)/∂q. We recover the body-frame Jacobian from each pose's Jet
/// partials and compare it, column by column, against `compute_joint_jacobians`
/// from `diff/fk_derivatives.hpp` (Carpentier-style analytical recursion). Two
/// independently-derived derivative paths agreeing to 1e-10 is the correctness
/// story behind the differentiable-first bet.
#include <filesystem>
#include <random>
#include <string>
#include <vector>

#include <Eigen/Geometry>
#include <gtest/gtest.h>

#include "tinyspatial/algo/forward_kinematics.hpp"
#include "tinyspatial/core/jet.hpp"
#include "tinyspatial/diff/fk_derivatives.hpp"
#include "tinyspatial/urdf/urdf_loader.hpp"

namespace tinyspatial {
namespace {

constexpr const char* kFixtureDir = TINYSPATIAL_TEST_FIXTURE_DIR;

std::string fixture(const std::string& name) {
  return (std::filesystem::path(kFixtureDir) / name).string();
}

// `N` is the robot's nv (compile-time, for the Jet seed width). All fixtures
// used here are all-revolute, so nv == nq == N.
template <int N>
void check_fk_ad(const std::string& robot) {
  const Model m = build_model_from_urdf_file(fixture(robot));
  ASSERT_EQ(m.nv(), N);
  ASSERT_EQ(m.nq(), N);

  std::mt19937 gen(13);
  std::uniform_real_distribution<double> uni(-1.2, 1.2);
  VectorX q(m.nq());
  for (int i = 0; i < m.nq(); ++i) {
    q(i) = uni(gen);
  }

  // Analytical oracle: per-joint body Jacobian (angular-first).
  Data d(m);
  std::vector<Matrix6X> j_oracle;
  compute_joint_jacobians(m, d, q, j_oracle);

  // AD path: the SAME forward_kinematics, on a Jet-lifted model, q seeded as
  // independent autodiff variables.
  using J = Jet<N>;
  const ModelT<J> mj = model_cast<J>(m);
  DataT<J> dj(mj);
  typename Types<J>::VectorX qj(m.nq());
  for (int k = 0; k < N; ++k) {
    qj(k) = J(q(k), k);
  }
  forward_kinematics(mj, dj, qj);

  // Recover J_i.col(k) = vee(oMi^{-1} ∂oMi/∂q_k)
  //                    = ( unskew(Rᵀ ∂R/∂q_k); Rᵀ ∂t/∂q_k ).
  for (int i = 0; i < m.njoints(); ++i) {
    const Eigen::Matrix<J, 3, 3> r_jet = dj.pose_in_world[i].rotation().matrix();
    const Eigen::Matrix<J, 3, 1> t_jet = dj.pose_in_world[i].translation();
    Eigen::Matrix3d r_val;
    for (int a = 0; a < 3; ++a) {
      for (int b = 0; b < 3; ++b) {
        r_val(a, b) = r_jet(a, b).a;
      }
    }

    for (int k = 0; k < N; ++k) {
      Eigen::Matrix3d d_r;
      Eigen::Vector3d d_t;
      for (int a = 0; a < 3; ++a) {
        d_t(a) = t_jet(a).v[k];
        for (int b = 0; b < 3; ++b) {
          d_r(a, b) = r_jet(a, b).v[k];
        }
      }
      const Eigen::Matrix3d w_skew = r_val.transpose() * d_r;  // [ω]_×
      Eigen::Matrix<double, 6, 1> col;
      col << w_skew(2, 1), w_skew(0, 2), w_skew(1, 0), (r_val.transpose() * d_t);
      const double diff = (col - j_oracle[i].col(k)).cwiseAbs().maxCoeff();
      EXPECT_LE(diff, 1e-10) << robot << " joint " << i << " col " << k;
    }
  }
}

TEST(ForwardKinematicsAd, MatchesAnalyticalJacobianFranka) { check_fk_ad<7>("franka_fr3.urdf"); }
TEST(ForwardKinematicsAd, MatchesAnalyticalJacobianUr5e) { check_fk_ad<6>("ur5e.urdf"); }

}  // namespace
}  // namespace tinyspatial
