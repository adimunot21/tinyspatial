#include <filesystem>
#include <random>
#include <string>

#include "tinyspatial/algo/forward_kinematics.hpp"
#include "tinyspatial/ik/dls.hpp"
#include "tinyspatial/urdf/urdf_loader.hpp"

#include <gtest/gtest.h>

namespace tinyspatial {
namespace {

constexpr const char* kFixtureDir = TINYSPATIAL_TEST_FIXTURE_DIR;

std::string fixture(const std::string& name) {
  return (std::filesystem::path(kFixtureDir) / name).string();
}

// Random configuration in [-1.2, 1.2] on every coordinate. Works for our
// fixtures (all revolute joints with comfortable joint limits); not safe
// for floating-base joints (would need quaternion normalisation).
VectorX random_config(const Model& m, std::mt19937& gen) {
  std::uniform_real_distribution<Scalar> uni(-1.2, 1.2);
  VectorX q(m.nq());
  for (int i = 0; i < m.nq(); ++i) {
    q(i) = uni(gen);
  }
  return q;
}

// Pick the last movable joint in the topological order — the end-effector
// proxy for our fixture URDFs (joint indexing is universe-relative, so the
// last index is the deepest body in the chain).
int last_joint(const Model& m) {
  return m.njoints() - 1;
}

class DlsTest : public ::testing::TestWithParam<std::string> {};

TEST_P(DlsTest, ConvergesFromRandomSeed) {
  // For a target that is the FK of a known random configuration, DLS should
  // converge to *some* configuration that lands on the same target (not
  // necessarily q_target — IK is non-unique).
  //
  // Single-seed constant-damping DLS does not converge from every random
  // seed: ~14% of seeds on a 6-DoF arm (UR5e) end up in basins that
  // don't reach the target (Buss & Kim 2005; this is intrinsic to DLS, not
  // a bug). The 80/100 bar reflects realistic single-seed behaviour;
  // production callers should add random-restart wrappers.
  const Model m = build_model_from_urdf_file(fixture(GetParam()));
  Data d(m);
  std::mt19937 gen(31337);

  const int link = last_joint(m);
  int converged = 0;
  constexpr int kNumTrials = 100;
  for (int trial = 0; trial < kNumTrials; ++trial) {
    const VectorX q_target = random_config(m, gen);
    forward_kinematics(m, d, q_target);
    const SE3 target = d.pose_in_world[link];

    const VectorX q_init = random_config(m, gen);
    DlsOptions opts;
    opts.max_iters = 500;
    const IkResult res = solve_ik_dls(m, d, link, target, q_init, opts);

    if (res.converged) {
      // Verify the converged q actually puts the link at the target.
      forward_kinematics(m, d, res.q);
      const SE3 reached = d.pose_in_world[link];
      const Vector6 final_err = (reached.inverse() * target).log();
      EXPECT_LE(final_err.cwiseAbs().maxCoeff(), 1e-5)
          << "trial " << trial << " reports converged but error too large";
      ++converged;
    }
  }
  EXPECT_GE(converged, 80) << "DLS converged on only " << converged << "/" << kNumTrials
                           << " random seeds (expected ≥ 80)";
}

TEST_P(DlsTest, IdentitySeedZeroError) {
  // If the seed already places the link at the target, IK should converge
  // in 0 or 1 iterations with near-zero error.
  const Model m = build_model_from_urdf_file(fixture(GetParam()));
  Data d(m);
  std::mt19937 gen(7);
  const VectorX q_target = random_config(m, gen);
  forward_kinematics(m, d, q_target);
  const int link = last_joint(m);
  const SE3 target = d.pose_in_world[link];

  const IkResult res = solve_ik_dls(m, d, link, target, q_target);
  EXPECT_TRUE(res.converged);
  EXPECT_LE(res.iterations, 1);
  EXPECT_LE(res.error.cwiseAbs().maxCoeff(), 1e-6);
}

TEST_P(DlsTest, RespectsMaxIters) {
  // With max_iters = 0, the solver returns immediately without modifying q.
  const Model m = build_model_from_urdf_file(fixture(GetParam()));
  Data d(m);
  std::mt19937 gen(99);
  const VectorX q_init = random_config(m, gen);
  forward_kinematics(m, d, q_init);
  const int link = last_joint(m);

  // Pick a target unlikely to be already satisfied.
  const VectorX q_other = random_config(m, gen);
  forward_kinematics(m, d, q_other);
  const SE3 target = d.pose_in_world[link];

  DlsOptions opts;
  opts.max_iters = 0;
  const IkResult res = solve_ik_dls(m, d, link, target, q_init, opts);
  EXPECT_FALSE(res.converged);
  EXPECT_EQ(res.iterations, 0);
  EXPECT_TRUE((res.q - q_init).cwiseAbs().maxCoeff() == 0.0);
}

INSTANTIATE_TEST_SUITE_P(AllFixtures, DlsTest,
                         ::testing::Values("simple_arm.urdf", "franka_fr3.urdf", "ur5e.urdf",
                                           "so_arm101.urdf"));

}  // namespace
}  // namespace tinyspatial
