#include <filesystem>
#include <random>
#include <string>

#include "tinyspatial/algo/forward_kinematics.hpp"
#include "tinyspatial/ik/dls.hpp"
#include "tinyspatial/ik/nullspace.hpp"
#include "tinyspatial/urdf/urdf_loader.hpp"

#include <gtest/gtest.h>

namespace tinyspatial {
namespace {

constexpr const char* kFixtureDir = TINYSPATIAL_TEST_FIXTURE_DIR;

std::string fixture(const std::string& name) {
  return (std::filesystem::path(kFixtureDir) / name).string();
}

VectorX random_config(const Model& m, std::mt19937& gen) {
  std::uniform_real_distribution<Scalar> uni(-1.2, 1.2);
  VectorX q(m.nq());
  for (int i = 0; i < m.nq(); ++i) {
    q(i) = uni(gen);
  }
  return q;
}

TEST(NullspaceTracksPrimary, FrankaConverges) {
  // Even with a strong secondary gain, the primary task must still converge.
  const Model m = build_model_from_urdf_file(fixture("franka_fr3.urdf"));
  Data d(m);
  std::mt19937 gen(2027);

  const int link = m.njoints() - 1;
  const VectorX q_target = random_config(m, gen);
  forward_kinematics(m, d, q_target);
  const SE3 target = d.pose_in_world[link];

  const VectorX q_init = random_config(m, gen);
  const VectorX q_rest = VectorX::Zero(m.nq());  // attract to zero configuration

  NullspaceOptions opts;
  opts.max_iters = 500;
  opts.secondary_gain = 0.5;
  const IkResult res = solve_ik_nullspace(m, d, link, target, q_init, q_rest, opts);

  ASSERT_TRUE(res.converged) << "primary task failed to converge";
  forward_kinematics(m, d, res.q);
  const SE3 reached = d.pose_in_world[link];
  const Vector6 final_err = (reached.inverse() * target).log();
  EXPECT_LE(final_err.cwiseAbs().maxCoeff(), 1e-5);
}

TEST(NullspaceTracksSecondary, FrankaPostureIsImprovedVsDls) {
  // On a redundant arm (7-DoF Franka), the nullspace solver should land
  // *closer to q_rest* than plain DLS, while reaching the same target.
  // Average over 20 random (target, seed) pairs.
  const Model m = build_model_from_urdf_file(fixture("franka_fr3.urdf"));
  Data d(m);
  std::mt19937 gen(99991);

  const int link = m.njoints() - 1;
  const VectorX q_rest = VectorX::Zero(m.nq());

  int wins = 0;
  int trials_run = 0;
  constexpr int kTrials = 20;
  for (int trial = 0; trial < kTrials; ++trial) {
    const VectorX q_target = random_config(m, gen);
    forward_kinematics(m, d, q_target);
    const SE3 target = d.pose_in_world[link];
    const VectorX q_init = random_config(m, gen);

    DlsOptions dls_opts;
    dls_opts.max_iters = 500;
    const IkResult dls_res = solve_ik_dls(m, d, link, target, q_init, dls_opts);

    NullspaceOptions ns_opts;
    ns_opts.max_iters = 500;
    ns_opts.secondary_gain = 0.5;
    const IkResult ns_res = solve_ik_nullspace(m, d, link, target, q_init, q_rest, ns_opts);

    if (!dls_res.converged || !ns_res.converged) {
      continue;  // skip trials where one solver failed; not the property being tested.
    }
    ++trials_run;
    const Scalar dls_dist = (dls_res.q - q_rest).norm();
    const Scalar ns_dist = (ns_res.q - q_rest).norm();
    if (ns_dist < dls_dist) {
      ++wins;
    }
  }
  // Nullspace should win the posture comparison on a clear majority of
  // matched trials (≥ 70% of those where both solvers converged).
  ASSERT_GE(trials_run, 10) << "too few matched trials to draw a conclusion";
  const double win_rate = static_cast<double>(wins) / trials_run;
  EXPECT_GE(win_rate, 0.70) << "nullspace beat DLS on only " << wins << "/" << trials_run
                            << " posture comparisons";
}

TEST(NullspaceDegenerateAtFullRank, Simple2DofArmStillConverges) {
  // For a non-redundant model (2-DoF planar arm has rank-2 J), N(J) is
  // empty in the relevant directions, so the secondary task contributes
  // nothing. The primary task should still converge — exercising that the
  // implementation doesn't blow up on full-rank Jacobians.
  const Model m = build_model_from_urdf_file(fixture("simple_arm.urdf"));
  Data d(m);
  std::mt19937 gen(11);

  const int link = m.njoints() - 1;
  const VectorX q_target = random_config(m, gen);
  forward_kinematics(m, d, q_target);
  const SE3 target = d.pose_in_world[link];
  const VectorX q_init = random_config(m, gen);
  const VectorX q_rest = VectorX::Zero(m.nq());

  NullspaceOptions opts;
  opts.max_iters = 500;
  opts.secondary_gain = 0.5;
  const IkResult res = solve_ik_nullspace(m, d, link, target, q_init, q_rest, opts);
  EXPECT_TRUE(res.converged);
}

}  // namespace
}  // namespace tinyspatial
