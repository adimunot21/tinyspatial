#include <filesystem>
#include <random>
#include <string>

#include "tinyspatial/algo/forward_kinematics.hpp"
#include "tinyspatial/ik/differentiable.hpp"
#include "tinyspatial/ik/dls.hpp"
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

TEST(DifferentiableIk, FdAgreesOnFranka) {
  // Central-FD test of ∂q*/∂target on the 7-DoF Franka.
  //
  // For each of the 6 target-twist coordinates, perturb the target by
  // T*' = T* · exp(±ε e_k^∧), warm-restart IK from q_star to get q*_±,
  // and compare (q*_+ - q*_-) / (2ε) against the analytical column.
  const Model m = build_model_from_urdf_file(fixture("franka_fr3.urdf"));
  Data d(m);
  std::mt19937 gen(1234);

  const int link = m.njoints() - 1;
  const VectorX q_seed = random_config(m, gen);
  forward_kinematics(m, d, q_seed);
  const SE3 target = d.pose_in_world[link];

  // First, find a converged q_star from a different starting point so the
  // implicit derivative is at a real fixed point of the solver.
  const VectorX q_init = random_config(m, gen);
  DlsOptions opts;
  opts.max_iters = 500;
  opts.tolerance = 1e-12;  // very tight convergence
  opts.damping = 1e-5;     // tiny damping so the implicit theorem holds
  const IkResult res = solve_ik_dls(m, d, link, target, q_init, opts);
  ASSERT_TRUE(res.converged) << "primary IK didn't converge; can't test implicit derivative";

  const MatrixX dq_dtarget = ik_implicit_derivative(m, d, link, res.q, opts.damping);

  // Central FD column-by-column.
  MatrixX fd(m.nv(), 6);
  constexpr Scalar kEps = 1e-5;
  for (int k = 0; k < 6; ++k) {
    Vector6 delta = Vector6::Zero();
    delta(k) = kEps;
    const SE3 target_plus = target * SE3::exp(delta);
    delta(k) = -kEps;
    const SE3 target_minus = target * SE3::exp(delta);

    // Warm-start from q_star — the derivative is local, so we want the
    // closest fixed point.
    DlsOptions warm = opts;
    warm.max_iters = 200;
    const IkResult plus = solve_ik_dls(m, d, link, target_plus, res.q, warm);
    const IkResult minus = solve_ik_dls(m, d, link, target_minus, res.q, warm);
    ASSERT_TRUE(plus.converged && minus.converged)
        << "perturbed IK failed; can't run FD on k=" << k;

    fd.col(k) = (plus.q - minus.q) / (2.0 * kEps);
  }

  const Scalar diff = (dq_dtarget - fd).cwiseAbs().maxCoeff();
  // FD floor is ~ε² = 1e-10 in the numerator, divided by ε = 1e-5, so
  // ~1e-5 absolute, plus solver-tolerance contamination from each
  // perturbed call. 1e-4 is a defensible bar for FD-style validation.
  EXPECT_LE(diff, 1e-4) << "analytical ∂q*/∂target disagrees with FD";
}

TEST(DifferentiableIk, ShapeAndFinite) {
  // Sanity: output is nv × 6 and all entries finite for any seed config.
  const Model m = build_model_from_urdf_file(fixture("ur5e.urdf"));
  Data d(m);
  std::mt19937 gen(42);
  const VectorX q = random_config(m, gen);
  const MatrixX dq_dtarget = ik_implicit_derivative(m, d, m.njoints() - 1, q);
  EXPECT_EQ(dq_dtarget.rows(), m.nv());
  EXPECT_EQ(dq_dtarget.cols(), 6);
  EXPECT_TRUE(dq_dtarget.allFinite());
}

TEST(DifferentiableIk, DampingMonotonicallyShrinksDerivativeMagnitude) {
  // Increasing the damping λ should shrink the derivative's norm
  // (the damped pseudoinverse is "smaller" than the undamped one). This
  // catches a sign-flip on the damping term.
  const Model m = build_model_from_urdf_file(fixture("franka_fr3.urdf"));
  Data d(m);
  std::mt19937 gen(77);
  const VectorX q = random_config(m, gen);

  const MatrixX d_small = ik_implicit_derivative(m, d, m.njoints() - 1, q, 1e-4);
  const MatrixX d_large = ik_implicit_derivative(m, d, m.njoints() - 1, q, 1.0);
  EXPECT_GT(d_small.norm(), d_large.norm()) << "larger damping should reduce derivative magnitude";
}

}  // namespace
}  // namespace tinyspatial
