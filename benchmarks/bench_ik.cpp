/// \file bench_ik.cpp
/// \brief Throughput benchmarks for inverse-kinematics solvers.
///
/// Two flavours: damped least-squares (`solve_ik_dls`) and task-priority
/// nullspace IK (`solve_ik_nullspace`). We pick a *reachable* target by
/// forward-kinematicking a known seed, then time how fast the solver
/// re-finds it from a perturbed initial guess. The benchmark therefore
/// measures *converged-path* cost (~10–50 inner iterations on Franka)
/// rather than `max_iters`-bound worst-case cost.
#include <benchmark/benchmark.h>
#include <filesystem>
#include <random>
#include <string>

#include "tinyspatial/algo/forward_kinematics.hpp"
#include "tinyspatial/ik/differentiable.hpp"
#include "tinyspatial/ik/dls.hpp"
#include "tinyspatial/ik/nullspace.hpp"
#include "tinyspatial/urdf/urdf_loader.hpp"

namespace {

constexpr const char* kFixtureDir = TINYSPATIAL_FIXTURE_DIR;

std::string fixture(const std::string& name) {
  return (std::filesystem::path(kFixtureDir) / name).string();
}

struct IkKit {
  tinyspatial::Model model;
  tinyspatial::Data data;
  tinyspatial::VectorX q_truth;
  tinyspatial::VectorX q_init;
  tinyspatial::SE3 target;

  explicit IkKit(const std::string& urdf)
      : model(tinyspatial::build_model_from_urdf_file(urdf)),
        data(model),
        q_truth(tinyspatial::VectorX::Zero(model.nq())),
        q_init(tinyspatial::VectorX::Zero(model.nq())) {
    std::mt19937 gen(0xCAFE);
    std::uniform_real_distribution<double> uni(-0.5, 0.5);
    for (int i = 0; i < model.nq(); ++i) {
      q_truth(i) = uni(gen);
      q_init(i) = q_truth(i) + 0.1 * uni(gen);
    }
    tinyspatial::forward_kinematics(model, data, q_truth);
    target = data.pose_in_world[model.njoints() - 1];
  }
};

void bench_solve_ik_dls_impl(benchmark::State& state, const char* urdf) {
  IkKit kit(fixture(urdf));
  const int link = kit.model.njoints() - 1;
  tinyspatial::DlsOptions opts;
  opts.max_iters = 200;
  opts.tolerance = 1e-8;
  opts.damping = 1e-3;
  for (auto _ : state) {
    auto result =
        tinyspatial::solve_ik_dls(kit.model, kit.data, link, kit.target, kit.q_init, opts);
    benchmark::DoNotOptimize(result);
  }
  state.SetItemsProcessed(state.iterations());
}

void bench_solve_ik_nullspace_impl(benchmark::State& state, const char* urdf) {
  IkKit kit(fixture(urdf));
  const int link = kit.model.njoints() - 1;
  tinyspatial::NullspaceOptions opts;
  opts.max_iters = 200;
  opts.tolerance = 1e-8;
  opts.damping = 1e-3;
  opts.secondary_gain = 0.5;
  const tinyspatial::VectorX q_rest = tinyspatial::VectorX::Zero(kit.model.nq());
  for (auto _ : state) {
    auto result = tinyspatial::solve_ik_nullspace(kit.model, kit.data, link, kit.target, kit.q_init,
                                                  q_rest, opts);
    benchmark::DoNotOptimize(result);
  }
  state.SetItemsProcessed(state.iterations());
}

void bench_ik_implicit_derivative_impl(benchmark::State& state, const char* urdf) {
  IkKit kit(fixture(urdf));
  const int link = kit.model.njoints() - 1;
  for (auto _ : state) {
    tinyspatial::MatrixX dq_dT =
        tinyspatial::ik_implicit_derivative(kit.model, kit.data, link, kit.q_truth, 1e-3);
    benchmark::DoNotOptimize(dq_dT);
  }
  state.SetItemsProcessed(state.iterations());
}

}  // namespace

BENCHMARK_CAPTURE(bench_solve_ik_dls_impl, franka_fr3, "franka_fr3.urdf");
BENCHMARK_CAPTURE(bench_solve_ik_dls_impl, ur5e, "ur5e.urdf");
BENCHMARK_CAPTURE(bench_solve_ik_dls_impl, so_arm101, "so_arm101.urdf");

BENCHMARK_CAPTURE(bench_solve_ik_nullspace_impl, franka_fr3, "franka_fr3.urdf");
BENCHMARK_CAPTURE(bench_solve_ik_nullspace_impl, ur5e, "ur5e.urdf");

BENCHMARK_CAPTURE(bench_ik_implicit_derivative_impl, franka_fr3, "franka_fr3.urdf");
BENCHMARK_CAPTURE(bench_ik_implicit_derivative_impl, ur5e, "ur5e.urdf");

BENCHMARK_MAIN();
