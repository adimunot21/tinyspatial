/// \file bench_kinematics.cpp
/// \brief Throughput benchmarks for forward kinematics and Jacobian
/// computation across the fixture URDFs.
///
/// FK is the cheapest baseline (~hundreds of nanoseconds per joint); the
/// Jacobian builds on top of FK and is the per-call cost in any IK loop.
/// `compute_joint_jacobians` produces *all* per-joint Jacobians in one sweep
/// and is the workhorse for analytical-derivative paths.
#include <benchmark/benchmark.h>
#include <filesystem>
#include <random>
#include <string>
#include <vector>

#include "tinyspatial/algo/forward_kinematics.hpp"
#include "tinyspatial/algo/jacobian.hpp"
#include "tinyspatial/diff/fk_derivatives.hpp"
#include "tinyspatial/urdf/urdf_loader.hpp"

namespace {

constexpr const char* kFixtureDir = TINYSPATIAL_FIXTURE_DIR;

std::string fixture(const std::string& name) {
  return (std::filesystem::path(kFixtureDir) / name).string();
}

struct ModelKit {
  tinyspatial::Model model;
  tinyspatial::Data data;
  tinyspatial::VectorX q;

  explicit ModelKit(const std::string& urdf)
      : model(tinyspatial::build_model_from_urdf_file(urdf)),
        data(model),
        q(tinyspatial::VectorX::Zero(model.nq())) {
    std::mt19937 gen(0xCAFE);
    std::uniform_real_distribution<double> uni(-1.0, 1.0);
    for (int i = 0; i < model.nq(); ++i) {
      q(i) = uni(gen);
    }
  }
};

void bench_fk_impl(benchmark::State& state, const char* urdf) {
  ModelKit kit(fixture(urdf));
  for (auto _ : state) {
    tinyspatial::forward_kinematics(kit.model, kit.data, kit.q);
    benchmark::DoNotOptimize(kit.data);
  }
  state.SetItemsProcessed(state.iterations());
}

void bench_jacobian_impl(benchmark::State& state, const char* urdf) {
  ModelKit kit(fixture(urdf));
  tinyspatial::Matrix6X j(6, kit.model.nv());
  const int link = kit.model.njoints() - 1;
  tinyspatial::forward_kinematics(kit.model, kit.data, kit.q);
  for (auto _ : state) {
    tinyspatial::compute_jacobian(kit.model, kit.data, link, j, tinyspatial::JacobianFrame::kLocal);
    benchmark::DoNotOptimize(j);
  }
  state.SetItemsProcessed(state.iterations());
}

void bench_joint_jacobians_impl(benchmark::State& state, const char* urdf) {
  ModelKit kit(fixture(urdf));
  std::vector<tinyspatial::Matrix6X> jacobians;
  for (auto _ : state) {
    tinyspatial::compute_joint_jacobians(kit.model, kit.data, kit.q, jacobians);
    benchmark::DoNotOptimize(jacobians);
  }
  state.SetItemsProcessed(state.iterations());
}

}  // namespace

BENCHMARK_CAPTURE(bench_fk_impl, simple_arm, "simple_arm.urdf");
BENCHMARK_CAPTURE(bench_fk_impl, franka_fr3, "franka_fr3.urdf");
BENCHMARK_CAPTURE(bench_fk_impl, ur5e, "ur5e.urdf");
BENCHMARK_CAPTURE(bench_fk_impl, so_arm101, "so_arm101.urdf");

BENCHMARK_CAPTURE(bench_jacobian_impl, simple_arm, "simple_arm.urdf");
BENCHMARK_CAPTURE(bench_jacobian_impl, franka_fr3, "franka_fr3.urdf");
BENCHMARK_CAPTURE(bench_jacobian_impl, ur5e, "ur5e.urdf");
BENCHMARK_CAPTURE(bench_jacobian_impl, so_arm101, "so_arm101.urdf");

BENCHMARK_CAPTURE(bench_joint_jacobians_impl, simple_arm, "simple_arm.urdf");
BENCHMARK_CAPTURE(bench_joint_jacobians_impl, franka_fr3, "franka_fr3.urdf");
BENCHMARK_CAPTURE(bench_joint_jacobians_impl, ur5e, "ur5e.urdf");
BENCHMARK_CAPTURE(bench_joint_jacobians_impl, so_arm101, "so_arm101.urdf");

BENCHMARK_MAIN();
