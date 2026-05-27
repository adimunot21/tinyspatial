/// \file bench_rnea_derivatives.cpp
/// \brief Throughput for the analytical RNEA-derivative pass (Carpentier-
/// Mansard 2018). Reports ns/call so we can quote the ratio against plain
/// RNEA: in steady state the derivative pass should be ~3–5× a single RNEA.
#include <benchmark/benchmark.h>
#include <filesystem>
#include <random>
#include <string>

#include "tinyspatial/diff/rnea_derivatives.hpp"
#include "tinyspatial/urdf/urdf_loader.hpp"

namespace {

constexpr const char* kFixtureDir = TINYSPATIAL_FIXTURE_DIR;

std::string fixture(const std::string& name) {
  return (std::filesystem::path(kFixtureDir) / name).string();
}

struct ModelKit {
  tinyspatial::Model model;
  tinyspatial::Data data;
  tinyspatial::VectorX q, v, a;
  tinyspatial::MatrixX dq, dv, da;

  explicit ModelKit(const std::string& urdf)
      : model(tinyspatial::build_model_from_urdf_file(urdf)),
        data(model),
        q(tinyspatial::VectorX::Zero(model.nq())),
        v(tinyspatial::VectorX::Zero(model.nv())),
        a(tinyspatial::VectorX::Zero(model.nv())),
        dq(model.nv(), model.nv()),
        dv(model.nv(), model.nv()),
        da(model.nv(), model.nv()) {
    std::mt19937 gen(0xCAFE);
    std::uniform_real_distribution<double> uni(-1.0, 1.0);
    for (int i = 0; i < model.nq(); ++i)
      q(i) = uni(gen);
    for (int i = 0; i < model.nv(); ++i) {
      v(i) = uni(gen);
      a(i) = uni(gen);
    }
  }
};

void bench_rnea_derivatives_impl(benchmark::State& state, const char* urdf) {
  ModelKit kit(fixture(urdf));
  for (auto _ : state) {
    tinyspatial::rnea_derivatives(kit.model, kit.data, kit.q, kit.v, kit.a, kit.dq, kit.dv, kit.da);
    benchmark::DoNotOptimize(kit.dq);
  }
  state.SetItemsProcessed(state.iterations());
}

}  // namespace

BENCHMARK_CAPTURE(bench_rnea_derivatives_impl, simple_arm, "simple_arm.urdf");
BENCHMARK_CAPTURE(bench_rnea_derivatives_impl, franka_fr3, "franka_fr3.urdf");
BENCHMARK_CAPTURE(bench_rnea_derivatives_impl, ur5e, "ur5e.urdf");
BENCHMARK_CAPTURE(bench_rnea_derivatives_impl, so_arm101, "so_arm101.urdf");

BENCHMARK_MAIN();
