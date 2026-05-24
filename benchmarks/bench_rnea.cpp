/// \file bench_rnea.cpp
/// \brief Throughput benchmark for the Featherstone RNEA inverse-dynamics
/// algorithm.
///
/// The headline metric is **RNEA evaluations / second / core on a 7-DoF
/// Franka model** (CLAUDE.md §12). Target: ≥ 6M, within 1.4× of Pinocchio.
///
/// Each iteration calls `rnea(...)` once for the given fixture model at the
/// per-thread-cached `(q, v, a)`. We pre-build the Model / Data outside the
/// timed region; only the algorithm proper is measured.
#include <benchmark/benchmark.h>
#include <filesystem>
#include <random>
#include <string>

#include "tinyspatial/algo/aba.hpp"
#include "tinyspatial/algo/crba.hpp"
#include "tinyspatial/algo/rnea.hpp"
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
  tinyspatial::VectorX v;
  tinyspatial::VectorX a;
  tinyspatial::VectorX tau;

  explicit ModelKit(const std::string& urdf)
      : model(tinyspatial::build_model_from_urdf_file(urdf)),
        data(model),
        q(tinyspatial::VectorX::Zero(model.nq())),
        v(tinyspatial::VectorX::Zero(model.nv())),
        a(tinyspatial::VectorX::Zero(model.nv())),
        tau(tinyspatial::VectorX::Zero(model.nv())) {
    // Deterministic non-trivial state. Random values matter — RNEA at zero
    // skips most of the inner work that dominates real-world calls.
    std::mt19937 gen(0xCAFE);
    std::uniform_real_distribution<double> uni(-1.0, 1.0);
    for (int i = 0; i < model.nq(); ++i) {
      q(i) = uni(gen);
    }
    for (int i = 0; i < model.nv(); ++i) {
      v(i) = uni(gen);
      a(i) = uni(gen);
    }
  }
};

void bench_rnea_impl(benchmark::State& state, const char* urdf) {
  ModelKit kit(fixture(urdf));
  for (auto _ : state) {
    tinyspatial::rnea(kit.model, kit.data, kit.q, kit.v, kit.a, kit.tau);
    benchmark::DoNotOptimize(kit.tau);
  }
  state.SetItemsProcessed(state.iterations());
}

void bench_crba_impl(benchmark::State& state, const char* urdf) {
  ModelKit kit(fixture(urdf));
  tinyspatial::MatrixX m_out(kit.model.nv(), kit.model.nv());
  for (auto _ : state) {
    tinyspatial::crba(kit.model, kit.data, kit.q, m_out);
    benchmark::DoNotOptimize(m_out);
  }
  state.SetItemsProcessed(state.iterations());
}

void bench_aba_impl(benchmark::State& state, const char* urdf) {
  ModelKit kit(fixture(urdf));
  tinyspatial::VectorX qdd(kit.model.nv());
  // Seed tau with the gravity-bias torques so ABA returns ~zero qdd in
  // expectation; doesn't change throughput, but makes the numbers more
  // realistic.
  tinyspatial::rnea(kit.model, kit.data, kit.q, kit.v, kit.a, kit.tau);
  for (auto _ : state) {
    tinyspatial::aba(kit.model, kit.data, kit.q, kit.v, kit.tau, qdd);
    benchmark::DoNotOptimize(qdd);
  }
  state.SetItemsProcessed(state.iterations());
}

}  // namespace

// One benchmark per algorithm × fixture. The Franka 7-DoF row is the one
// CLAUDE.md §12 calls out as the headline; the others are there to show how
// performance scales with DoF count and topology.
BENCHMARK_CAPTURE(bench_rnea_impl, simple_arm, "simple_arm.urdf");
BENCHMARK_CAPTURE(bench_rnea_impl, franka_fr3, "franka_fr3.urdf");
BENCHMARK_CAPTURE(bench_rnea_impl, ur5e, "ur5e.urdf");
BENCHMARK_CAPTURE(bench_rnea_impl, so_arm101, "so_arm101.urdf");

BENCHMARK_CAPTURE(bench_crba_impl, simple_arm, "simple_arm.urdf");
BENCHMARK_CAPTURE(bench_crba_impl, franka_fr3, "franka_fr3.urdf");
BENCHMARK_CAPTURE(bench_crba_impl, ur5e, "ur5e.urdf");
BENCHMARK_CAPTURE(bench_crba_impl, so_arm101, "so_arm101.urdf");

BENCHMARK_CAPTURE(bench_aba_impl, simple_arm, "simple_arm.urdf");
BENCHMARK_CAPTURE(bench_aba_impl, franka_fr3, "franka_fr3.urdf");
BENCHMARK_CAPTURE(bench_aba_impl, ur5e, "ur5e.urdf");
BENCHMARK_CAPTURE(bench_aba_impl, so_arm101, "so_arm101.urdf");

BENCHMARK_MAIN();
