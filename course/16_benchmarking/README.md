# 16 · Benchmarking

You can't optimise what you can't measure. This chapter is about the
discipline of measurement — what to measure, how to measure it, and how
to *not* mislead yourself.

The library's benchmark harness lives in [`benchmarks/`](../../benchmarks/),
one file per algorithm family, all built with Google Benchmark. The
ratios against Pinocchio live in
[`docs/BENCHMARKS.md`](../../docs/BENCHMARKS.md), and the script that
produces them is at
[`python/tools/benchmark_vs_pinocchio.py`](../../python/tools/benchmark_vs_pinocchio.py).

## What you'll learn

- The difference between *latency*, *throughput*, and what each one tells
  you.
- How to use Google Benchmark: `BENCHMARK_CAPTURE`, `state.SetItemsProcessed`,
  `DoNotOptimize`, and what those gymnastics are actually preventing.
- Why micro-benchmarks lie, and the simple steps you can take to make
  them lie less (CPU pinning, warmup, removing allocations).
- How to use `perf stat` and `callgrind` to find out *where* the time
  goes — not just how much there is.
- The governing principle: **don't optimise blind**. Get
  baselines in. Measure the change. Commit the result.

## The chapters

1. [Latency, throughput, and what 'fast' means](01_what_is_fast.md) — why a
   `ns/call` number on its own is meaningless and what context turns it
   into a useful metric.
2. [Using Google Benchmark](02_google_benchmark.md) — a tour of the
   `BENCHMARK_*` macros, fixtures, the iteration count trick, and what
   `DoNotOptimize` actually does at the assembly level.
3. [Profiling with `perf` and callgrind](03_profiling.md) — how to find
   the hot loop. Includes the workaround for kernel `perf_event_paranoid`
   and reading `callgrind_annotate` output.
4. [The optimisation loop](04_optimisation_loop.md) — the discipline that
   keeps optimisation from being random thrashing. Measure → hypothesise
   → change → measure. With a worked example: the
   `compute_joint_jacobians` per-call allocation we removed in Phase 9a.

## Where this lives in the library

| Concept                        | File / line                                |
| ------------------------------ | ------------------------------------------ |
| Benchmark harness              | [`benchmarks/CMakeLists.txt`](../../benchmarks/CMakeLists.txt) |
| `bench_rnea.cpp`               | [`benchmarks/bench_rnea.cpp`](../../benchmarks/bench_rnea.cpp) |
| `bench_kinematics.cpp`         | [`benchmarks/bench_kinematics.cpp`](../../benchmarks/bench_kinematics.cpp) |
| Pinocchio side-by-side script  | [`python/tools/benchmark_vs_pinocchio.py`](../../python/tools/benchmark_vs_pinocchio.py) |
| Tracked numbers                | [`docs/BENCHMARKS.md`](../../docs/BENCHMARKS.md) |
