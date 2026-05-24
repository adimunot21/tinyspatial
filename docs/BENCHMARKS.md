# tinyspatial — benchmark log

Each row is the latest committed throughput on the canonical fixture URDFs.
Numbers come from `benchmarks/bench_rnea.cpp` built with the `release` preset
on the machine listed below. **Optimisation work hasn't started yet**
(CLAUDE.md §12 — don't optimise blind; benchmarks first); the numbers below
are the unoptimised Featherstone-textbook baseline.

## Headline

| Algorithm | Robot | Throughput (calls / sec / core) |
| --------- | ----- | ------------------------------- |
| RNEA      | `franka_fr3` (7 DoF) | **194 K** |
| CRBA      | `franka_fr3` (7 DoF) | **122 K** |
| ABA       | `franka_fr3` (7 DoF) | **63 K**  |

CLAUDE.md §12 target for RNEA on a 7-DoF arm: **≥ 6M / s / core, within
1.4× of Pinocchio.** We are ~30× short — that gap is what Phase 7 / 8
optimisation work is for.

## Full table (`2026-05-24` baseline)

| Algorithm | Robot | ns / call | calls / sec / core |
| --------- | ----- | --------- | ------------------ |
| RNEA | `simple_arm` (2 DoF) | 1350 | 741 K |
| RNEA | `franka_fr3` (7 DoF) | 5148 | 194 K |
| RNEA | `ur5e` (6 DoF) | 4402 | 227 K |
| RNEA | `so_arm101` (5 DoF + fixed) | 4289 | 233 K |
| CRBA | `simple_arm` | 1103 | 906 K |
| CRBA | `franka_fr3` | 8212 | 122 K |
| CRBA | `ur5e` | 6078 | 165 K |
| CRBA | `so_arm101` | 4786 | 209 K |
| ABA | `simple_arm` | 4352 | 230 K |
| ABA | `franka_fr3` | 15983 | 63 K |
| ABA | `ur5e` | 14093 | 71 K |
| ABA | `so_arm101` | 12108 | 83 K |

## Machine

- Lenovo Legion Y540, Intel i7-9750H (6 cores, 12 threads, 2.6 GHz base /
  4.5 GHz boost)
- 32 GB DDR4-2666
- Ubuntu 24.04.4 LTS, Linux 6.17
- GCC 13.3.0, `-O3 -DNDEBUG` (CMake `Release`)
- CPU scaling left enabled (Google Benchmark warned); will be pinned for
  the next round.

## Reproducing

```bash
cmake --preset=release
cmake --build build/release -j
./build/release/benchmarks/bench_rnea --benchmark_min_time=0.5s
```

## Next steps (deferred)

1. Disable CPU frequency scaling for stable numbers.
2. Add a comparison row from Pinocchio 3.9.
3. Profile the hot path; `cross()` and the SE(3) inverse-then-multiply
   pattern in `rnea.hpp` are the obvious places to start.
4. Track the perf delta of each PR via a small comment-on-PR action.
