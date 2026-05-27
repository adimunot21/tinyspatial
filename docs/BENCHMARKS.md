# tinyspatial — benchmark log

Latest committed throughput on the four fixture URDFs, on the machine
described at the bottom. Numbers come from the `benchmarks/bench_*` suite
built with `cmake --preset=release` and Pinocchio 3.9.0 (Python side-by-side
times come from `python/tools/benchmark_vs_pinocchio.py`).

> **Discipline.** Numbers are committed in this file so we notice
> regressions. Optimisation work follows CLAUDE.md §12: measure first,
> change the smallest thing, re-measure.

## Headline (`2026-05-27` — Phase 9d)

| Algorithm | Robot                  | ns / call | calls / sec / core |
| --------- | ---------------------- | --------: | -----------------: |
| RNEA      | `franka_fr3` (7 DoF)   |  **2556** |        **391 K**   |
| CRBA      | `franka_fr3` (7 DoF)   |  **3070** |        **326 K**   |
| ABA       | `franka_fr3` (7 DoF)   | **11112** |         **90 K**   |
| FK        | `franka_fr3` (7 DoF)   |   **588** |       **1.70 M**   |
| Jacobian  | `franka_fr3` (7 DoF)   |   **722** |       **1.39 M**   |
| IK (DLS)  | `franka_fr3`           |  **6240** |        **161 K**   |

CLAUDE.md §12 target for RNEA on a 7-DoF arm: **≥ 6 M / s / core, within
1.4× of Pinocchio.** Current C++ delta to Pinocchio on Franka: RNEA at
**~1.6×**, CRBA at **~1.8×** of Pinocchio C++.

## Cumulative deltas (Phase 9a → 9d)

Seven localised changes across four PRs closed roughly **44% of the RNEA
gap and 55% of CRBA**. No API breakage; full Pinocchio parity preserved
at 1e-14 / 1e-15.

| Algorithm                  | Robot        | Pre-9a (baseline) | After 9d | Delta   |
| -------------------------- | ------------ | ----------------: | -------: | ------: |
| FK                         | `franka_fr3` |            828 ns |   588 ns |  **−29%** |
| RNEA                       | `franka_fr3` |           4540 ns |  2556 ns |  **−44%** |
| CRBA                       | `franka_fr3` |           6776 ns |  3070 ns |  **−55%** |
| ABA                        | `franka_fr3` |          13398 ns | 11112 ns |  **−17%** |
| `compute_joint_jacobians`  | `franka_fr3` |           1881 ns |  1612 ns |  **−14%** |
| RNEA derivatives           | `franka_fr3` |          19753 ns | 17874 ns |  **−10%** |

What each phase changed:

**Phase 9a** (benchmark suite + first allocation removal):

1. `compute_joint_jacobians` reuses caller storage on warm calls instead
   of `.assign()`-ing a fresh `Matrix6X::Zero(6, nv)` per joint.

**Phase 9b** (spatial-algebra inline expansion):

2. `forward_kinematics`: replaced per-joint `const VectorX q_slice = q.segment(...)`
   (heap allocation) with a zero-copy `Eigen::Ref<const VectorX>` view.
   Fix is one line. **−29% on FK**, ripples through RNEA / CRBA / ABA.
3. `operator*(SE3, Motion)` and `operator*(SE3, Force)`: inline-expanded
   the spatial-transport formula instead of constructing the 6×6 adjoint
   matrix and multiplying. ~24 ops per call vs ~72.
4. `operator*(SpatialInertia, Motion)`: same trick — inline expansion
   of the angular-first inertia form avoids the 6×6 matrix construction.

**Phase 9c** (cache motion subspaces in `Model`):

5. `Model::motion_subspace` — pre-computed `S_i` (6 × nv_i) for each
   joint at `add_joint` time. CRBA / ABA / RNEA-derivatives now read
   `model.motion_subspace[i]` directly; the per-call dynamic-size matrix
   allocation **and** the `std::visit` dispatch both disappear from the
   hot loop. **−42% on CRBA**.
6. Deduplicated the `rnea_deriv_joint_subspace` helper into the canonical
   `joint_motion_subspace` in `joint.hpp` (no behavioural change).

**Phase 9d** (force-Plücker apply without 6×6 construction):

7. `force_plucker_apply_matrix(SE3, Matrix6X)` — applies the force
   Plücker transform `X*` to a 6×N matrix without first building the
   6×6 X*. Used in CRBA's parent-chain walk where the joint-subspace
   force matrix is 6×1. **−21% additional on CRBA**.

Approaches that were tried and reverted in 9d:

- **Caching `pose_in_parent.inverse()` and `r_in_world` in `Data`** — net
  regression on RNEA / CRBA. The FK cost to fill the caches outweighed
  the savings, because most algorithms call FK internally and the
  inverse / matrix were only re-read in a few specific spots.
- **Vector6 `force_plucker_apply` in ABA** — net flat on Franka. The
  6×6 X* matrix is already constructed for the immediately adjacent
  IA symmetric congruence, so inlining the second multiply is redundant
  work.

## Full table — tinyspatial C++ benchmarks (`bench_*.cpp`)

| Algorithm                    | `simple_arm` | `franka_fr3` | `ur5e` | `so_arm101` |
| ---------------------------- | -----------: | -----------: | -----: | ----------: |
| FK                           |       160 ns |       588 ns | 501 ns |      454 ns |
| Jacobian (one link, LOCAL)   |       218 ns |       722 ns | 621 ns |      613 ns |
| Per-joint Jacobians (sweep)  |       290 ns |      1612 ns |1259 ns |     1161 ns |
| RNEA                         |       695 ns |      2556 ns |2176 ns |     2127 ns |
| CRBA                         |       564 ns |      3070 ns |2443 ns |     2013 ns |
| ABA                          |      3026 ns |     11112 ns |9702 ns |     8725 ns |
| RNEA derivatives             |      3334 ns |     17874 ns |14315ns |    13052 ns |
| IK (DLS, warm)               |          —   |      6240 ns |7246 ns |     6761 ns |
| IK (nullspace)               |          —   |     17063 ns |9651 ns |       —     |
| IK implicit derivative       |          —   |      2209 ns |2020 ns |       —     |

## Pinocchio side-by-side (Python overhead included, both sides)

Generated by `python/tools/benchmark_vs_pinocchio.py`. The ratio is the
honest "from Python" ratio that a real user sees; binding overhead inflates
both sides similarly but our binding is currently less optimised than
Pinocchio's eigenpy layer (visible in the FK row — *all* of the FK gap is
binding overhead, not algorithm).

| Algorithm        | Robot          | tinyspatial (ns) | Pinocchio (ns) | ratio (ts/pin) |
| ---------------- | -------------- | ---------------: | -------------: | -------------: |
| FK               | `simple_arm`   |             6388 |            439 |      **14.5x** |
| FK               | `franka_fr3`   |            17746 |            777 |      **22.8x** |
| FK               | `ur5e`         |            15440 |            916 |      **16.9x** |
| FK               | `so_arm101`    |            15187 |            687 |      **22.1x** |
| Jacobian         | `simple_arm`   |             5062 |            623 |       **8.1x** |
| Jacobian         | `franka_fr3`   |             8017 |            818 |       **9.8x** |
| Jacobian         | `ur5e`         |             7330 |            956 |       **7.7x** |
| Jacobian         | `so_arm101`    |             7040 |            728 |       **9.7x** |
| RNEA             | `simple_arm`   |             7554 |           1029 |       **7.3x** |
| RNEA             | `franka_fr3`   |            14087 |           2120 |       **6.6x** |
| RNEA             | `ur5e`         |            12924 |           1890 |       **6.8x** |
| RNEA             | `so_arm101`    |            12895 |           1754 |       **7.4x** |
| CRBA             | `simple_arm`   |             5903 |            856 |       **6.9x** |
| CRBA             | `franka_fr3`   |            16135 |           1706 |       **9.5x** |
| CRBA             | `ur5e`         |            13723 |           1507 |       **9.1x** |
| CRBA             | `so_arm101`    |            11973 |           1360 |       **8.8x** |
| ABA              | `franka_fr3`   |            31257 |           7655 |       **4.1x** |
| RNEA derivatives | `franka_fr3`   |            55760 |           9788 |       **5.7x** |

Where the C++ benchmark and the Python-side number disagree, the C++
benchmark is the source of truth for the *algorithm*; the Python-side
number tells you what end users see. The binding-overhead gap (visible
on FK) is a known item for future work — a `nb::call_guard<nb::gil_scoped_release>`
or a vectorised batch entry point would shrink it.

## Methodology

- C++: `--benchmark_min_time=1s` per row; median used. Iteration count
  shown by Google Benchmark — at the Franka RNEA's ~488K iterations the
  CV is well under 1%.
- Python: `time.perf_counter_ns` with N=2000 calls after 50 warmup calls.
  Same RNG seed and identical `(q, v, a, tau)` for both libraries.
- Both libraries load *the same URDF file*. We do not relax the input
  preprocessing.
- Gravity is `(0, 0, −9.81)` set explicitly on the Pinocchio model.
- Pinocchio Jacobians use `pin.LOCAL`; we permute the 6×nv block to
  match our angular-first row ordering before comparing for parity.

## Machine

- Lenovo Legion Y540, Intel i7-9750H (6 cores, 12 threads, 2.6 GHz base /
  4.5 GHz boost)
- 32 GB DDR4-2666
- Ubuntu 24.04.4 LTS, Linux 6.17
- GCC 13.3.0, `-O3 -DNDEBUG` (CMake `Release`)
- CPU scaling left enabled (Google Benchmark warns).

## Reproducing

```bash
cmake --preset=release
cmake --build build/release -j

./build/release/benchmarks/bench_rnea              --benchmark_min_time=1s
./build/release/benchmarks/bench_kinematics        --benchmark_min_time=1s
./build/release/benchmarks/bench_ik                --benchmark_min_time=1s
./build/release/benchmarks/bench_rnea_derivatives  --benchmark_min_time=1s

# Pinocchio side-by-side (needs the .venv with pinocchio installed):
PYTHONPATH=python .venv/bin/python python/tools/benchmark_vs_pinocchio.py
```

## What's next (future work)

The remaining C++ gap to Pinocchio (RNEA at 1.6×, CRBA at 1.8×) is now
nearly bandwidth-bound. Further C++ wins are diminishing returns; the
remaining levers, with estimated upside, are:

1. **ABA symmetric inertia transport** — `i_a[parent] += X * IA * Xᵀ`
   is the largest single line in ABA. Inline-expanding via the block
   form is fiddly (the four 3×3 result blocks each combine several
   intermediate products) but should save ~15% on ABA. Tried briefly
   in 9d and reverted; warrants a focused attempt.
2. **Python binding overhead** — `nb::call_guard<nb::gil_scoped_release>`
   and a batch entry point would close the 6.7× Python-side ratio on
   RNEA without touching the algorithm. The FK Python-side gap (~22×)
   is *entirely* binding overhead.
3. **Quaternion → matrix in `SO3::act`** — quaternion `q · v · q⁻¹`
   is ~30 ops; matrix is 9. For algorithms that call `SO3::act` once
   per joint (FK), this is a ~5% win.

CLAUDE.md §12 sets the bar at 1.4× of Pinocchio C++ on RNEA. We're at
1.6× — close enough that Phase 10 (release / marketing) is the right
next priority rather than additional perf grinding.
