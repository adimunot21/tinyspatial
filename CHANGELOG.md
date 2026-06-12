# Changelog

All notable changes to this project are documented here. The format follows
[Keep a Changelog](https://keepachangelog.com/en/1.1.0/), and this project
adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Added

- **`validation.yml` workflow — Pinocchio parity in CI (CLAUDE.md §10/§11).**
  - Runs the `tests/validation/test_kinematics.py` cross-check on PRs that touch
    `include/`, `src/`, or `tests/validation/`, plus a nightly schedule and
    manual dispatch. Uses a pip venv with the pinned `pin==3.9.0` cmeel wheels
    (no conda in CI), builds the nanobind binding, runs
    `ctest --preset=validation -L pinocchio_parity`, and upserts the regenerated
    parity table as a sticky PR comment. Closes the gap where the parity table
    was hand-committed and never CI-verified.
- **`docs/ARCHITECTURE.md`** — structural overview for reviewers (header-mostly
  rationale, Model/Data split, convention table, the dependency boundary,
  build-target map). Referenced by the README but previously missing.
- **`docs/DEVELOPMENT.md`** — contributor guide (build/test/validate commands,
  style, the dual-track rule, the add-an-algorithm flow, git/PR conventions).
  Referenced by the README but previously missing.

### Changed

- **`ci.yml` expanded from the Phase-0 single job to the CLAUDE.md §10 matrix:**
  `ubuntu-22.04/gcc-12` and `ubuntu-24.04/clang-17`, each in Debug (ASan+UBSan)
  and Release. Added a `lint` job: `clang-format` (gating) + `clang-tidy`
  (advisory, per CLAUDE.md §7). Release configs skip benchmarks to protect the
  time budget; the slow Pinocchio cross-check stays in `validation.yml`.
- **Honest performance accounting.** The README and `PROJECT_PLAN.md` §2 now
  state plainly that the `≥ 6 M RNEA/s, within 1.4×` figure is an *aspiration*,
  not a delivered number — current measured standing is **~390 K/s, ~1.6×** of
  Pinocchio C++ — and that the Pinocchio comparison column is an estimate
  pending a measured C++ head-to-head. No claim now reads as achieved when it
  is not.
- **Documented the IK `std::expected` exception** in `CLAUDE.md` §7 and
  `docs/ARCHITECTURE.md`: iterative best-effort solvers return a result struct
  with a `converged` flag by design.

### Added

- **GitHub Pages deploy for the course site.**
  - `.github/workflows/docs.yml` builds `mkdocs` and deploys to
    `https://adimunot21.github.io/tinyspatial/` on every push to main
    that touches `course/**` or `mkdocs.yml`.
  - Uses `actions/upload-pages-artifact` + `actions/deploy-pages` (the
    modern Actions-based Pages flow; no `gh-pages` branch).

### Changed

- Moved `course/mkdocs.yml` → repo-root `mkdocs.yml` with `docs_dir: course`.
  Required because mkdocs 1.4+ rejects `docs_dir: .` (the docs dir must be a
  child of the config file). Nav paths are unchanged (still relative to
  `docs_dir`). Local invocation simplifies from `mkdocs serve -f course/mkdocs.yml`
  to just `mkdocs serve`; CLAUDE.md updated.

## [0.1.0] — 2026-05-27

Initial public release. Every algorithm planned in `PROJECT_PLAN.md` is
implemented and validated against Pinocchio 3.9.0 to `1e-13` or below on
the Franka FR3, UR5e, SO-ARM101, and `simple_arm` fixture robots.

### Headline metrics (Franka FR3, Intel i7-9750H, GCC 13.3)

| Algorithm | Throughput      | vs Pinocchio C++ |
| --------- | --------------: | ---------------: |
| RNEA      | **391 K / s**   |             1.6× |
| CRBA      | **326 K / s**   |             1.8× |
| ABA       | **90 K / s**    |             1.5× |
| FK        | **1.70 M / s**  |             2.1× |
| Jacobian  | **1.39 M / s**  |             1.4× |

### Library surface

- **Lie groups** (`SO3`, `SE3`): `exp` / `log`, adjoint, left/right Jacobians.
- **Spatial algebra**: `Motion`, `Force`, `SpatialInertia`, Plücker transforms.
- **Kinematic tree**: `Model` / `Data`, joint variants (revolute, prismatic,
  fixed, floating), URDF loader.
- **Forward kinematics + Jacobians** in three reference frames.
- **Inverse dynamics** (RNEA), **mass matrix** (CRBA), **forward dynamics** (ABA).
- **Analytical derivatives**: per-joint Jacobians, `∂τ/∂q`, `∂τ/∂v`, `∂τ/∂a`.
- **Inverse kinematics**: damped-least-squares + task-priority null-space +
  analytical `∂q*/∂T*` via the implicit function theorem.
- **Python bindings** (nanobind), with type stubs and 3 example notebooks.

### Course

A 16-chapter from-scratch tutorial (`course/`) covering rotations, Lie
groups, spatial algebra, kinematic trees, URDF, forward kinematics,
Jacobians, RNEA, ABA + CRBA, IK (DLS + null-space + differentiable),
Python bindings, validating against Pinocchio, and benchmarking. Builds
to a mkdocs-material site.

### Added

- **Course chapter 15 — Validating against Pinocchio.**
  - Three sub-chapters: the oracle approach (why Pinocchio, what makes
    a good oracle, the build-isolation trick that keeps Pinocchio out
    of the standalone library); convention mismatches as bugs in
    disguise (angular-first vs linear-first, universe-joint offset,
    quaternion sign, Jacobian frame, gravity vector); reading the
    parity table (what each cell means, FP noise floor vs algorithm
    bug, anatomy of `tests/validation/test_kinematics.py`).
  - Registered in `course/mkdocs.yml`.

### Changed

- **Phase 9d — Force-Plücker apply without 6×6 construction (-21% CRBA).**
  - Added `force_plucker_apply_matrix(SE3, Matrix6X)` in `plucker.hpp`:
    applies the force-Plücker transform `X* = Ad_T⁻ᵀ` to a 6 × N matrix
    via the block formula `new_top = R · top + t × (R · bot); new_bot = R · bot`,
    without first building the 6×6 X* matrix.
  - CRBA's parent-chain walk now uses it for the per-joint force-Plücker
    transport. Franka: 3893 → 3070 ns (**-21%**); UR5e: 3030 → 2443 ns
    (-19%); so_arm101: 2400 → 2013 ns (-16%).
  - Also documented the two reverts in the same PR: caching
    `pose_in_parent.inverse()` and `r_in_world` in `Data` is net regression
    (the FK cost to fill the caches outweighs the savings, because
    most algorithms call FK internally), and the Vector6
    `force_plucker_apply` in ABA is flat (X* is already built for the
    adjacent IA congruence). Both lessons captured in `docs/BENCHMARKS.md`.
  - **Cumulative Phase 9 result on Franka**: CRBA 6776 → 3070 ns (**-55%
    total**); RNEA 4540 → 2556 ns (**-44%**); both at ~1.6-1.8× of
    Pinocchio C++.

- **Phase 9c — Cache joint motion subspaces in Model (-42% CRBA, -12% RNEA).**
  - Added `Model::motion_subspace` (`std::vector<Matrix6X>`), filled in
    `add_joint` from a canonical `joint_motion_subspace(Joint)` helper in
    `joint.hpp`. The matrix `S_i` (6 × nv_i) is a pure function of the
    joint variant; no need to recompute it on every CRBA / ABA call.
  - CRBA inner loop and walk-up now read `model.motion_subspace[i]`
    directly: no per-call heap allocation for the dynamic-size matrix,
    no per-call `std::visit` dispatch. Franka: 6709 → 3893 ns (**-42%**).
  - ABA now reads the cached subspaces too. Franka: 12191 → 11154 ns (-9%).
  - RNEA derivatives picks up the same pattern. Franka: 19753 → 17874 ns
    (-10%). Removed the duplicated `rnea_deriv_joint_subspace` helper.
  - Pinocchio parity preserved at 1e-13 / 1e-14 (no FP-order change in
    the affected paths).
  - **Cumulative Phase 9 result on Franka**: RNEA 4540 → 2567 ns (**-43%
    total**), CRBA 6776 → 3893 ns (**-43% total**). Now at ~1.6× of
    Pinocchio C++ on RNEA (CLAUDE.md §12 target is 1.4×).
  - `docs/BENCHMARKS.md` regenerated with cumulative table and the new
    "future work" list (ABA articulated-inertia, cached rotation matrices).

- **Phase 9b — Hot-path optimisation pass (-36% RNEA on Franka).**
  - **`forward_kinematics`**: replaced the per-joint `const VectorX q_slice =
    q.segment(...)` heap allocation with a zero-copy `Eigen::Ref<const VectorX>`
    view passed directly to `joint_transform`. FK Franka: 828 → 588 ns (-29%).
  - **`operator*(SE3, Motion)`**: inline expansion of the body-frame
    adjoint action (`new_w = R·ω`, `new_v = R·v + t × new_w`) replaces
    constructing the 6×6 adjoint matrix and multiplying. ~24 ops vs ~72.
  - **`operator*(SE3, Force)`**: same trick on the dual adjoint
    (`new_lin = R·f_lin`, `new_ang = R·f_ang + t × new_lin`).
  - **`operator*(SpatialInertia, Motion)`**: inline expansion of the
    angular-first inertia form (`new_w = I_O·ω + m·c×v`,
    `new_v = m·(v − c×ω)`) avoids the 6×6 matrix construction.
  - **Cumulative on `franka_fr3`**: RNEA 4540 → 2908 ns (-36%); ABA
    13398 → 12191 ns (-9%); FK 828 → 588 ns (-29%); `compute_joint_jacobians`
    1881 → 1612 ns (-14%).
  - **vs Pinocchio C++**: was 2.8×, now ~1.8× (target per CLAUDE.md §12
    is 1.4×). All Pinocchio parity tests still pass at the same numerical
    accuracy (RNEA 7e-14, FK 1e-15, CRBA 5e-15).
  - `docs/BENCHMARKS.md` regenerated with the new headline numbers and
    the Phase 9c roadmap (variant→switch, CRBA subspace caching).

### Added

- **Phase 9a — Benchmarks + first optimisation pass.**
  - `benchmarks/bench_kinematics.cpp` — FK, single-link Jacobian,
    `compute_joint_jacobians` rows across the 4 fixture URDFs.
  - `benchmarks/bench_ik.cpp` — `solve_ik_dls`, `solve_ik_nullspace`,
    `ik_implicit_derivative` rows.
  - `benchmarks/bench_rnea_derivatives.cpp` — analytical-derivative
    pass throughput (3–5× plain RNEA, as expected from theory).
  - `python/tools/benchmark_vs_pinocchio.py` — Python-side
    side-by-side timing emitting a Markdown table for
    `docs/BENCHMARKS.md`. Avoids the C++ Pinocchio link (Boost ban,
    cmeel libstdc++ ABI mismatch); ratios still informative.
  - **First optimisation:** `compute_joint_jacobians` now reuses the
    caller's vector storage on warm calls instead of `.assign()`-ing a
    fresh `Matrix6X::Zero(6, nv)` per joint. Franka throughput: 1881 ns
    → 1748 ns (-7%). Hot-loop callers (derivative inner sweeps) become
    allocation-free after the first call.
  - `docs/BENCHMARKS.md` regenerated with the full table + Pinocchio
    side-by-side ratios and explicit methodology section.
  - Course chapter [16 — Benchmarking](course/16_benchmarking/README.md)
    with 4 sub-chapters: what 'fast' means, using Google Benchmark,
    profiling with `perf` + callgrind, and the optimisation loop
    (with the Phase 9a allocation-fix as a worked example).

### Changed

- `compute_joint_jacobians` warm-call path is now allocation-free
  (Phase 9a optimisation, see above). The cold-call path is unchanged.

- **Phase 8 — Python bindings + examples.**
  - `src/bindings/main.cpp` expanded from the validation-only surface to
    the full public API: `SO3` / `SE3` (constructors, `exp`/`log`, `*`,
    `inverse`, `adjoint`, `matrix`, `quaternion`), read-only `Data`
    (`pose_in_world` / `pose_in_parent` accessors), `compute_joint_jacobians`,
    `DlsOptions` / `NullspaceOptions` / `IkResult`, `solve_ik_dls`,
    `solve_ik_nullspace`, and `ik_implicit_derivative`. Bindings use
    `std::optional<Options>` paired with `nb::arg(..) = nb::none()` so the
    Python defaults match the C++ defaults without boilerplate.
  - `python/tinyspatial/__init__.py` rewritten to export the full surface;
    `python/tinyspatial/__init__.pyi` ships full type stubs and
    `py.typed` advertises them for static analyzers (mypy, pyright).
  - `python/examples/` — three executable Jupyter notebooks:
    1. `01_fk_tour_ur5e.ipynb` — FK + workspace cloud + manipulability.
    2. `02_rnea_vs_pinocchio_franka.ipynb` — 1000-sample residual
       histogram against Pinocchio.
    3. `03_ik_nullspace_elbow_franka.ipynb` — DLS vs nullspace IK
       side-by-side with the implicit-derivative bonus.
  - `python/tests/test_parity.py` — 21 pytest checks across the 3
    fixture URDFs (FK, J, RNEA, CRBA, ABA, IK round-trip, implicit
    derivative shape) at `1e-9`.
  - Course chapter [14 — Python bindings](course/14_python_bindings/README.md)
    with 4 sub-chapters: why-wrap-C++, reading-the-glue (a tour of
    `main.cpp`), using-from-Python (notebook walkthrough), pitfalls
    (Eigen storage order, GIL, lifetime / rv-policy).

- **Phase 7b — Differentiable inverse kinematics.**
  - `include/tinyspatial/ik/differentiable.hpp` — `ik_implicit_derivative()`
    returns `∂q*/∂T*` as an `nv × 6` matrix using the implicit function
    theorem at the IK fixed point: `∂q*/∂T* = J^T(JJ^T + λ²I)⁻¹`,
    matching the DLS solver's damped pseudoinverse for consistency
    near singularities. Body-frame parametrization (right-multiplicative
    `T* → T* · exp(δξ^∧)`); world-frame via post-multiplication by
    `Ad_{T*}^{-1}`.
  - Tests (3 new): central-FD agreement to ≤ 1e-4 on Franka with
    matched-damping setup (λ=1e-5 in both solver and analytical formula
    removes the damping-bias contribution); shape + finiteness on
    UR5e; larger λ → smaller derivative norm (sign-flip catch).
  - Course chapter [13 — Differentiable IK](course/13_differentiable_ik/README.md)
    with 4 sub-chapters + exercises: why differentiable IK matters
    (end-to-end learning, sensitivity, MPC), the implicit function
    theorem from scratch, the IK derivation, an in-code walkthrough
    plus a PyTorch wrapping sketch.

- **Phase 7a — Inverse kinematics (DLS + nullspace).**
  - `include/tinyspatial/ik/dls.hpp` — damped least-squares (Nakamura-
    Hanafusa) iterative IK with body-frame Lie-tangent error.
    `IkResult { q, error, iterations, converged }` so callers handle
    non-convergence without exceptions.
  - `include/tinyspatial/ik/nullspace.hpp` — task-priority IK with a
    posture-attraction secondary objective `(q_rest − q)` projected
    through `N(J)`. Two-tier damping (λ=1e-2 for the primary step,
    λ_proj=1e-10 for the projector) makes the projector near-exact so
    the primary task can converge tightly.
  - Tests (15 new): convergence on ≥ 80/100 random seeds (the realistic
    single-seed bar on 6-DoF UR5e per Buss & Kim 2005); identity-seed
    early exit; max_iters honored; nullspace primary still converges
    with strong secondary gain; nullspace beats plain DLS on
    ‖q − q_rest‖ on ≥ 70% of matched Franka trials; degenerate non-
    redundant case (2-DoF simple_arm) doesn't blow up.
  - Course chapter [12 — Inverse kinematics](course/12_inverse_kinematics/README.md)
    with 6 sub-chapters and exercises: the inverse problem and SE(3)
    error, the Jacobian approach (Newton on a manifold), the
    Moore-Penrose pseudoinverse and DLS damping, an in-code walkthrough,
    null-space + secondary tasks + the two-tier damping trick, and a
    failure-modes field guide.

- **Phase 6 — Analytical derivatives.**
  - `include/tinyspatial/diff/fk_derivatives.hpp`:
    `compute_joint_jacobians()` fills a `std::vector<Matrix6X>` (one 6×nv
    matrix per joint) via the recurrence `J_i = X_pi · J_p` plus the
    joint's own screw axis at column `idx_v[i]`. Single forward sweep.
  - `include/tinyspatial/diff/rnea_derivatives.hpp`: analytical
    `∂τ/∂q`, `∂τ/∂v`, `∂τ/∂a` following Carpentier & Mansard (2018).
    Body-fixed, angular-first; O(N²) total cost via two identities
    (cross-motion antisymmetry, force-cross-motion duality) that turn
    per-column work into matrix products.
  - Unit tests (20 new): FK-derivative consistency with the existing
    single-joint Jacobian; central-FD agreement of RNEA derivatives at
    ε=1e-6 to ≤ 1e-6; `∂τ/∂a ≡ M(q)` from CRBA (structural identity) to
    ≤ 1e-10; ∂τ/∂a symmetric to ≤ 1e-10.
  - **Pinocchio 3.9 cross-validation** extended over 1000 random
    `(q, v, a)` per fixture: `∂τ/∂q` ~1e-13, `∂τ/∂v` ~1e-14, `∂τ/∂a`
    ~1e-15. All three orders of magnitude inside the 1e-10 spec.
  - Python binding: `rnea_derivatives(model, q, v, a, gravity)` returns
    a `(∂τ/∂q, ∂τ/∂v, ∂τ/∂a)` tuple.
  - Course chapter [10b — RNEA derivatives (advanced)](course/10b_rnea_derivatives/README.md):
    optional advanced read on the Carpentier-Mansard algorithm with both
    identities derived and the implementation walked through.

- **Phase 5 — Featherstone dynamics.**
  - `algo/rnea.hpp`: Recursive Newton–Euler inverse dynamics. Two-pass
    body-fixed implementation (outward velocities + accelerations with the
    gravity trick, inward wrench accumulation and joint-subspace projection).
    `std::visit` dispatches per joint type for both the `S · v_slice` motion
    contribution and the `Sᵀ · F` torque projection.
  - `algo/crba.hpp`: Composite Rigid Body Algorithm — joint-space inertia
    matrix `M(q)`. Leaves-to-root composite-inertia sweep, then for each joint
    walks up the parent chain transporting `F = ic · S_i` via the force
    Plücker transform to fill the off-diagonal blocks.
  - `algo/aba.hpp`: Articulated-Body Algorithm — forward dynamics in O(N).
    Three passes: outward `v[i]` + seed `IA[i]/pA[i]`, inward articulated-
    inertia accumulation with rank-`nv_i` reduction at each joint, outward
    `q̈` propagation with the gravity trick.
  - Tests: hand-computed gravity compensation and coupling-sign sanity for
    RNEA; symmetry + positive-definiteness + an `M(q) ≡ ∂τ/∂a` cross-check
    against RNEA for CRBA; the strongest property check for ABA —
    `ABA(q, v, RNEA(q, v, a)) ≡ a` — plus consistency with `M⁻¹(τ − h)`.
    All parameterised across the four fixtures.
  - Pinocchio cross-validation extended to RNEA, CRBA, and ABA over 1000
    random `(q, v, a, τ)` samples per fixture: agreement at **~1e-15**
    (machine precision) for RNEA/CRBA and **~1e-12** for ABA, well inside
    the `1e-10` tolerance.
  - Python binding additions: `rnea(model, q, v, a, gravity)`,
    `crba(model, q)`, `aba(model, q, v, tau, gravity)`.
  - **Benchmarks** (`benchmarks/bench_rnea.cpp`, Google Benchmark): RNEA /
    CRBA / ABA throughput on all four fixtures. Baseline on a 7-DoF Franka:
    RNEA 194K/s, CRBA 122K/s, ABA 63K/s. CLAUDE.md §12 target is 6M/s on
    RNEA — the gap is what later optimisation work is for. Recorded in
    [`docs/BENCHMARKS.md`](docs/BENCHMARKS.md).
  - Course chapters [10 (RNEA, 5 sub-chapters)](course/10_dynamics_RNEA/README.md)
    and [11 (ABA + CRBA, 5 sub-chapters)](course/11_ABA_and_CRBA/README.md),
    each with exercises and "Where this lives" tables.

- **Phase 4 — Forward kinematics + Jacobians + Pinocchio validation.**
  - `algo/forward_kinematics.hpp`: single outbound sweep over the
    topologically-ordered Model, fills `Data::pose_in_parent` and
    `Data::pose_in_world`.
  - `algo/jacobian.hpp`: 6×nv geometric Jacobian in three reference frames
    (`kLocal` / `kWorld` / `kLocalWorldAligned`). Walks up via `parent[]` so
    only ancestor joints contribute; `std::visit` dispatches each joint
    type's screw axis. One outer matrix multiply applies the frame
    conversion at the end.
  - 25 new C++ tests (92 total): hand-computed FK on `simple_arm`, 50 random
    configurations per fixture URDF (rotation matrices stay orthonormal),
    LOCAL/WORLD/LWA Jacobians agree with central finite differences to 1e-6,
    LWA equals `diag(R, R) · LOCAL` exactly, non-ancestor joints contribute
    zero columns.
  - **Pinocchio 3.9 cross-validation harness** (`TINYSPATIAL_BUILD_VALIDATION`):
    minimal nanobind binding (`src/bindings/main.cpp`), Python module under
    `python/tinyspatial/`, and `tests/validation/test_kinematics.py` running
    each fixture URDF in both libraries over 1000 random configurations.
    Agreement is at **~1e-15 (machine precision)** for FK and all three
    Jacobian frames on every robot — five orders of magnitude inside the
    `1e-10` tolerance specified in `CLAUDE.md` §11.
  - First entry in [`docs/PINOCCHIO_PARITY.md`](docs/PINOCCHIO_PARITY.md).
  - Course chapters 08 (forward kinematics, 4 sub-chapters) and 09
    (Jacobians, 5 sub-chapters), each with exercises and "Where this lives"
    tables.

- **Phase 3 — Kinematic tree + URDF loader.**
  - `model/joint.hpp`: `Joint = std::variant<JointFixed, JointRevolute,
    JointPrismatic, JointFloating>`, with `nq()/nv()/joint_transform()` free
    functions dispatching via `std::visit`.
  - `model/model.hpp`: `Model` (joints + parents + placements + inertias +
    flat `idx_q`/`idx_v` slices) with `add_joint()`/`find_joint()` builders,
    plus `Data` per-configuration scratchpad (pose_in_parent / pose_in_world /
    v / a / f, sized to the Model at construction).
  - `urdf/urdf_loader.hpp` + `src/urdf/urdf_loader.cpp`: pure-URDF subset
    loader via tinyxml2 (three-pass: links → joints → BFS assembly). Throws
    typed `UrdfParseError`; ignores `<visual>`/`<collision>`/etc.
  - Fixture URDFs under `data/robots/`: `simple_arm` (test fixture),
    `franka_fr3` (synthetic 7-DOF), `ur5e` (synthetic 6-DOF), `so_arm101`
    (synthetic 5-DOF + fixed gripper). All clearly marked synthetic; Phase 4
    swap plan documented in `data/robots/README.md`.
  - 26 new tests (73 total green, no compiler warnings, clang-tidy clean):
    each joint variant's transform, model indexing for mixed-size joints,
    URDF round-trips for all four fixtures, q=0 placement chain matches a
    hand-computed reference, 1000-iteration fuzz under ASan+UBSan that never
    crashes.
  - Course chapters 06 (kinematic trees, 4 sub-chapters) and 07 (URDF, 5
    sub-chapters), each with exercises and "Where this lives" tables.

- **Phase 2 — Spatial algebra.**
  - `spatial/motion.hpp`, `spatial/force.hpp`: typed `Motion` (twist) and
    `Force` (wrench), angular-first; SE(3) acts on each via `operator*`
    (`Ad_T` for motions, `Ad_T^{-T}` for forces).
  - `spatial/cross.hpp`: typed `cross(Motion, Motion)` and `cross(Motion, Force)`
    overloads alongside the existing matrix forms.
  - `spatial/inertia.hpp`: `SpatialInertia` with separable storage (mass / COM /
    inertia about COM), `matrix6()` for the 6×6 form, composite-body
    `operator+`, and an SE(3) transform that moves each parameter cleanly.
  - `spatial/plucker.hpp`: `motion_plucker()` / `force_plucker()` naming the
    SE(3) adjoint and its dual in Featherstone language.
  - 22 new tests (47 total green, no compiler warnings, clang-tidy clean):
    duality of motion/force adjoints, Jacobi identity for the typed cross,
    kinetic-energy identity for `SpatialInertia`, separable inertia transform
    agrees with the 6×6 congruence, composite-body linearity, Plücker = adjoint.
  - `docs/ALGORITHMS.md`: convention reference for the angular-first / body-fixed
    spatial-algebra layer.
  - Course chapter 05 (5 sub-chapters + exercises).

- **Phase 1 — Lie groups SO(3) and SE(3).**
  - `core/types.hpp`: concrete `double` algebra aliases (Vector3/6, Matrix3/4/6,
    Quaternion). Spatial 6-vectors are angular-first (ω; v).
  - `liegroup/so3.hpp`: `SO3` (canonical w≥0 quaternion) with exp/log, right/left
    Jacobians and inverses, `skew`/`unskew`. Quaternion-based `log`, stable at θ=π.
  - `liegroup/se3.hpp`: `SE3` with exp/log, 6×6 adjoint and inverse, and SE(3)
    group Jacobians via Barfoot's Q matrix (angular-first).
  - `spatial/cross.hpp`: `cross_motion` and `cross_force` spatial cross products.
  - `src/examples/se3_basics.cpp`: worked transform-composition example.
  - 25 unit tests (group axioms, exp/log round-trips to 1e-10, π-angle, FD
    Jacobians, adjoint identity, Jacobi identity). Course chapters 03 & 04.

- **Phase 0 — Bootstrap.** Repo skeleton that compiles, tests, and renders the
  course locally.
  - Top-level `CMakeLists.txt` (header-only `tinyspatial` INTERFACE target) and
    `CMakePresets.json` with `debug` (ASan+UBSan), `release`, and `validation`
    presets.
  - `.gitignore`, Apache-2.0 `LICENSE`, `.clang-format`, `.clang-tidy`,
    `.editorconfig`.
  - `third_party/` submodules pinned by SHA: Eigen 3.4.0, GoogleTest,
    Google Benchmark, tinyxml2, urdfdom_headers, nanobind.
  - Placeholder `include/tinyspatial/version.hpp` + `tests/unit/test_version.cpp`.
  - `docker/` skeleton (builder, runtime, validation-oracle stub).
  - `.github/workflows/ci.yml`: configure → build → test on Ubuntu 22.04 / GCC 12.
  - Course welcome: `course/README.md`, `00_welcome/` chapters, `mkdocs.yml`.
