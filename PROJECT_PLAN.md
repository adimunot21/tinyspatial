# PROJECT_PLAN.md — `tinyspatial`

> **What** we are building and **when**. `CLAUDE.md` covers *how*.

---

## 1. Vision

A header-mostly C++20 library that re-implements the core rigid-body kinematics and dynamics algorithms from Pinocchio, with:

- Lie-group operations on SO(3) and SE(3)
- Spatial vectors, inertias, and Plücker algebra
- Featherstone **RNEA**, **ABA**, **CRBA**
- Geometric and analytical Jacobians, plus analytical derivatives
- Damped-least-squares and null-space-projected inverse kinematics
- Optional differentiable IK
- Python bindings via nanobind
- A URDF loader using upstream `urdfdom_headers`

…validated numerically against **Pinocchio 3.9** to `1e-10` on the Franka FR3, UR5e, and SO-ARM101.

## 2. Success criteria (resume metric)

> "Open-source C++20 rigid-body dynamics library (≥ 250 GitHub stars, ~1.6 K LoC of unit tests, full-build CI < 90 s). RNEA / ABA / CRBA results numerically match Pinocchio 3.9 to `1e-10` on the Franka FR3, UR5e, and SO-ARM101 URDFs. On a 7-DoF Franka model, achieves > 6M RNEA evaluations / second / core on x86, within 1.4× of Pinocchio's hand-tuned implementation."

This sentence is the contract. Every milestone below ladders up to one of its clauses.

**Current status (v0.1.0) — honest accounting of the contract:**

| Clause | Status |
| ------ | ------ |
| Pinocchio 3.9 parity to `1e-10` (RNEA/ABA/CRBA + FK/Jacobian/derivatives) | ✅ met — measured at `1e-13`–`1e-15`, see `docs/PINOCCHIO_PARITY.md` |
| ~1.6 K LoC of unit tests | ✅ met |
| Full-build CI green | ✅ met (matrix in `.github/workflows/ci.yml`) |
| CI < 90 s | ⚠️ holds for the fast matrix; the Pinocchio cross-check runs in a separate `validation.yml` outside that budget |
| ≥ 6 M RNEA / s / core, within 1.4× of Pinocchio | ❌ **not yet met** — currently **~390 K/s, ~1.6×** of Pinocchio C++ on Franka (`docs/BENCHMARKS.md`). Treated as a stretch target; the planned fixed-size / compile-time (`Scalar`-generic) work is the path to closing it. |
| ≥ 250 GitHub stars | ⏳ post-launch metric |

The `≥ 6 M/s` figure is an **aspiration**, not a delivered number. Keep it
labelled as such anywhere it appears publicly until it is actually measured.

## 3. Audience model

| Audience              | What they're looking for                                                             | Where we serve them                       |
| --------------------- | ------------------------------------------------------------------------------------ | ----------------------------------------- |
| Senior C++ reviewer   | Clean code, CI green, benchmarks, Pinocchio parity                                   | `/include`, `/src`, `/tests`, `/benchmarks`, `README` |
| Hiring manager        | The resume metric in §2 verified at a glance                                          | `README`, `docs/BENCHMARKS.md`, `docs/PINOCCHIO_PARITY.md` |
| Total beginner        | A guided tour from "what is a robot" to "I just wrote a Jacobian"                     | `/course`                                 |
| Curious open-source contributor | Issues, contribution guide, small first-issue tags                            | `docs/DEVELOPMENT.md`, GitHub Issues       |

## 4. Phase plan (12 months @ ~10–15 hrs/week)

Each phase has: **duration**, **deliverables**, **acceptance criteria**, **library work**, **course work**, **risks**.

If you (Claude Code) finish a phase early, do not skip ahead — instead, harden tests, add benchmarks, or polish the course chapter. Quality > speed.

---

### Phase 0 — Bootstrap (Weeks 0–1)

**Goal**: an empty but well-formed repo that compiles "hello world," runs CI, and renders the course site locally.

**Library work**

- Initialise git repo with `.gitignore`, Apache-2.0 `LICENSE`, top-level `README.md`.
- Top-level `CMakeLists.txt` and `CMakePresets.json` (debug, release, validation).
- `.clang-format`, `.clang-tidy`, `.editorconfig`.
- `third_party/` submodules added and pinned: Eigen 3.4.0, GoogleTest, Google Benchmark, tinyxml2, urdfdom_headers, nanobind.
- One placeholder header (`include/tinyspatial/version.hpp`) + one placeholder test that just asserts the version string.
- `docker/Dockerfile` skeleton (builder + runtime stages; validation-oracle stage stub).
- `.github/workflows/ci.yml` minimal: configure → build → test on Ubuntu 22.04 GCC 12.

**Course work**

- `course/README.md` — course index, learning path diagram.
- `course/00_welcome/README.md` — what is this, who is it for, what will you learn.
- `course/00_welcome/01_what_is_a_robot.md` — gentle motivation chapter.
- `course/00_welcome/02_setup_your_machine.md` — install Docker, clone, build, run tests. Aimed at someone who has never opened a terminal.
- `course/mkdocs.yml` — mkdocs-material config.

**Acceptance criteria**

- `git clone` → `git submodule update --init` → `cmake --preset=debug` → `ctest` works on a fresh Ubuntu 24.04 VM with no prior setup beyond `apt install build-essential cmake git`.
- CI is green.
- `mkdocs serve` renders the welcome chapter.

**Risks**

- Submodule pinning to specific tags can lag CI if tags move. **Mitigation**: pin to SHA, not tag.

---

### Phase 1 — Lie groups SO(3) and SE(3) (Weeks 2–5)

**Goal**: a correct, tested, and *documented* implementation of the two Lie groups every rigid-body algorithm depends on.

**Library work**

- `include/tinyspatial/core/types.hpp`: type aliases (`Scalar`, `Vector3`, `Matrix3`, `Vector6`, `Matrix6`, `Quaternion`).
- `include/tinyspatial/liegroup/so3.hpp`:
  - `SO3` class wrapping a quaternion (storage) with a 3×3 rotation matrix accessor.
  - `exp(omega) -> SO3`, `log(R) -> Vector3`.
  - Right- and left-Jacobians: `Jr(omega)`, `Jl(omega)`, plus inverses.
  - `*` (composition), `inverse`, `act(Vector3) -> Vector3`.
- `include/tinyspatial/liegroup/se3.hpp`:
  - `SE3` class storing `SO3 rotation; Vector3 translation;`.
  - `exp(xi) -> SE3` where `xi = (omega; v)` ∈ ℝ⁶.
  - `log(T) -> Vector6`.
  - Right- and left-Jacobians on SE(3).
  - `*` (composition), `inverse`, `act(Vector3) -> Vector3`.
  - Adjoint and its inverse.
- `include/tinyspatial/spatial/cross.hpp`: spatial cross products `cross_motion` (6×6) and `cross_force`. (These belong here because they're tightly coupled to SE(3) adjoint.)

**Tests**

- Closed-form sanity: identity, small-angle, π-angle (the tricky one for `log`).
- Group axioms (associativity, identity, inverse).
- `exp(log(T)) ≈ T` and `log(exp(xi)) ≈ xi` to `1e-12`.
- Adjoint identities.
- Numerical Jacobians vs analytical: finite-difference matches to `1e-7`.

**Course work**

- `course/03_rotations_and_transforms/` — 5 chapters from "what is a rotation" to "matrices, quaternions, axis-angle, how they relate."
- `course/04_lie_groups/` — 6 chapters: the manifold idea, tangent space, exp/log intuition, Jacobians, the adjoint, why we need all this for robots.

**Acceptance criteria**

- All Phase-1 unit tests green.
- `clang-format` clean, `clang-tidy` clean.
- Course chapters 03 and 04 have "Where this lives in the library" sections.
- One worked example in `src/examples/se3_basics.cpp` that prints the result of composing two `SE3` transforms.

**Risks**

- The `log` near π is numerically delicate. **Mitigation**: copy Pinocchio's stable formulation and cite the source in a header comment.

---

### Phase 2 — Spatial algebra (Weeks 6–9)

**Goal**: data structures for spatial vectors (twists, wrenches), Plücker transforms, and spatial inertias — the "vocabulary" of Featherstone's notation.

**Library work**

- `include/tinyspatial/spatial/motion.hpp`: `Motion` (a.k.a. twist), ℝ⁶ with angular and linear parts.
- `include/tinyspatial/spatial/force.hpp`: `Force` (wrench).
- `include/tinyspatial/spatial/inertia.hpp`: `SpatialInertia` — mass, COM, 3×3 rotational inertia; conversions to/from 6×6.
- `include/tinyspatial/spatial/plucker.hpp`: 6×6 Plücker transform built from an `SE3`.
- Operators: motion-on-motion cross, motion-on-force cross, inertia-times-motion, transform-times-motion/force/inertia.

**Tests**

- Inertia transformations vs hand-computed examples.
- Plücker transform of a unit twist matches the SE(3) adjoint times the twist.
- Cross-product identities (Jacobi identity, antisymmetry).

**Course work**

- `course/05_spatial_algebra/` — 5 chapters: why 6-vectors, twists & wrenches, Plücker, spatial inertia, "how Featherstone thinks."

**Acceptance criteria**

- Unit tests green.
- A short note in `docs/ALGORITHMS.md` documenting our spatial-vector convention (body-fixed; angular-first).

---

### Phase 3 — Kinematic tree + URDF loader (Weeks 10–13)

**Goal**: build a `Model` (a kinematic tree) by reading a URDF.

**Library work**

- `include/tinyspatial/model/joint.hpp`: joint variants — `JointRevolute`, `JointPrismatic`, `JointFixed`, `JointFloating`. Use `std::variant` (or a hand-rolled tagged union if visit overhead matters).
- `include/tinyspatial/model/model.hpp`:
  - `Model` holds: vector of joints, vector of link names, parent indices, child indices, joint placements (`SE3` from parent to joint frame), link inertias.
  - `Data` holds the per-configuration scratchpad (per-link transforms, velocities, accelerations). This separation mirrors Pinocchio.
- `include/tinyspatial/urdf/urdf_loader.hpp`:
  - `Model build_model_from_urdf(const std::string& xml)`.
  - Uses `urdfdom_headers` types + `tinyxml2` for parsing.
  - Mesh paths handled but meshes themselves not loaded (we're not a renderer).
- Three fixture URDFs in `data/robots/`: `franka_fr3.urdf`, `ur5e.urdf`, `so_arm101.urdf`. Download licences cleanly; document provenance in `data/robots/README.md`.

**Tests**

- Loader round-trip: parse a URDF, inspect joint count / link count / DOF, match expectations.
- Fuzz test: feed mutated XML (drop tags, swap attributes) and assert we either parse or return a clean error — never crash.
- Joint frame placements at `q = 0` match a manually computed reference.

**Course work**

- `course/06_kinematic_trees/` — 4 chapters: links, joints, parent/child, the tree as a data structure, the model/data split.
- `course/07_urdf_robot_models/` — 5 chapters: XML, URDF tags, inertias, the loader, exercises with all three robots.

**Acceptance criteria**

- All three fixture URDFs load without errors.
- Fuzz test runs 1000 mutated inputs without a crash.

**Risks**

- URDF dialects (xacro, ROS-specific tags) — **don't support them**, we read pure URDF.

---

### Phase 4 — Forward kinematics + Jacobians (Weeks 14–17)

**Goal**: given `q`, compute every link's pose; given `q` and a link, compute the geometric Jacobian.

**Library work**

- `include/tinyspatial/algo/forward_kinematics.hpp`:
  - `void forward_kinematics(const Model&, Data&, Eigen::Ref<const VectorXd> q)`.
- `include/tinyspatial/algo/jacobian.hpp`:
  - `void compute_jacobian(const Model&, Data&, int link_id, Eigen::Ref<Matrix6Xd> J)`.
  - Variants: local-frame J, world-frame J, local-world-aligned J. Document each — convention mismatch is the #1 cause of Pinocchio parity bugs.
- **Begin the Pinocchio validation harness** — `tests/validation/test_kinematics.py` cross-checks FK and Jacobians on all three robots, 1000 random `q`, all three Jacobian frames.

**Tests**

- FK on `q = 0` matches URDF-declared origins.
- FK + Jacobian numerical agreement with Pinocchio < `1e-10`.

**Course work**

- `course/08_forward_kinematics/` — 4 chapters.
- `course/09_jacobians/` — 5 chapters including geometric vs analytical, local vs world, the singularity intuition.

**Acceptance criteria**

- Validation suite passes for FK and J on Franka, UR5e, SO-ARM101.
- First entry in `docs/PINOCCHIO_PARITY.md`.

---

### Phase 5 — Featherstone dynamics: RNEA, ABA, CRBA (Weeks 18–23)

**The biggest phase. Don't rush it.** This is the credibility centrepiece.

**Library work**

- `include/tinyspatial/algo/rnea.hpp` — Recursive Newton-Euler (inverse dynamics). Forward pass for velocities/accelerations, backward pass for forces.
- `include/tinyspatial/algo/aba.hpp` — Articulated Body Algorithm (forward dynamics).
- `include/tinyspatial/algo/crba.hpp` — Composite Rigid Body Algorithm (mass matrix `M(q)`).
- Internal helpers in `include/tinyspatial/algo/internal/` — kept out of the public API until they stabilise.

**Tests**

- RNEA at `(q, v=0, a=0)` returns the gravity-compensation torque (sanity).
- RNEA × M⁻¹ = ABA (cross-check ourselves before cross-checking Pinocchio).
- Pinocchio validation: all three algorithms, all three robots, `1e-10` tolerance.

**Course work**

- `course/10_dynamics_RNEA/` — 6 chapters culminating in a worked 2-DoF arm example.
- `course/11_ABA_and_CRBA/` — 5 chapters.

**Acceptance criteria**

- All three algorithms green against Pinocchio.
- `docs/PINOCCHIO_PARITY.md` shows the full table.
- First benchmark file `benchmarks/bench_rnea.cpp` exists and prints (unoptimised) throughput.

**Risks**

- Featherstone conventions differ across textbooks. **Mitigation**: pick one (Featherstone 2008, body-fixed) and apply mechanically. If you derive something different from Pinocchio, suspect convention, not bug.

---

### Phase 6 — Analytical derivatives (Weeks 24–29)

**Goal**: closed-form derivatives `∂τ/∂q`, `∂τ/∂v`, `∂τ/∂a` (RNEA derivatives), and `∂FK/∂q`.

**Library work**

- `include/tinyspatial/diff/fk_derivatives.hpp`
- `include/tinyspatial/diff/rnea_derivatives.hpp` — follow the Carpentier & Mansard (2018) paper closely; cite line-for-line.
- Optional: `TINYSPATIAL_BUILD_CODEGEN=ON` to wire CppADCodeGen for autodiff-based cross-check.

**Tests**

- Analytical vs finite difference < `1e-7`.
- Analytical vs Pinocchio's analytical derivatives < `1e-10`.

**Course work**

- (Defer chapter 13 to after IK; build the math chapter here as `course/10b_rnea_derivatives.md`, an "advanced" optional read.)

**Acceptance criteria**

- Validation suite green.

---

### Phase 7 — Inverse kinematics (Weeks 30–33)

**Library work**

- `include/tinyspatial/ik/dls.hpp` — damped least squares.
- `include/tinyspatial/ik/nullspace.hpp` — task-priority IK with null-space projection.
- `include/tinyspatial/ik/differentiable.hpp` — IK that returns `∂q*/∂x_target` using the implicit function theorem.

**Tests**

- DLS converges on a reachable target for all three robots from 100 random initial seeds.
- Null-space tracks a secondary objective (e.g. elbow-up).
- Differentiable IK gradient matches finite difference to `1e-6`.

**Course work**

- `course/12_inverse_kinematics/` — 6 chapters.
- `course/13_differentiable_ik/` — 4 chapters.

---

### Phase 8 — Python bindings + examples (Weeks 34–37)

**Library work**

- `src/bindings/` — nanobind glue for `SO3`, `SE3`, `Model`, `Data`, all algorithms.
- `python/tinyspatial/__init__.py` plus type stubs (`*.pyi`).
- `python/examples/` — three Jupyter notebooks:
  1. Forward kinematics tour on a UR5e.
  2. RNEA vs Pinocchio side-by-side on Franka.
  3. IK to a Cartesian target with null-space elbow control.

**Tests**

- `python/tests/test_parity.py` — Python-side parity check vs `pinocchio`. (Belt-and-braces; C++ already has this.)

**Course work**

- `course/14_python_bindings/` — 4 chapters.

---

### Phase 9 — Benchmarks + polish (Weeks 38–45)

**Library work**

- `benchmarks/` — one file per algorithm. Tracks: ops/sec, ops/sec/core, allocations per call.
- Optimisation pass on RNEA / ABA / CRBA. Target: within 1.4× of Pinocchio on Franka. Order of attack:
  1. Confirm fixed-size Eigen types throughout the hot path.
  2. Eliminate any heap allocation in the per-call path.
  3. Inline the inner loop functions; check `perf stat`.
  4. Look at the disassembly; do anything obvious (e.g. vectorisable loops Eigen isn't catching).
- `docs/BENCHMARKS.md` auto-generated by CI on tagged releases.

**Course work**

- `course/15_validation_vs_pinocchio/` — 3 chapters.
- `course/16_benchmarking/` — 4 chapters; teach `perf`, `valgrind --tool=callgrind`, the Google Benchmark API.

**Acceptance criteria**

- Headline benchmark on README cleared.
- CI < 90 s end-to-end.

---

### Phase 10 — Release & marketing (Weeks 46–52)

- Cut **v0.1.0**.
- README polish: badges, GIF / SVG of a robot moving (rendered offline, committed as static), the resume metric prominently.
- Blog post (~2000 words): "I re-implemented Pinocchio in 5K lines of C++20 to learn rigid-body dynamics."
- Submit PR to [`awesome-robotics-libraries`](https://github.com/jslee02/awesome-robotics-libraries).
- Deploy `course/` site to GitHub Pages with a custom subdomain if available.
- Post to: r/robotics, r/cpp, Hacker News (Saturday morning Pacific is the lore).

## 5. Cross-reference table: source → course

This is the master map. Update it as you go.

| Library file                                | Course chapter             |
| ------------------------------------------- | -------------------------- |
| `include/tinyspatial/liegroup/so3.hpp`      | `course/04_lie_groups/`    |
| `include/tinyspatial/liegroup/se3.hpp`      | `course/04_lie_groups/`    |
| `include/tinyspatial/spatial/*`             | `course/05_spatial_algebra/` |
| `include/tinyspatial/model/*`               | `course/06_kinematic_trees/` |
| `include/tinyspatial/urdf/*`                | `course/07_urdf_robot_models/` |
| `include/tinyspatial/algo/forward_kinematics.hpp` | `course/08_forward_kinematics/` |
| `include/tinyspatial/algo/jacobian.hpp`     | `course/09_jacobians/`     |
| `include/tinyspatial/algo/rnea.hpp`         | `course/10_dynamics_RNEA/` |
| `include/tinyspatial/algo/aba.hpp`          | `course/11_ABA_and_CRBA/`  |
| `include/tinyspatial/algo/crba.hpp`         | `course/11_ABA_and_CRBA/`  |
| `include/tinyspatial/ik/*`                  | `course/12_inverse_kinematics/`, `course/13_differentiable_ik/` |
| `src/bindings/`                             | `course/14_python_bindings/` |
| `tests/validation/`                         | `course/15_validation_vs_pinocchio/` |
| `benchmarks/`                               | `course/16_benchmarking/`  |

## 6. Validation strategy (recap)

- Oracle: Pinocchio 3.9.0 from conda-forge.
- Tests live in `tests/validation/` (Python).
- Tolerance: `1e-10` absolute, `1e-10` relative. Never relaxed without a written justification in `docs/PINOCCHIO_PARITY.md`.
- 1000 random `(q, v, a)` per algorithm per robot, fixed seed.
- Conventions: where Pinocchio and we differ (e.g. Jacobian frame), document and apply a converter in the test, not in the library.

## 7. Course architecture

Numbered, sequential. Each chapter is its own folder. Each chapter has:

- `README.md` — the chapter itself.
- `exercises.md` — hands-on exercises with hint sections.
- `solutions/` (optional) — solutions, gitignored from the rendered site if instructors want to keep them out.
- "Where this lives in the library" closing section.

For chapters 00–02 (welcome, C++ foundations, linear algebra), we **link out** to external resources and provide only a thin layer of original framing and exercises. From chapter 03 onward, content is original because the gap in free material is real.

## 8. Scope discipline — what we are NOT building

- **Collision / contact** — no FCL, no HPP-FCL. Out of scope.
- **Path planning** — out of scope.
- **Simulation** — no integrator, no contact solver.
- **Mesh visualisation** — we read mesh paths from URDFs but don't load meshes.
- **ROS integration** — explicitly forbidden (see `CLAUDE.md` §8).

If the maintainer requests any of these, push back and propose a follow-up project instead.

## 9. Open questions for the maintainer to decide later

These will come up; flag them when they do, don't decide unilaterally.

1. **Author / org name** on the GitHub URL (affects README badges and the awesome-robotics PR).
2. **Custom domain** for the course site, or just `<user>.github.io/tinyspatial/`.
3. **CppADCodeGen** — keep optional, or drop entirely to reduce surface area? Decide at Phase 6 entry.
4. **License of fixture URDFs** — Franka and UR5e are typically Apache-2.0 or BSD-3, but check each. SO-ARM101 should be redistribution-friendly.
5. **Blog post host** — personal site, Substack, Medium, or just the course site itself.

## 10. Glossary of recurring terms

- **Pinocchio**: the reference C++ rigid-body dynamics library we validate against.
- **RNEA**: Recursive Newton-Euler Algorithm; inverse dynamics.
- **ABA**: Articulated Body Algorithm; forward dynamics.
- **CRBA**: Composite Rigid Body Algorithm; mass matrix.
- **Featherstone**: Roy Featherstone, author of *Rigid Body Dynamics Algorithms*.
- **Spatial vector**: a 6-vector packing angular and linear parts.
- **Twist**: a spatial motion vector.
- **Wrench**: a spatial force vector.
- **SE(3)**: the Special Euclidean group in 3D (rigid transforms).
- **SO(3)**: the Special Orthogonal group in 3D (rotations).
- **Plücker transform**: the 6×6 representation of an SE(3) transform acting on spatial vectors.
- **DLS**: Damped Least Squares (IK).

---

*End of PROJECT_PLAN.md.*
