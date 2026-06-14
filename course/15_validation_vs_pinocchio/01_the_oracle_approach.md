# The oracle approach

When you re-implement an algorithm, you have a deep credibility problem:
*how do you know your version is right?*

For Featherstone's RNEA on a 7-DoF arm, there's no closed-form solution
to compare against. The algorithm has ~40 floating-point operations per
joint, the chain has 7 joints, and the answer changes for every random
configuration. Eyeballing the output for plausibility is useless. Hand-
computing the truth value is intractable.

The way out is to use an **oracle**: a peer-reviewed reference
implementation that the field already trusts. You write your version,
run both on the same input, compare to machine precision, ship if they
agree. If they disagree, the oracle is right and you have a bug.

This chapter is about why we picked Pinocchio specifically and how to
make the oracle pattern work without compromising the library you're
trying to build.

## What makes a good oracle

Three criteria. In order:

1. **Trusted by the field.** Has been cited, reproduced, used in
   production projects for years. Pinocchio is the default in ROS 2 /
   Drake / OCS2; it's used by INRIA, ETH Zürich, Open Robotics, NASA
   JPL. If it has a major bug, the world would have found it by now.
2. **Open source.** You need to read the source when convention
   mismatches confuse you. (Spoiler: this *will* happen — see the next
   chapter.) Closed-source oracles are not viable.
3. **Has a Python API.** You're going to compare from a script that
   feeds the same inputs to both libraries and diffs the outputs. Doing
   that in C++ is possible but painful; in Python it's a 20-line script.

Pinocchio scores 3/3. The alternatives:

- **RBDL** (Rigid Body Dynamics Library) — trusted but smaller community,
  Python bindings exist but less polished.
- **Drake** — Drake is huge and great, but its kinematics-and-dynamics
  module is buried in 100K lines of robotics utilities. Hard to isolate.
- **MuJoCo** — physics simulator with rigid-body dynamics underneath, but
  the integration tolerance and the contact handling muddy comparisons
  against algorithm-pure outputs.
- **Hand-written reference** — the temptation. Don't do this. Your hand-
  written reference will have the same bugs as your implementation
  because you wrote both.

We use **Pinocchio 3.9.0**, pinned exactly. When Pinocchio
4.0 ships, we'll re-validate against it; in the meantime, pinning means
nightly-build drift can't sneak parity regressions in.

## What "validation" gives you and what it doesn't

It gives you:

- **Confidence in the algorithm.** If your output matches a trusted
  oracle on 1000 random configurations to 1e-13, your algorithm is
  correctly implemented. Full stop.
- **A regression alarm.** Any future change that breaks parity surfaces
  immediately. The validation suite runs in CI; a PR that takes the
  RNEA-Franka parity from 1e-14 to 1e-3 is rejected before review.
- **A teaching tool.** Reading the parity table next to the convention-
  mismatch notes is how you internalize what "angular-first" actually
  means.

It does *not* give you:

- **Proof of mathematical correctness.** Both libraries could be
  wrong in the same way. (Unlikely for RNEA — it's been validated
  to death — but in principle possible.)
- **Proof of API correctness.** Two libraries with identical RNEA
  output can still have wildly different ergonomics. Validation
  checks numerics, not interface.
- **A speed benchmark.** Parity tests time how slow you are vs the
  oracle, but that's a separate concern from correctness. See
  [chapter 16](../16_benchmarking/) for benchmarking.

The parity test is a *correctness* check, run cheaply, that catches
real bugs. Treat it as a non-negotiable gate, not a nice-to-have.

## The build-isolation problem

Pinocchio depends on Boost. Boost is enormous, complex, and exactly
the kind of dependency `tinyspatial` excludes by design. We
want a library that compiles standalone with only header-only / vendored
dependencies — that's a credibility signal for senior C++ reviewers.

So how do we use Pinocchio for validation without making Pinocchio a
dependency of the library?

**Solution: build-flag isolation.**

```cmake
option(TINYSPATIAL_BUILD_VALIDATION "Build the Pinocchio cross-check" OFF)
```

The default build (`cmake --preset=release`) doesn't see Pinocchio at all.
The standalone C++ library compiles with nothing but Eigen,
urdfdom_headers, and tinyxml2 — all header-only.

Only when you flip the flag (`cmake --preset=validation`) does the build
system go looking for Pinocchio. And it doesn't even *link* Pinocchio to
the library — it uses the *Python* bindings (already in a conda / pip
environment) and runs the comparison from Python:

```bash
cmake --preset=validation
cmake --build build/validation -j
ctest --preset=validation -L pinocchio_parity --output-on-failure
```

The flow:

```
┌──────────────────────────────────────────────────────────────────┐
│  Standalone build path (cmake --preset=release)                  │
│                                                                  │
│    Eigen + urdfdom + tinyxml2  →  libtinyspatial.a               │
│                                                                  │
│    Zero conda. Zero Pinocchio. Zero Boost.                       │
└──────────────────────────────────────────────────────────────────┘

┌──────────────────────────────────────────────────────────────────┐
│  Validation path (cmake --preset=validation)                     │
│                                                                  │
│    Above + nanobind  →  _tinyspatial.so  (Python module)         │
│                                                                  │
│    Then in a Python venv with `pip install pin`:                 │
│      python tests/validation/test_kinematics.py                  │
│         ├── load same URDF in both libs                          │
│         ├── sample 1000 random (q, v, a)                         │
│         ├── call FK/J/RNEA/CRBA/ABA in both                      │
│         └── assert max abs diff < 1e-10                          │
└──────────────────────────────────────────────────────────────────┘
```

The point: **the conda environment with Pinocchio never touches the
production library binary.** It only touches the test harness. So we
get the credibility of validating against the field's reference *without*
inheriting Pinocchio's dependency soup.

This is the same pattern Drake uses to validate against analytical
results, and the same pattern Tensorflow uses to validate against
numpy: a separate test process loads both, the build artefact loads
neither.

## Reproducing the validation locally

Once you have a Python venv with `pinocchio` installed:

```bash
cmake --preset=validation
cmake --build build/validation -j
ctest --preset=validation -L pinocchio_parity --output-on-failure
```

The `-L pinocchio_parity` filter selects only the validation tests.
Other test labels (unit tests, fuzz tests) run in the same build
directory but aren't gated on having Pinocchio available.

When the comparison runs, it regenerates
[`docs/PINOCCHIO_PARITY.md`](../../docs/PINOCCHIO_PARITY.md) — the
committed table of max-abs differences per algorithm × robot. If your
changes pushed any cell above 1e-10, the test fails and the table
shows you which one.

## The bigger lesson

Validating against an oracle is the right pattern for *any* re-
implementation project. Building a parser? Compare against a reference
parser. Re-implementing an ML model? Run the inference side-by-side with
the published checkpoint. Writing a physics simulator? Compare to a
hand-derived solution on a 2-body problem.

The cost is low: one oracle dependency, a comparison script, a
committed table. The payoff is enormous: you can prove your
implementation is correct, not just argue it.

The next chapter is about what happens when the comparison *fails* and
the answer turns out to be a convention difference, not a bug.

> ## Where this lives in the library
>
> | Concept                       | File path                                |
> | ----------------------------- | ---------------------------------------- |
> | Build flag                    | [`CMakeLists.txt`](../../CMakeLists.txt) (search `TINYSPATIAL_BUILD_VALIDATION`) |
> | The validation driver         | [`tests/validation/test_kinematics.py`](../../tests/validation/test_kinematics.py) |
> | The committed table           | [`docs/PINOCCHIO_PARITY.md`](../../docs/PINOCCHIO_PARITY.md) |
