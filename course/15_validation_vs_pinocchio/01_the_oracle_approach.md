# The oracle approach

Re-implementing an algorithm raises a deep credibility problem:
*how is a fresh implementation known to be correct?*

For Featherstone's RNEA on a 7-DoF arm, there's no closed-form solution
to compare against. The algorithm has ~40 floating-point operations per
joint, the chain has 7 joints, and the answer changes for every random
configuration. Eyeballing the output for plausibility is useless. Hand-
computing the truth value is intractable.

The way out is an **oracle**: a peer-reviewed reference
implementation that the field already trusts. Write the new
implementation, run both on the same input, compare to machine precision,
ship if they agree. If they disagree, the oracle is right and the new
implementation has a bug.

This chapter covers why Pinocchio is the chosen oracle and how to
make the oracle pattern work without compromising the library under
construction.

## What makes a good oracle

Three criteria. In order:

1. **Trusted by the field.** Has been cited, reproduced, used in
   production projects for years. Pinocchio is the default in ROS 2 /
   Drake / OCS2; it's used by INRIA, ETH Zürich, Open Robotics, NASA
   JPL. A major bug would have surfaced by now.
2. **Open source.** Reading the source is necessary when convention
   mismatches arise. (This *will* happen — see the next
   chapter.) Closed-source oracles are not viable.
3. **Has a Python API.** The comparison runs from a script that
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
- **Hand-written reference** — the temptation. Avoid this. A hand-
  written reference shares the same bugs as the implementation,
  because the same author wrote both.

The oracle is **Pinocchio 3.9.0**, pinned exactly. When Pinocchio
4.0 ships, re-validation against it follows; in the meantime, pinning means
nightly-build drift can't sneak parity regressions in.

## What "validation" provides and what it doesn't

It provides:

- **Confidence in the algorithm.** Output that matches a trusted
  oracle on 1000 random configurations to 1e-13 is a
  correctly implemented algorithm. Full stop.
- **A regression alarm.** Any future change that breaks parity surfaces
  immediately. The validation suite runs in CI; a PR that takes the
  RNEA-Franka parity from 1e-14 to 1e-3 is rejected before review.
- **A teaching tool.** Reading the parity table next to the convention-
  mismatch notes is how "angular-first" becomes concrete.

It does *not* provide:

- **Proof of mathematical correctness.** Both libraries could be
  wrong in the same way. (Unlikely for RNEA — it's been validated
  to death — but in principle possible.)
- **Proof of API correctness.** Two libraries with identical RNEA
  output can still have wildly different ergonomics. Validation
  checks numerics, not interface.
- **A speed benchmark.** Parity tests measure runtime against the
  oracle, but that's a separate concern from correctness. See
  [chapter 16](../16_benchmarking/) for benchmarking.

The parity test is a *correctness* check, run cheaply, that catches
real bugs. It is a non-negotiable gate, not a nice-to-have.

## The build-isolation problem

Pinocchio depends on Boost. Boost is enormous, complex, and exactly
the kind of dependency `tinyspatial` excludes by design. The goal is
a library that compiles standalone with only header-only / vendored
dependencies — a credibility signal for senior C++ reviewers.

The question, then: how can Pinocchio serve as a validation oracle
without becoming a dependency of the library?

**Solution: build-flag isolation.**

```cmake
option(TINYSPATIAL_BUILD_VALIDATION "Build the Pinocchio cross-check" OFF)
```

The default build (`cmake --preset=release`) doesn't see Pinocchio at all.
The standalone C++ library compiles with nothing but Eigen,
urdfdom_headers, and tinyxml2 — all header-only.

Only with the flag set (`cmake --preset=validation`) does the build
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
production library binary.** It only touches the test harness. The result
is the credibility of validating against the field's reference *without*
inheriting Pinocchio's dependency soup.

This is the same pattern Drake uses to validate against analytical
results, and the same pattern Tensorflow uses to validate against
numpy: a separate test process loads both, the build artefact loads
neither.

## Reproducing the validation locally

With a Python venv that has `pinocchio` installed:

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
committed table of max-abs differences per algorithm × robot. If a
change pushes any cell above 1e-10, the test fails and the table
identifies which one.

## The bigger lesson

Validating against an oracle is the right pattern for *any* re-
implementation project. A parser is compared against a reference
parser. A re-implemented ML model runs side-by-side with the published
checkpoint. A physics simulator is compared to a hand-derived solution on
a 2-body problem.

The cost is low: one oracle dependency, a comparison script, a
committed table. The payoff is enormous: the implementation can be proven
correct, not merely argued correct.

The next chapter covers what happens when the comparison *fails* and
the cause turns out to be a convention difference, not a bug.

> ## Where this lives in the library
>
> | Concept                       | File path                                |
> | ----------------------------- | ---------------------------------------- |
> | Build flag                    | [`CMakeLists.txt`](../../CMakeLists.txt) (search `TINYSPATIAL_BUILD_VALIDATION`) |
> | The validation driver         | [`tests/validation/test_kinematics.py`](../../tests/validation/test_kinematics.py) |
> | The committed table           | [`docs/PINOCCHIO_PARITY.md`](../../docs/PINOCCHIO_PARITY.md) |
