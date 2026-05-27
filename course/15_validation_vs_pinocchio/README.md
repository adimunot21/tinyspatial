# 15 · Validating against Pinocchio

You've written your own implementation of FK, RNEA, ABA, CRBA. How do you
*know* it's right?

You could:

1. Write unit tests against hand-computed values. Works for a 2-DoF arm;
   intractable for a 7-DoF arm with random configurations.
2. Compare to an analytical solution. Doesn't exist for most robot
   models.
3. Compare to a peer-reviewed reference implementation that the field
   already trusts.

This library chooses option 3. The reference is **Pinocchio 3.9.0** —
the de-facto standard rigid-body library in robotics research, maintained
by INRIA. If `tinyspatial` and Pinocchio agree to machine precision on
1000 random configurations of every fixture robot, then either we both
have the same bug (unlikely — Pinocchio is heavily peer-reviewed and used
in production) or we're both right.

This chapter is about the *discipline* of doing that comparison: how to
set it up, what the convention mismatches look like, and how to read the
output.

## What you'll learn

- Why "validate against an oracle" is the right pattern for a
  portfolio re-implementation, and what makes a good oracle.
- How to set up the Python-level cross-check (`tests/validation/`)
  without contaminating the standalone C++ build.
- The convention mismatches between `tinyspatial` and Pinocchio that
  feel like bugs but are by design — and how to handle them at the
  boundary.
- How to read [`docs/PINOCCHIO_PARITY.md`](../../docs/PINOCCHIO_PARITY.md):
  what each row means, when 1e-13 vs 1e-15 matters, and what
  triggers a regeneration.

## The chapters

1. [The oracle approach](01_the_oracle_approach.md) — why we validate against
   Pinocchio specifically, the rules for picking an oracle, and the build
   strategy that keeps the validation dependency from leaking into the
   library.
2. [Convention mismatches are bugs in disguise](02_convention_mismatches.md) —
   angular-first vs linear-first, the universe-joint indexing offset,
   Jacobian row order, quaternion sign, gravity vector. Each one a
   story of "I thought it was a numerics bug; it was a convention
   difference."
3. [Reading the parity table](03_reading_the_parity_table.md) — what each row
   in `docs/PINOCCHIO_PARITY.md` means, what a 1e-13 vs 1e-15 error tells
   you, the difference between "tolerance" and "noise floor," and the
   anatomy of [`tests/validation/test_kinematics.py`](../../tests/validation/test_kinematics.py).

## Where this lives in the library

| Concept                       | File path                                |
| ----------------------------- | ---------------------------------------- |
| Validation test driver        | [`tests/validation/test_kinematics.py`](../../tests/validation/test_kinematics.py) |
| Python-side parity (pytest)   | [`python/tests/test_parity.py`](../../python/tests/test_parity.py) |
| Committed parity numbers      | [`docs/PINOCCHIO_PARITY.md`](../../docs/PINOCCHIO_PARITY.md) |
| Build flag for Pinocchio link | `TINYSPATIAL_BUILD_VALIDATION=ON` (see [CLAUDE.md §8](../../CLAUDE.md)) |
