# Validating against Pinocchio

Up to here we've been writing code that *looks* right. Now we ask the harder
question: **is it right against the reference?**

Open [`docs/PINOCCHIO_PARITY.md`](../../docs/PINOCCHIO_PARITY.md). The number
that matters for forward kinematics is the **FK max-abs diff** column. On
every fixture URDF it's on the order of `1e-15` — *one floating-point ULP*.

## What that number actually means

The harness does this, for each fixture URDF and for 1000 random `q`s:

1. Load the URDF in **Pinocchio 3.9.0** and in our library.
2. Run forward kinematics in each.
3. For every joint, take the absolute max element of
   `pinocchio_pose - tinyspatial_pose` (4×4 matrices).
4. Track the worst across all joints, all configurations.

`1e-15` means: across all 7 (FR3) or 6 (UR5e) or 5 (SO-ARM) joints, across
1000 random `q`'s, the maximum element of the homogeneous-matrix difference
is at most a few ULPs of a `double`. Effectively bit-identical.

The validation tolerance is `1e-10`. These results are five orders of
magnitude better than that — the specification accepted "within `1e-10`";
the implementation delivers "within machine precision."

## Why this is the right test, not just a victory lap

It's tempting to think the FK code is so simple it doesn't need cross-checking.
Two things would have failed silently without this harness:

1. **A rotation convention mismatch.** If our `<origin rpy>` interpretation
   disagreed with Pinocchio's by an `Rxyz` vs `Rzyx` ordering, every random
   `q` would produce a different pose. The `1e-15` agreement says we got the
   convention right.

2. **A composition order mismatch.** Multiplying `parent * local` vs
   `local * parent` is the difference between "rotate then translate" and
   "translate then rotate" — same answer for some special `q`'s, different
   for almost all of them. 1000 random `q`'s would smoke this out.

## What the *Jacobian* part of the table proves

The other three columns (Jacobian: LOCAL / WORLD / LWA) also land at
`1e-15`. They are the topic of chapter 09. Spoiler: the Jacobian is the
derivative of FK, so it can only be that close if FK is also correct. The
two columns being simultaneously at machine precision is independent
evidence the whole stack is right.

## Running it yourself

```bash
# One-time: install the Python deps into the project venv.
.venv/bin/pip install -r tests/validation/requirements.txt

cmake --preset=validation
cmake --build build/validation -j
ctest --preset=validation -L pinocchio_parity --output-on-failure
```

The same test re-runs every time you touch FK or Jacobian code. If a future
refactor introduces a sign-flip or a wrong matrix transpose, the harness
fails *immediately* instead of months later when an integrator silently
diverges. That's the whole point.

## Where this lives in the library

| Concept | File |
| ------- | ---- |
| The parity table | [`docs/PINOCCHIO_PARITY.md`](../../docs/PINOCCHIO_PARITY.md) |
| Harness | [`tests/validation/test_kinematics.py`](../../tests/validation/test_kinematics.py) |
| Bindings to call our code from Python | [`src/bindings/main.cpp`](../../src/bindings/main.cpp) |
| Python deps | [`tests/validation/requirements.txt`](../../tests/validation/requirements.txt) |
| Convention reference | [`docs/ALGORITHMS.md`](../../docs/ALGORITHMS.md) |

Next: the [exercises](exercises.md), then [chapter 09 — Jacobians](../09_jacobians/README.md).
