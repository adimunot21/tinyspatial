# Convention mismatches are bugs in disguise

The first time `tinyspatial`'s RNEA output disagreed with Pinocchio's by
a factor of ten, my reaction was "we have a bug in the algorithm." Three
hours of staring at the code later, the actual answer: Pinocchio uses
linear-first spatial-vector ordering, and we use angular-first. Wrong
permutation of the 6-vector → wrong torques out → looks like a math bug,
isn't.

This pattern repeats. Every convention difference between two robotics
libraries looks like a bug the first time you encounter it. This chapter
is a catalogue, so the next time it happens to you (it will) you can
recognize the shape.

## The five mismatches you will hit

Each one has the same shape: the math is the same, but the *layout* of
inputs/outputs differs by a known permutation or sign. The fix is
always at the comparison boundary, never in the algorithm.

### 1. Spatial-vector ordering: angular-first vs linear-first

A 6-D twist (spatial velocity) is `(ω, v)` — angular part and linear
part. The question is: do you store ω first or v first?

| Library      | Convention   | A twist looks like         |
| ------------ | ------------ | -------------------------- |
| `tinyspatial` | angular-first | `[ωx, ωy, ωz, vx, vy, vz]` |
| Pinocchio    | linear-first  | `[vx, vy, vz, ωx, ωy, ωz]` |

This is Featherstone's vs Spatial Vector Calculus's preference. Both
authors wrote definitive textbooks; neither convention is "wrong."

The fix at the validation boundary is a 6×6 permutation matrix:

```python
P_ROW_SWAP = np.zeros((6, 6))
P_ROW_SWAP[:3, 3:] = np.eye(3)   # send (3..5) → (0..2)
P_ROW_SWAP[3:, :3] = np.eye(3)   # send (0..2) → (3..5)
```

For Jacobians, since the *rows* index the output twist's coordinates:

```python
j_pin = P_ROW_SWAP @ pin.getJointJacobian(pin_model, pin_data, k, pin.LOCAL)
j_ts  = ts.compute_jacobian(ts_model, q, k - 1, ts.LOCAL)
assert np.allclose(j_pin, j_ts, atol=1e-10)
```

For wrenches (which are also 6-vectors but dual to twists), it's the
same permutation applied to the rows.

Two things to note:

- We don't change `tinyspatial`'s convention. The fix is one matrix
  multiply at the test boundary. If we changed the convention, every
  spatial-algebra header would invert and we'd lose Featherstone's
  textbook for our reader.
- Both `0` and `1` go through the same matrix. There's no per-call
  branching; the permutation is a constant.

CLAUDE.md §15 calls this out as the most common cause of Pinocchio
parity bugs. Now you know why.

### 2. Joint indexing: the "universe joint" offset

Pinocchio includes a virtual joint at index 0 called the **universe
joint** — the parent of all real joints. So a 7-DoF Franka has:

| Index | Pinocchio                 | tinyspatial            |
| ----- | ------------------------- | ---------------------- |
| 0     | universe (virtual root)   | (not represented)      |
| 1     | first real joint          | first real joint (`0`) |
| 2     | second                    | second (`1`)           |
| …     | …                         | …                      |
| 7     | last                      | last (`6`)             |

So `pin_model.njoints == 8` and `ts_model.njoints == 7`.

When you iterate to compare poses:

```python
for k in range(1, pin_model.njoints):       # 1..7 in Pinocchio
    pin_pose = pin_data.oMi[k].homogeneous
    ts_pose  = ts_poses[k - 1]              # 0..6 in tinyspatial
    assert np.allclose(pin_pose, ts_pose, atol=1e-10)
```

This *also* affects how you ask for the last joint:

```python
last_pin = pin_model.njoints - 1   # 7 = the wrist
last_ts  = last_pin - 1            # 6
```

The convention is reasonable on both sides: Pinocchio's universe
joint is a natural place to anchor a floating base; we don't have
floating bases as a first-class concept yet. The fix at the boundary
is just to subtract one.

### 3. Quaternion sign

A unit quaternion `q` and its negation `-q` represent the *same
rotation*. So you have a choice: store the canonical representative
where `w ≥ 0`, or store whatever you computed last (which could be
either sign).

| Library      | Convention                     |
| ------------ | ------------------------------ |
| `tinyspatial` | Normalise to `w ≥ 0` (CLAUDE.md §15) |
| Pinocchio    | Normalise to `w ≥ 0`           |

For once, the two libraries agree! Both libraries pick the canonical
representative. So this mismatch isn't actually a problem between
`tinyspatial` and Pinocchio — but it *will* bite you if you ever
compare against MATLAB's Robotics Toolbox, ROS 1's `tf`, or any
library that doesn't normalise.

Mentioned here because:

1. Your test inputs (`pin.randomConfiguration(model)`) can return
   either sign on free joints. If your code compares quaternions
   component-wise, half your random tests will fail.
2. The right way to compare two rotations is via `(R₁ · R₂⁻¹).log()`
   norm, not component-by-component. The log map yields a unique
   rotation vector in `[−π, π]`.

### 4. Jacobian frame: `LOCAL`, `WORLD`, and `LOCAL_WORLD_ALIGNED`

Pinocchio's `getJointJacobian` takes a frame argument:

- `pin.LOCAL` — twist expressed in the body's own frame.
- `pin.WORLD` — twist expressed in the world frame.
- `pin.LOCAL_WORLD_ALIGNED` — origin at the body, axes aligned with
  world. (Useful for IK: the linear part is "world delta x at the
  body's location.")

`tinyspatial` mirrors this exactly with `ts.JacobianFrame.{LOCAL,
WORLD, LOCAL_WORLD_ALIGNED}`. **Same names, same definitions.**

There *is* a subtle gotcha though: the frame argument changes the
*output* coordinates, not the input. So a Jacobian's rows correspond
to the chosen frame; its columns always correspond to the joint
velocities `q̇`. When you apply the row-swap permutation (mismatch #1
above), you do it on the rows of the Jacobian *after* the frame is
chosen.

If you mix this up — apply the permutation before the frame conversion,
say — you get a Jacobian that's wrong in 36 of its 42 entries. Don't
ask me how I know.

### 5. Gravity vector convention

Pinocchio stores gravity on the `Model` itself, as a `Motion`:

```python
pin_model.gravity.linear  = np.array([0, 0, -9.81])
pin_model.gravity.angular = np.zeros(3)
```

If you don't set it, it has a default — usually `[0, 0, -9.81]`, but
it can be mutated by other code. The validation script sets it
explicitly to be safe:

```python
pin_model.gravity.linear = np.array([0, 0, -9.81])
pin_model.gravity.angular = np.zeros(3)
```

`tinyspatial`'s RNEA / ABA take gravity as a function argument:

```cpp
rnea(model, data, q, v, a, tau, /*gravity=*/Vector3(0, 0, -9.81))
```

This is by design — passing gravity per-call lets you do "RNEA with
gravity off" without mutating the model. But when comparing to Pinocchio,
you have to remember to *set* Pinocchio's gravity to match yours.

If you forget, you get a constant per-joint bias on the order of
`m·g·d` (mass times gravity times moment arm) — looks like a
systematic error in the algorithm, but it's just one missing line in
the test setup.

## The pattern

Notice the structure: each "bug" is a known, documentable, *systematic*
difference between two conventions. None of them are random noise. None
of them are mathematical errors.

The recipe for diagnosing one:

1. Run the comparison; observe the disagreement.
2. Compute the residual: `pin_out - ts_out`. Look at its *structure*.
3. If the residual looks like a permutation of the right answer →
   convention mismatch.
4. If the residual is constant across q (not a function of input) →
   probably a wrong reference frame.
5. If the residual is huge (10^0 or more), check the units (radians vs
   degrees, m vs mm).
6. If the residual scales with `q`, then *and only then* suspect a
   real algorithm bug.

In `tinyspatial`'s development, most "parity failures" so far have been
mismatches 1, 2, or 4. Only one was a real algorithm bug, and it was
discovered after a code change broke a previously-passing test —
exactly what validation is for.

## How to avoid creating new mismatches

When you add a new algorithm, ask up front:

- What's my input convention?
- What's Pinocchio's input convention for the same algorithm?
- If they differ, where does the conversion live?

The answer is *always* "in the test." Never in the library.

Documenting it explicitly in `docs/PINOCCHIO_PARITY.md` (with a sentence
like "Pinocchio Jacobians are permuted to angular-first row order
before comparison") is the difference between a passing test and a
trustworthy one. The next maintainer needs to know whether a 0.7×
disagreement is a bug or a permutation.

> ## Where this lives in the library
>
> | Concept                            | File path                                |
> | ---------------------------------- | ---------------------------------------- |
> | Permutation matrix `P_ROW_SWAP`    | [`tests/validation/test_kinematics.py:44-46`](../../tests/validation/test_kinematics.py) |
> | Joint-index offset comment         | [`tests/validation/test_kinematics.py:9-12`](../../tests/validation/test_kinematics.py) |
> | Angular-first convention rationale | [`CLAUDE.md` §5, §15](../../CLAUDE.md)   |
