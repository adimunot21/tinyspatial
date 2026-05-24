# Validating against Pinocchio

The same harness that cross-checks RNEA (chapter 10, section 05) is
extended to CRBA and ABA. The structure is identical:

1. Load the URDF in both libraries.
2. Sample 1000 random configurations + random velocities + random
   "input" (acceleration for RNEA, torques for ABA).
3. Compare the outputs in $\infty$-norm.
4. Aggregate the max difference across samples.
5. Fail CI if any robot exceeds the `1e-10` tolerance.

## CRBA: the symmetrise-then-compare step

Pinocchio's `pin.crba` fills only the upper triangle of $M$. We
fill both triangles in tinyspatial (by mirroring at the end). Before
comparing, the harness symmetrises Pinocchio's output:

```python
m_pin = pin.crba(pin_model, pin_data, q)
m_pin = np.triu(m_pin) + np.triu(m_pin, 1).T   # symmetrise (Pinocchio convention)
m_ts = ts.crba(ts_model, q)
diff = float(np.max(np.abs(m_pin - m_ts)))
```

Observed agreement (current run, from
[`docs/PINOCCHIO_PARITY.md`](../../docs/PINOCCHIO_PARITY.md)):

- `simple_arm`: `1.8e-15`
- `franka_fr3`: `5.3e-15`
- `ur5e`: `3.1e-15`
- `so_arm101`: `2.4e-17`

All ~ machine precision — five orders of magnitude inside the
`1e-10` spec. Symmetry of the CRBA output is *structural* (we write the
mirror image, not compute it independently), so the symmetry of
$M_{\text{pin}}$ vs $M_{\text{ts}}$ shouldn't differ.

## ABA: the freshly-sampled torque

Note that the ABA comparison uses a *separate* random torque vector
($\tau$), not the $\tau$ that came out of RNEA in the previous step. If
we had reused $\mathrm{RNEA}(q, v, a)$ as the ABA input, the cross-check
would devolve into "$\mathrm{ABA}(\mathrm{RNEA}^{-1}(a)) = a$," which
only checks the internal consistency of the *pair* — it would still
catch a sign flip, but not a coordinated bug in both algorithms. Drawing
a fresh $\tau$ makes the test independent:

```python
tau_in = rng.standard_normal(pin_model.nv)
qdd_pin = pin.aba(pin_model, pin_data, q, v, tau_in)
qdd_ts = ts.aba(ts_model, q, v, tau_in, gravity)
diff = float(np.max(np.abs(qdd_pin - qdd_ts)))
```

Observed agreement (current run):

- `simple_arm`: `7.1e-15`
- `franka_fr3`: `9.4e-13`
- `ur5e`: `4.5e-13`
- `so_arm101`: `5.5e-12`

Two orders of magnitude looser than RNEA / CRBA, but still 2–3 orders
inside the `1e-10` tolerance. The looseness is expected: ABA does an
effective matrix inversion (the $D_i^{-1}$ steps), and so the conditioning
of the small fixed-mass tip links matters. `so_arm101` has a fixed
gripper attached at the end, so its outermost inertia chain is the
"thinnest" of the four — and it's the loosest agreement, exactly as
predicted.

## What this combined harness catches

- **CRBA off-diagonal indexing.** Easy bug: writing $M[i, j]$ when you
  meant $M[j, i]$. Random configs catch this immediately because
  $M$ is symmetric but the bug breaks symmetry.
- **ABA's $D_i^{-1}$ inversion.** If for a multi-DOF joint you transpose
  $S_i$ wrong, $D_i$ becomes non-positive-definite or wrong-sized; the
  inverse blows up.
- **Inertia transport sign / convention.** ABA's congruence transport
  ($X^* I X^{*\top}$) is the easiest place to write
  `motion_plucker` where you mean `force_plucker`. The cross-check
  on a 7-DOF arm with non-trivial geometry catches it loud.

## What it doesn't catch

Same caveats as RNEA's harness (chapter 10, section 05):

- Both libraries implementing the same textbook bug (improbable).
- Performance regressions (chapter 16's job).
- Edge cases not present in the fixture URDFs — *e.g.* loops, articulated
  closed chains, soft constraints. None of those exist in our fixtures
  yet.

## The Pinocchio parity table

[`docs/PINOCCHIO_PARITY.md`](../../docs/PINOCCHIO_PARITY.md) is
auto-generated. Every row is a tinyspatial algorithm × Pinocchio
algorithm × fixture URDF, with the max-abs-diff across 1000 random
samples. If you make changes to the dynamics, regenerate it, and the
table goes red, *don't* relax the tolerance — find the bug.

> ## Where this lives in the library
>
> | Concept | File / line |
> | ------- | ----------- |
> | The harness | [`tests/validation/test_kinematics.py`](../../tests/validation/test_kinematics.py) |
> | CRBA comparison | [`test_kinematics.py:126-134`](../../tests/validation/test_kinematics.py#L126-L134) |
> | ABA comparison | [`test_kinematics.py:136-143`](../../tests/validation/test_kinematics.py#L136-L143) |
> | Parity table | [`docs/PINOCCHIO_PARITY.md`](../../docs/PINOCCHIO_PARITY.md) |
