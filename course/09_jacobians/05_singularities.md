# Singularities

A 6-DOF arm has 6 columns in its Jacobian. At most configurations, those 6
columns are linearly independent, and the end-effector can move in any
direction through some combination of joint motions. At *some* configurations,
the columns become linearly dependent — the Jacobian drops rank — and there
is a direction in which the end-effector *cannot* move at any joint
speed. Those configurations are **singularities**.

## What "drops rank" looks like

For a Jacobian $J \in \mathbb{R}^{6 \times 6}$, the rank is "how many
independent directions of motion can be produced." Generic configurations
have rank 6 — motion in any direction is possible. Near a singularity, the determinant of
$J$ heads to zero, and one or more singular values shrinks toward zero —
visually, $J$ is approaching a 5-dimensional "slab" that can't fill all of
$\mathbb{R}^6$.

The most useful diagnostic is the smallest singular value:
$\sigma_{\min}(J)$. Far from any singularity it's a healthy number (e.g.
~0.1 for an arm at "normal" reach); near a singularity it drops smoothly
toward zero.

## Three flavours of singularity

A 6-DOF arm's singularities fall into three named families:

1. **Workspace boundary**: the arm is fully extended (elbow straight). One
   joint's contribution is now collinear with another's. This is the
   geometric "fully stretched" singularity. The fix: avoid it by
   pulling the arm in.
2. **Wrist singularity**: in a spherical wrist (UR5e, FR3), the *axes* of
   two wrist joints become collinear. Famous example on UR robots: the
   first wrist axis and the third wrist axis align when the second is at 0°
   or π. The arm can spin around the alignment axis many ways but can't
   take certain end-effector poses without a discontinuous joint motion.
3. **Algorithmic / internal**: an internal alignment of links not at the
   boundary. Less common but they exist (e.g. some elbow-up vs elbow-down
   transitions).

A 7-DOF arm like the Franka FR3 has *families* of singularities — they
form curves in the 7-D configuration space, and escape is usually possible via
the redundancy direction. Part of the appeal of having that extra DOF.

## Why "damped least squares" is the fix

Using $J^{-1}$ naively for IK makes that inverse blow up near a singularity
(small singular value → huge inverse). The classic remedy is **damped
least squares**:

$$
\dot q \;=\; J^\top (J J^\top + \lambda^2 I)^{-1}\,v_{\text{target}}.
$$

For non-zero $\lambda$, the inversion stays bounded even when $J$ is
singular: in the singular directions, $\dot q$ shrinks to zero instead of
exploding. The IK chapter (12) builds this in detail.

## What this means in practice

Two takeaways:

- Small singular values in the Jacobian are not cause for alarm — they are
  geometric facts of the arm's structure, not implementation bugs.
- Algorithms that touch $J^{-1}$ must use damping or a pseudo-inverse with
  truncation, *not* the bare inverse.

## A quick numerical probe

Adapted from `test_jacobian.cpp`:

```cpp
Matrix6X j(6, m.nv());
compute_jacobian(m, d, link_id, j, JacobianFrame::kLocalWorldAligned);
Eigen::JacobiSVD<Matrix6X> svd(j);
std::cout << "singular values: " << svd.singularValues().transpose() << "\n";
```

Sweeping one joint angle and tracking $\sigma_{\min}$ reveals dips in
predictable places — the wrist or workspace-boundary singularity
making itself visible.

## Where this lives in the library

This chapter is about *intuition*; the actual damped-least-squares IK is in
chapter 12 (`ik/dls.hpp`). The key point is what the Jacobian
signals when it nearly fails.

| Concept | File |
| ------- | ---- |
| Jacobian function (still the source) | [`jacobian.hpp`](../../include/tinyspatial/algo/jacobian.hpp) |
| Future damped least squares | `include/tinyspatial/ik/dls.hpp` (Phase 7) |

Next: the [exercises](exercises.md), then [chapter 10 — RNEA](../10_dynamics_RNEA/README.md).
