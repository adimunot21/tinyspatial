# Singularities

A 6-DOF arm has 6 columns in its Jacobian. At most configurations, those 6
columns are linearly independent, and you can move the end-effector in any
direction by some combination of joint motions. At *some* configurations,
the columns become linearly dependent — the Jacobian drops rank — and there
is a direction in which you *cannot* move the end-effector at any joint
speed. Those configurations are **singularities**.

## What "drops rank" looks like

For a Jacobian $J \in \mathbb{R}^{6 \times 6}$, the rank is "how many
independent directions of motion can be produced." Generic configurations
have rank 6; you can move anywhere. Near a singularity, the determinant of
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
   geometric "fully stretched" singularity. The fix: don't go there;
   pull the arm in.
2. **Wrist singularity**: in a spherical wrist (UR5e, FR3), the *axes* of
   two wrist joints become collinear. Famous example on UR robots: the
   first wrist axis and the third wrist axis align when the second is at 0°
   or π. The arm can spin around the alignment axis many ways but can't
   take certain end-effector poses without a discontinuous joint motion.
3. **Algorithmic / internal**: an internal alignment of links not at the
   boundary. Less common but they exist (e.g. some elbow-up vs elbow-down
   transitions).

A 7-DOF arm like the Franka FR3 has *families* of singularities — they
form curves in the 7-D configuration space, and you can usually escape via
the redundancy direction. Part of the appeal of having that extra DOF.

## Why "damped least squares" is the fix

If you naively use $J^{-1}$ for IK, near a singularity that inverse blows
up (small singular value → huge inverse). The classic remedy is **damped
least squares**:

$$
\dot q \;=\; J^\top (J J^\top + \lambda^2 I)^{-1}\,v_{\text{target}}.
$$

For non-zero $\lambda$, the inversion stays bounded even when $J$ is
singular: in the singular directions, $\dot q$ shrinks to zero instead of
exploding. The IK chapter (12) builds this in detail.

## What this means in practice

Two takeaways:

- Don't be alarmed when the Jacobian has small singular values — they're
  geometric facts of the arm's structure, not bugs in your code.
- Algorithms that touch $J^{-1}$ must use damping or a pseudo-inverse with
  truncation, *not* the bare inverse.

## A quick numerical probe

Adapt this from `test_jacobian.cpp`:

```cpp
Matrix6X j(6, m.nv());
compute_jacobian(m, d, link_id, j, JacobianFrame::kLocalWorldAligned);
Eigen::JacobiSVD<Matrix6X> svd(j);
std::cout << "singular values: " << svd.singularValues().transpose() << "\n";
```

Sweep one joint angle and watch $\sigma_{\min}$. You'll see it dip in
predictable places — that's the wrist or workspace-boundary singularity
making itself visible.

## Where this lives in the library

This chapter is about *intuition*; the actual damped-least-squares IK is in
chapter 12 (`ik/dls.hpp`). For now, just have a feel for what the Jacobian
is telling you when it nearly fails.

| Concept | File |
| ------- | ---- |
| Jacobian function (still the source) | [`jacobian.hpp`](../../include/tinyspatial/algo/jacobian.hpp) |
| Future damped least squares | `include/tinyspatial/ik/dls.hpp` (Phase 7) |

Next: the [exercises](exercises.md), then [chapter 10 — RNEA](../10_dynamics_RNEA/README.md).
