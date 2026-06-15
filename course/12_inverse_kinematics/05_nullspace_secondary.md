# Null-space and secondary tasks

A 7-DoF arm has *more* joints than the 6 dimensions of a rigid-body
pose. The hand can be held in place while the elbow still moves. The
extra freedom is the **null-space of the Jacobian** — motions that
don't move the end-effector.

That extra freedom is a feature. We can use it to satisfy a *secondary
objective* without disturbing the primary one.

## The null-space, geometrically

For a Jacobian $J \in \mathbb{R}^{6 \times n_v}$ at some configuration,

$$
\mathrm{null}(J) \;=\; \{\delta q \in \mathbb{R}^{n_v} \mid J \delta q = 0\}.
$$

It's a $(n_v - r)$-dimensional subspace, where $r = \mathrm{rank}(J)$.
For a non-singular 7-DoF arm, that's a 1-D subspace — one direction in
joint space that doesn't move the hand. For a 6-DoF arm at full rank,
$\mathrm{null}(J) = \{0\}$ — no freedom.

The projector onto the null-space is

$$
N = I - J^+ J,
$$

where $J^+$ is the Moore-Penrose pseudoinverse. For any vector $u$,
$N u$ is the part of $u$ in $\mathrm{null}(J)$.

## Task priority

Pick a primary task — usually end-effector pose — and a secondary task
that we'd like to satisfy *if possible*. The classical formulation
(Siciliano 1990):

$$
\delta q \;=\; J^+ \,e_{\text{primary}} \;+\; \alpha \, N \, g_{\text{secondary}}.
$$

Where:

- The first term solves the primary task (sub-chapter 02).
- $g_{\text{secondary}}$ is the *gradient* of the secondary cost.
- $N$ projects it into $\mathrm{null}(J)$, so adding it doesn't disturb
  the primary.

The order matters: primary first, secondary in the leftover space.

## A practical secondary: posture attraction

The simplest useful secondary task is "stay near a comfortable
configuration." Pick a desired posture $q_{\mathrm{rest}}$ (often
$\mathbf{0}$ or a known-good "home" config). The secondary cost is

$$
\Phi_2(q) \;=\; \tfrac{1}{2} \|q - q_{\mathrm{rest}}\|^2,
$$

and its negative gradient is just $q_{\mathrm{rest}} - q$. The
nullspace-projected step:

$$
\delta q_{\text{secondary}} \;=\; N \cdot (q_{\mathrm{rest}} - q).
$$

In practice this yields "the elbow-up branch" or "the joint-limits-
respecting branch" of IK without writing a smarter solver.

The library implements exactly this in `nullspace.hpp`.

## The two-tier damping trick

There's a subtle issue: if we use the *damped* pseudoinverse for both
the primary step and the projector $N$, the projector leaks. Specifically,

$$
N_\lambda \;:=\; I - J^\top (J J^\top + \lambda^2 I)^{-1} J
$$

is *not* an exact projector onto $\mathrm{null}(J)$. The leakage has
magnitude $O(\lambda^2 / \sigma_{\min}^2)$ where $\sigma_{\min}$ is the
smallest singular value of $J$. With $\lambda = 10^{-2}$, that's
$\sim 10^{-4}$ — small, but it blocks primary convergence below that
level.

The fix is to use *two different* damping coefficients:

- $\lambda = 10^{-2}$ for the **primary step** (singularity-robust).
- $\lambda_{\text{proj}} = 10^{-10}$ for the **projector** (near-exact).

This is what the library does:

```cpp
// Primary step: damped to handle singularities.
const Matrix6 jjt_damped = j * j.transpose() + lambda_sq_i6;
const Vector6 alpha = jjt_damped.ldlt().solve(result.error);
const VectorX dq_primary = j.transpose() * alpha;

// Projector: much smaller regularizer to keep near(null J) near-null.
constexpr Scalar kProjectorReg = 1e-10;
const Matrix6 jjt_projector = j * j.transpose() + kProjectorReg * Matrix6::Identity();
const auto jjt_proj_ldlt = jjt_projector.ldlt();
```

Without this two-tier damping, the test
[`NullspaceTracksPrimary`](../../tests/unit/ik/test_nullspace.cpp) fails:
the primary error gets to $10^{-4}$ and stops, because the secondary
gradient keeps re-injecting that much error per step. With two-tier
damping, it converges below $10^{-6}$ as expected.

## The fundamental limitation

The two-tier trick works because away from singularities, the projector
*is* near-exact. *At* a singularity, even $\lambda_{\text{proj}} = 0$
gives a degenerate projector — there's no "true" null-space direction
to project onto. The right thing in that case is to either:

1. Detect the singularity (smallest singular value of $J$ very small)
   and disable the secondary, or
2. Use a singularity-robust SVD-based projector instead of the
   $(J J^\top)^{-1}$ formulation.

For our fixture robots — which don't have closed-form singularities
in random configurations — the simple two-tier approach is good enough.
A future PR could add SVD-based projection for robustness.

## A worked call

```cpp
#include "tinyspatial/ik/nullspace.hpp"
// ... model setup ...

// Posture target: "natural" elbow-up configuration.
tinyspatial::VectorX q_rest = tinyspatial::VectorX::Zero(m.nq());
q_rest(3) = -0.5;  // example: bias one elbow joint toward "up"

tinyspatial::NullspaceOptions opts;
opts.secondary_gain = 0.5;

const auto result = tinyspatial::solve_ik_nullspace(
    m, d, end_effector, target, q_init, q_rest, opts);
```

The primary task (hand at `target`) is solved exactly; the resulting
configuration is the one closest to `q_rest` *along the null-space*.

## Going further: multi-priority

Real robotics applications often want more than two priority levels —
*e.g.* primary EE pose, secondary obstacle avoidance, tertiary posture.
The classical Siciliano formulation generalises:

$$
\delta q_k \;=\; J_k^+ \,e_k + N_k \, \delta q_{k+1}, \quad N_k = N_{k-1} (I - J_k^+ J_k),
$$

projecting each level into the null-space of all higher-priority
levels. This requires keeping the augmented projector $N_k$ around while
descending the priority list.

The library currently supports only two levels (primary + posture
secondary). Adding multi-priority is in scope for a future phase if a
concrete use case appears.

> ## Where this lives in the library
>
> | Concept | File · symbol |
> | ------- | ------------- |
> | `NullspaceOptions` | [`nullspace.hpp`](../../include/tinyspatial/ik/nullspace.hpp) · `NullspaceOptions` |
> | Two-tier damping, projector step | `nullspace.hpp` · `solve_ik_nullspace` |
