# Pseudoinverse and damping

The Newton step in IK is $\delta q = J^+ e$ where $J^+$ is some flavour
of pseudoinverse. This sub-chapter is about which flavour to use, and
when.

## The Moore-Penrose pseudoinverse

For any matrix $J \in \mathbb{R}^{m \times n}$, there's a unique matrix
$J^+ \in \mathbb{R}^{n \times m}$ satisfying:

1. $J J^+ J = J$
2. $J^+ J J^+ = J^+$
3. $(J J^+)^\top = J J^+$
4. $(J^+ J)^\top = J^+ J$

This is the **Moore-Penrose pseudoinverse**. For our case ($J$ is
6 × $n_v$):

- If $n_v \geq 6$ and $J$ has full row rank, $J^+ = J^\top (J J^\top)^{-1}$
  (the *right* pseudoinverse). It satisfies $J J^+ = I_6$, so the step
  $\delta q = J^+ e$ exactly produces $J \delta q = e$. Among all
  such $\delta q$, it gives the minimum-norm one.
- If $n_v < 6$ or $J$ has reduced row rank, the right-inverse formula
  fails (the matrix being inverted is singular). The general SVD-based
  formula still works, but you need a separate code path.

In practice, the moment we account for *singularities* — places where
the smallest singular value of $J$ approaches zero — even the
"full-rank" case needs care, because $(J J^\top)^{-1}$ has huge
condition number.

## What goes wrong near singularities

A singular configuration is one where $J$ loses rank. The classic example
on a 6-DoF arm: stretching the elbow out straight makes the wrist twist
about a single axis no matter what the proximal joints do — the rank
drops from 6 to 5.

Near such a configuration:

- $J J^\top$ has a near-zero eigenvalue.
- $(J J^\top)^{-1}$ has a *huge* eigenvalue.
- $\delta q = J^\top (J J^\top)^{-1} e$ has correspondingly huge magnitude
  in the direction of the near-singular mode.

The resulting step is wildly larger than the local linearisation
warrants. The next iteration's $J$ at the new $q$ is *totally different*
from the old one. The robot's hand jumps somewhere unrelated. The
iteration diverges.

This is why pure Newton-with-Moore-Penrose isn't what production IK uses.

## Damped least-squares (DLS)

Replace $J J^\top$ with $J J^\top + \lambda^2 I$:

$$
J^+_\lambda \;:=\; J^\top \bigl(J J^\top + \lambda^2 I_6\bigr)^{-1}.
$$

This is the **damped pseudoinverse**, the Nakamura-Hanafusa /
Wampler-1986 / Levenberg-Marquardt formulation depending on whose
notation you read.

Why it works:

- Far from singularities, $\lambda^2$ is much smaller than the smallest
  eigenvalue of $J J^\top$, so $J^+_\lambda \approx J^+$ — the normal
  Newton step.
- Near singularities, $\lambda^2$ dominates the smallest direction.
  $J^+_\lambda$ in that direction is now $\sim \lambda^{-2} J^\top$ — a
  bounded, well-conditioned step that approaches the Jacobian-transpose
  step.
- The transition is smooth.

You can derive DLS as the solution to a *regularised* least-squares
problem:

$$
\delta q \;=\; \arg\min_{\delta q} \; \|J \delta q - e\|^2 + \lambda^2 \|\delta q\|^2.
$$

The first term is "match the desired pose change"; the second term is
"don't take huge steps." $\lambda$ trades off between them.

## Choosing $\lambda$

A constant $\lambda$ (the library's default) is the simplest. Typical
values are $10^{-2}$ to $10^{-3}$ for arms working in metres and radians;
the units of $\lambda$ are "radians per metre" essentially, so they
should match the scale of the workspace.

Smarter strategies:

- **Wampler's adaptive damping.** $\lambda^2$ scales with the
  pose-error magnitude: $\lambda^2 = \lambda_0^2 + k \|e\|^2$. Near
  convergence (small $\|e\|$), damping shrinks for precise convergence;
  far from target (large $\|e\|$), damping grows for stability.
- **Sugihara's selectively damped least-squares (SDLS).** Damps each
  singular value independently based on its proximity to zero. More
  surgical, but requires an SVD per iteration.

The library uses constant damping for simplicity. If you find a
particular robot needs more sophistication, Wampler is the easy
upgrade.

## Implementation: how to compute the damped step without forming $J^+$

You almost never want to *form* $J^+_\lambda$ explicitly — it's a
dense $n_v \times 6$ matrix. Instead, solve the 6 × 6 linear system:

```cpp
const Matrix6 jjt_damped = j * j.transpose() + lambda_sq_i6;
const Vector6 alpha = jjt_damped.ldlt().solve(error);  // (JJ^T + λ²I) α = e
const VectorX dq = j.transpose() * alpha;              // δq = J^T α
```

Two products, one Cholesky-style solve. Total cost $O(n_v)$ for the
products plus $O(1)$ for the 6 × 6 solve. Per iteration cost is
dominated by FK and Jacobian, not by this step.

`Eigen::LDLT` is well-suited because $J J^\top + \lambda^2 I$ is
symmetric positive definite by construction.

## The Wampler intuition

The damped least-squares step can be read in two equivalent ways:

1. **As a regularised Newton step.** Trust-region style: don't move
   further than the linearisation justifies.
2. **As a soft interpolation between Newton and Jacobian-transpose.**
   For $\lambda \to 0$, DLS becomes Newton; for $\lambda \to \infty$,
   it becomes (a scaled version of) gradient descent on the squared
   error.

The second reading is the one I keep coming back to. DLS gives you
*Newton-like* convergence when things are well-conditioned, and falls
back to a *guaranteed-descent* step when they aren't. That's exactly
the kind of robustness you want from a default IK solver.

> ## Where this lives in the library
>
> | Concept | File / line |
> | ------- | ----------- |
> | DLS computation | [`dls.hpp:83-88`](../../include/tinyspatial/ik/dls.hpp#L83-L88) |
> | `damping` parameter | [`dls.hpp:42-45`](../../include/tinyspatial/ik/dls.hpp#L42-L45) |
