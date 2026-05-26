# The Jacobian approach

Newton's method on a manifold. Same shape as the high-school version,
but with $\log_{\mathrm{SE}(3)}$ in the place of vector subtraction and
the spatial Jacobian in the place of the derivative.

## The local linearisation

If we're at configuration $q_k$ with pose $T_k = \mathrm{FK}_L(q_k)$ and
target $T^*$, the body-frame error is

$$
e_k = \log_{\mathrm{SE}(3)}\!\bigl(T_k^{-1} T^*\bigr).
$$

For a small configuration update $q_{k+1} = q_k + \delta q$, the
linearisation of FK around $q_k$ in the body-local frame is

$$
\mathrm{FK}_L(q_k + \delta q) \;\approx\; \mathrm{FK}_L(q_k) \cdot \exp\!\bigl(J_L(q_k) \delta q\bigr),
$$

where $J_L$ is the LOCAL-frame Jacobian of link $L$ (chapter 09, the
default frame our `compute_jacobian` returns). The body-frame error
after the step becomes (to first order):

$$
e_{k+1} \;\approx\; e_k - J_L(q_k) \,\delta q.
$$

To drive $e_{k+1}$ to zero in one step, solve

$$
J_L(q_k) \,\delta q \;=\; e_k.
$$

That's the Newton update on $\mathrm{SE}(3)$. Each iteration: one FK,
one Jacobian, one linear solve.

## The shape problem

$J_L$ is $6 \times n_v$. Three cases:

- **$n_v = 6$ (full-rank, non-redundant).** $J_L$ is square. Solve
  $J_L \delta q = e$ exactly — *if* $J_L$ is invertible. At singularities
  it isn't.
- **$n_v > 6$ (redundant).** $J_L$ has more columns than rows. The
  system is *underdetermined*: infinitely many $\delta q$ produce the
  same change in pose. We need to pick one.
- **$n_v < 6$.** Rare in practice (a 5-DoF robot can't reach arbitrary
  6-DoF poses). The system is overdetermined; the best we can do is
  least-squares.

In every case the answer involves a *pseudoinverse*, denoted $J^+$:

$$
\delta q \;=\; J^+ \,e.
$$

What kind of pseudoinverse depends on the shape. Sub-chapter 03 covers
this; for now, the practical answer for $n_v \geq 6$ is the *right*
pseudoinverse:

$$
J^+ \;=\; J^\top (J J^\top)^{-1}.
$$

It produces the *minimum-norm* $\delta q$ that satisfies $J \delta q
= e$. That's a property we'll often want.

## The whole loop

```python
q = q_init
for iter in range(max_iters):
    T_k = FK_L(q)
    e = log(T_k.inverse() * T_target)
    if max(abs(e)) < tol:
        return q  # converged
    J = compute_jacobian(model, q, link_id, LOCAL)
    delta_q = J_pseudoinverse(J) @ e
    q = q + alpha * delta_q
```

Five lines of math. The hard parts are:

- **What if $J$ is singular?** Damping (next sub-chapter).
- **What if we're far from the target?** Step-size and damping
  together (also next sub-chapter).
- **What if we get stuck?** Random restart (sub-chapter 06).

## Why the *local* Jacobian

We use the LOCAL frame for both the error and the Jacobian. That's not
the only choice — you can do the same algorithm with the WORLD frame —
but LOCAL has a clean interpretation:

> *The error $e$ is the body-frame twist that would take body $L$ to
> the target in unit time. The Jacobian $J_L$ maps joint velocities to
> body-frame twists.*

So "solve $J \delta q = e$" reads as: "find the joint velocity that
produces this body-frame twist." A clean physical meaning.

WORLD-frame IK is equivalent and gives the same final $q^*$, just with
different intermediate steps. We use LOCAL because the error definition
is more natural and because all the library's existing Jacobian code
defaults to LOCAL.

## What about the gradient of the squared error?

There's a tempting alternative: define the cost $\Phi(q) = \tfrac{1}{2}
\| e(q) \|^2$ and run gradient descent. The gradient is

$$
\nabla \Phi(q) \;=\; J^\top e
$$

(modulo a Lie-group correction that's small for small errors). So the
"naïve gradient descent" update is $\delta q = -J^\top e$ — no inverse
at all.

This is the **Jacobian transpose method**. It always makes progress
(every step reduces the squared error along the gradient), but it
converges *linearly* and very slowly. Useful as a fallback near
singularities, but you don't want to use it as the only method.

Pinocchio and most other libraries use it as a starting move when the
DLS step would blow up. The library's current DLS implementation doesn't
fall back to $J^\top$ explicitly because the damping does the same job
implicitly: when $\lambda^2$ dominates $J J^\top$, the DLS step
$\delta q = J^\top (J J^\top + \lambda^2 I)^{-1} e$ approaches
$\lambda^{-2} \cdot J^\top e$ — a scaled Jacobian-transpose step.

That's actually a *very* nice property of DLS that we'll come back to in
the next sub-chapter.

> ## Where this lives in the library
>
> | Concept | File / line |
> | ------- | ----------- |
> | The iteration | [`dls.hpp:71-93`](../../include/tinyspatial/ik/dls.hpp#L71-L93) |
> | The Jacobian call (LOCAL frame) | [`dls.hpp:82`](../../include/tinyspatial/ik/dls.hpp#L82) |
