# The IK case

Apply the implicit function theorem to the IK fixed-point condition and
read off the answer.

## The residual at the fixed point

At a converged IK solution $q^*$:

$$
r(q^*, T^*) \;=\; \log_{\mathrm{SE}(3)}\!\bigl(T(q^*)^{-1} T^*\bigr) \;=\; 0 \;\in\; \mathbb{R}^6.
$$

This is the body-frame Lie-tangent error we met in chapter 12. The
solution $q^*$ depends on $T^*$; we want $\partial q^* / \partial T^*$.

The implicit theorem says:

$$
\frac{\partial q^*}{\partial T^*} \;=\; -\!\left(\frac{\partial r}{\partial q}\right)^{\!+} \frac{\partial r}{\partial T^*}.
$$

(I'm writing pseudoinverse $({}^+)$ because for redundant arms $\partial r /
\partial q$ is rectangular; we'll come back to which pseudoinverse to
use.)

So we need the two partials.

## ∂r/∂q

Perturb $q$ by $\delta q$:

$$
T(q + \delta q) \;\approx\; T(q) \cdot \exp\!\bigl([J_L(q) \delta q]^\wedge\bigr),
$$

where $J_L$ is the body-frame Jacobian (chapter 09). Then:

$$
T(q + \delta q)^{-1} \;=\; \exp\!\bigl(-[J_L \delta q]^\wedge\bigr) \cdot T(q)^{-1},
$$

and:

$$
T(q + \delta q)^{-1} T^* \;=\; \exp\!\bigl(-[J_L \delta q]^\wedge\bigr) \cdot T(q)^{-1} T^* \;=\; \exp\!\bigl(-[J_L \delta q]^\wedge\bigr) \cdot \exp\!\bigl([r]^\wedge\bigr).
$$

Taking the log and using Baker-Campbell-Hausdorff (which simplifies
since we evaluate at $r = 0$):

$$
\log\bigl(T(q + \delta q)^{-1} T^*\bigr) \;\approx\; -J_L \delta q + r \;=\; -J_L \delta q + 0.
$$

So:

$$
\boxed{\;\frac{\partial r}{\partial q} \;=\; -J_L(q^*).\;}
$$

## ∂r/∂T*

Parametrise the target perturbation as right-multiplication by an
$\mathrm{SE}(3)$ exponential:

$$
T^* \;\to\; T^* \cdot \exp([\delta \xi]^\wedge), \qquad \delta\xi \in \mathbb{R}^6.
$$

This is the **body-frame** parametrisation of target perturbations.
Then:

$$
T(q^*)^{-1} \cdot T^* \cdot \exp([\delta \xi]^\wedge) \;=\; \exp([r]^\wedge) \cdot \exp([\delta \xi]^\wedge),
$$

and at $r = 0$:

$$
\log\bigl(\exp([\delta \xi]^\wedge)\bigr) \;=\; \delta \xi.
$$

So:

$$
\boxed{\;\frac{\partial r}{\partial T^*} \;=\; +I_6.\;}
$$

## Combining

Plug into the implicit theorem:

$$
\frac{\partial q^*}{\partial T^*} \;=\; -(-J_L)^+ \cdot I \;=\; J_L^+.
$$

That's it. The Jacobian of IK is the *pseudoinverse* of the body-frame
spatial Jacobian.

## Which pseudoinverse?

For a 6-DoF arm (non-redundant), $J_L$ is square. When not at a
singularity, $J_L^+ = J_L^{-1}$ — the exact inverse. Easy.

For a 7-DoF arm (redundant), $J_L$ is $6 \times 7$. There are infinitely
many left-inverses; the Moore-Penrose pseudoinverse is

$$
J_L^+ \;=\; J_L^\top (J_L J_L^\top)^{-1}.
$$

This is the one that produces a **minimum-norm** $\delta q$. It's the
"natural" choice geometrically.

But — and this is the subtle bit — when the IK is *implemented with
damping* (DLS, chapter 12), the algorithm's actual fixed-point
manifold and its sensitivity differ from the Moore-Penrose ideal by
$O(\lambda^2)$. The correct derivative *of the DLS algorithm's
output* is the **damped** pseudoinverse:

$$
J_L^+_\lambda \;=\; J_L^\top (J_L J_L^\top + \lambda^2 I)^{-1}.
$$

The library uses this. For tight tolerances and small damping, it
approaches the Moore-Penrose value; near singularities, it's the only
sensible choice.

So the final formula, as implemented:

$$
\boxed{\;\frac{\partial q^*}{\partial T^*} \;=\; J_L^\top (J_L J_L^\top + \lambda^2 I)^{-1}.\;}
$$

## Verifying by finite difference

The test [`DifferentiableIk.FdAgreesOnFranka`](../../tests/unit/ik/test_differentiable.cpp)
does exactly this:

1. Find $q^*$ at some target $T^*$ via DLS (tight tolerance, small damping).
2. Compute the analytical $\partial q^* / \partial T^*$.
3. For each $k = 0, \ldots, 5$, perturb $T^*$ to $T^* \cdot \exp(\pm \epsilon e_k^\wedge)$ and re-run DLS warm-started from $q^*$.
4. Compute the central FD column $\bigl(q^*_+ - q^*_-\bigr) / (2 \epsilon)$.
5. Compare to the analytical column.

Agreement is at the FD floor (~1e-4 with $\epsilon = 10^{-5}$), exactly
as the theory predicts.

A subtle but important detail: the FD test uses **tight damping
(λ=1e-5)** in *both* the solver and the analytical formula. With the
default DLS damping (λ=1e-2), the IK convergence has bias that
shifts the fixed point, and the implicit theorem's prediction differs
from FD by ~0.1% (= λ² / σ_min²) — visible but small. Tight damping
removes the bias; the agreement is then bounded only by FD precision.

## World-frame target perturbations

Sometimes you want target perturbations in **world** coordinates
instead of body-local:

$$
T^* \;\to\; \exp([\eta]^\wedge) \cdot T^*, \qquad \eta \in \mathbb{R}^6.
$$

A world-frame perturbation $\eta$ corresponds to a body-frame
perturbation $\delta \xi = \mathrm{Ad}_{T^{*-1}} \eta$ (the adjoint
transports twists between frames; see chapter 04).

So the world-frame derivative is

$$
\frac{\partial q^*}{\partial T^*}\bigg|_{\mathrm{world}} \;=\; \left(\frac{\partial q^*}{\partial T^*}\bigg|_{\mathrm{body}}\right) \cdot \mathrm{Ad}_{T^{*-1}}.
$$

Multiply the returned $n_v \times 6$ matrix by `T_target.adjoint_inverse()`
from the right. (The library doesn't do this for you because the
right convention depends on what you're trying to do.)

## Cost

Per call:

- One FK pass: $O(N)$.
- One Jacobian call: $O(N)$.
- One $6 \times 6$ Cholesky solve: $O(1)$.
- One $n_v \times 6$ matrix product: $O(N)$.

Total: $O(N)$. Effectively free.

Compare with autodiff through the IK iteration: $O(N \cdot k)$ where
$k$ is the number of solver iterations (typically 10–100). The
implicit method is 10–100× faster.

> ## Where this lives in the library
>
> | Concept | File · symbol |
> | ------- | ------------- |
> | The formula | [`differentiable.hpp`](../../include/tinyspatial/ik/differentiable.hpp) · `ik_implicit_derivative` |
> | The FD test | [`test_differentiable.cpp`](../../tests/unit/ik/test_differentiable.cpp) |
