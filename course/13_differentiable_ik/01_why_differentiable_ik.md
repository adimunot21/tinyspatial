# Why differentiable IK matters

Three concrete situations where treating IK as a *differentiable
function* — not just a callable solver — earns its keep.

## 1. Optimising a target pose under a downstream constraint

You have a task that involves a robot reaching for some object. The
target pose isn't fixed — there's a region of acceptable grasp poses,
and you want to pick the one that lands in the most *comfortable*
configuration (lowest torque, furthest from joint limits, etc.).

The optimisation looks like:

$$
T^*_{\mathrm{best}} \;=\; \arg\min_{T^* \in \mathcal{R}} \; \mathcal{L}(q^*(T^*)),
$$

where $\mathcal{R}$ is the reachable set of grasp poses,
$q^*(T^*) = \mathrm{IK}(T^*)$, and $\mathcal{L}$ is your task loss.

Standard gradient descent on $T^*$ needs $\nabla_{T^*} \mathcal{L}$.
By the chain rule:

$$
\nabla_{T^*} \mathcal{L} \;=\; \left(\frac{\partial q^*}{\partial T^*}\right)^{\!\top} \nabla_{q^*} \mathcal{L}.
$$

The first factor is exactly what `ik_implicit_derivative` returns. Without
it, you'd have to finite-difference: 6 extra IK calls per gradient
evaluation. For a 7-DoF arm at ~5 ms per IK call, that's 30 ms of
wasted compute per gradient step. With the implicit derivative, you
pay for *one* extra Jacobian and a 6 × 6 solve.

## 2. End-to-end learning through IK

You're training a policy or planner that outputs target poses to a
robot. The training signal comes from rolling out the trajectory in
simulation and measuring some reward. The trajectory uses IK to convert
the planner's output into joint commands.

Backprop has to flow through IK. PyTorch and JAX both support "custom"
backward passes (`torch.autograd.Function`, `jax.custom_vjp`). Inside,
you put exactly the analytical derivative this chapter computes.

You *could* run IK with autodiff turned on inside the solver itself
(unrolling 200 iterations of damped Newton), but that's:

- **Slow** — 200× more compute than the implicit method.
- **Memory-hungry** — you store every iteration's intermediate
  tensors.
- **Numerically unstable** — the gradient through a long iteration
  often blows up near singularities.

The implicit method sidesteps all three. Just one Jacobian call at the
final $q^*$, one 6 × 6 solve.

## 3. Sensitivity analysis and trust regions

Production robot code often needs to know *how much* a small target
mis-calibration would shift the joint angles. For instance:

- "The vision system reports object position with $\sigma = 5$ mm.
  What's the resulting standard deviation of the joint angles?"
- "I want to plan a smooth trajectory. As I move the target through
  space, how much does $q^*$ change per metre of target motion?"

Both are first-order sensitivity questions. The answer is
$\partial q^* / \partial T^* \cdot \sigma_{T^*}$, possibly propagated
forward through the joint-space covariance.

Without the analytical derivative, sensitivity analysis means a lot of
IK calls. With it, it's matrix multiplication.

## What this chapter does *not* unlock

Some things you might think differentiable IK would help with, but
doesn't:

- **It doesn't make IK solvable in closed form.** You still need to
  call the iterative solver to *find* $q^*$. Differentiability is
  about the derivative *at* a known $q^*$.
- **It doesn't help if the IK doesn't converge.** Garbage in, garbage
  derivative out.
- **It assumes the fixed point is locally isolated.** For redundant
  arms at the boundary of the workspace, $q^*$ might be a manifold of
  solutions and the implicit theorem needs the damped pseudoinverse
  instead of the exact one — which is what the library uses, so this
  is handled, but the derivative is *algorithm-specific* in that
  regime, not problem-specific.

## What about backprop through the full iteration?

If you really need the gradient through the *iteration trajectory*
(not just the fixed point), there are deep-learning techniques —
"deep equilibrium" / "implicit deep learning" frameworks. The
implicit function theorem is the foundation; those frameworks build
on it.

For our use case (robotics, fixed-point IK), the implicit method is
all you need.

> ## Where this lives in the library
>
> | Concept | File · symbol |
> | ------- | ------------- |
> | The function | [`differentiable.hpp`](../../include/tinyspatial/ik/differentiable.hpp) · `ik_implicit_derivative` |
