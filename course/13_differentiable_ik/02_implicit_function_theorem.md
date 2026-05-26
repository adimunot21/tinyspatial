# The implicit function theorem

Here's the math toolkit we need. Before applying it to IK, let's see it
on simpler problems where the algebra is transparent.

## The setup

You have an equation defining one variable in terms of another:

$$
F(x, y) \;=\; 0,
$$

where $x \in \mathbb{R}^m$ and $y \in \mathbb{R}^n$. For each $y$,
this equation pins down $x$ (or several $x$'s; we'll talk about
uniqueness in a moment). Let's call the solution $x = g(y)$.

Question: what's $\partial g / \partial y$?

The **implicit function theorem** says: if $\partial F / \partial x$ is
invertible at the current $(x_0, y_0)$, then locally $g$ is well-defined
and differentiable, and

$$
\boxed{\;\frac{\partial g}{\partial y} \;=\; -\left(\frac{\partial F}{\partial x}\right)^{\!-1} \frac{\partial F}{\partial y}.\;}
$$

That's it. No need to solve $F = 0$ for $x$ in closed form.

The derivation is one line: differentiate $F(g(y), y) = 0$ w.r.t. $y$,
apply the chain rule:

$$
\frac{\partial F}{\partial x}\,\frac{\partial g}{\partial y} \;+\; \frac{\partial F}{\partial y} \;=\; 0.
$$

Solve for $\partial g / \partial y$. Done.

## A toy example

Take $F(x, y) = x^2 - y = 0$, so $x = \sqrt y$ (for $x > 0$). The
derivative is $1/(2\sqrt y)$.

Implicit theorem:
$\partial F / \partial x = 2x$, $\partial F / \partial y = -1$.
$\partial g / \partial y = -(2x)^{-1} \cdot (-1) = 1/(2x) = 1/(2\sqrt y)$. ✓

We never had to take the square root in the algebra.

## When does the theorem fail?

Whenever $\partial F / \partial x$ is **not invertible**. Two cases:

1. **The square Jacobian is singular.** Example: at $x = 0$ in
   $F(x, y) = x^2 - y$, $\partial F / \partial x = 0$. The
   solution function $\sqrt y$ has a vertical tangent at $y = 0$;
   $\partial g / \partial y$ blows up. The theorem correctly tells us
   it can't say anything there.

2. **The Jacobian is rectangular.** If $x$ has more dimensions than
   the equations $F$, then $\partial F / \partial x$ can't be
   "inverted" — it's a wide matrix. The fix is to use a
   **pseudoinverse**, but then you've made a choice (Moore-Penrose? a
   regularised one? a different one?). The "derivative" you get
   depends on which pseudoinverse you choose, which in turn depends
   on the *algorithm* you used to pick $x$ in the first place.

This second case is *exactly* the IK case for redundant robots, and
it's where the damping in DLS gets baked into the derivative formula.
We'll see that in sub-chapter 03.

## Another worked example: the equation of a circle

$F(x, y) = x^2 + y^2 - 1 = 0$ (the unit circle).

For $y > 0$, $g(y) = \sqrt{1 - y^2}$.
Direct derivative: $g'(y) = -y / \sqrt{1 - y^2}$.

Implicit:
$\partial F / \partial x = 2x$, $\partial F / \partial y = 2y$.
$g'(y) = -(2x)^{-1} \cdot 2y = -y/x = -y / \sqrt{1 - y^2}$. ✓

Same answer. The implicit method saved us the algebra of expressing
$x$ in terms of $y$.

## Why this is the right tool for IK

IK is exactly the implicit-function-theorem setup. We have a residual

$$
r(q, T^*) \;=\; \log_{\mathrm{SE}(3)}\!\bigl(T(q)^{-1} T^*\bigr).
$$

The IK problem is "find $q$ such that $r(q, T^*) = 0$." Call the
solution $q^*(T^*)$.

We want $\partial q^* / \partial T^*$. We could:

- **Differentiate the algorithm.** Take Newton's method or DLS, unroll
  $N$ iterations, autodiff through every line. Painful.
- **Use the implicit theorem.** Just look at the fixed-point condition
  $r = 0$ and read off the derivative from $\partial r / \partial q$
  and $\partial r / \partial T^*$ alone. Fast, exact, robust.

## The general principle

If your code computes something by *solving an equation*, you almost
certainly don't want to autodiff through the solver. You want the
implicit function theorem.

This pattern appears all over numerical computing:

- **ODE integrators.** Solving $\dot x = f(x, t)$ for $x(T)$. The
  *adjoint state method* is the implicit theorem applied to the
  Hamiltonian fixed-point.
- **Optimisation.** Solving $\min_x f(x, \theta)$ for the optimal
  $x^*(\theta)$. Differentiating w.r.t. $\theta$ uses the implicit
  theorem on the KKT conditions.
- **Equilibrium problems.** Solving $g(x) = 0$ where $g$ is some
  forward model. Same theorem.

If you spot "I'm computing a value by iterating to a fixed point" in
your code and you're considering autodiff, stop and apply the implicit
theorem instead. It's almost always cheaper, more stable, and easier
to reason about.

> ## Where this lives in the library
>
> The implicit theorem is applied in
> [`differentiable.hpp:52-66`](../../include/tinyspatial/ik/differentiable.hpp#L52-L66).
> Sub-chapter 03 walks through the derivation.
