# 13.5 — Differentiable *everything*: the Jet approach

The first four sub-chapters got you `∂q*/∂T*` for IK using the implicit
function theorem — a hand-derived, analytical derivative. That's the
right tool when you know the formula. But what if you want the
derivative of something you *haven't* hand-derived? The gradient of a
custom cost built on forward kinematics? The sensitivity of a joint
torque to the configuration?

There's a second route, and it's the one that makes `tinyspatial`
unusual: **the whole library is differentiable, in pure C++, with no
external autodiff dependency.** You don't write a single new derivative
formula. You change the *number type* the algorithms run on.

## Dual numbers in one paragraph

A **dual number** carries a value and a derivative together:
$x = a + b\,\varepsilon$, where $\varepsilon^2 = 0$. Push it through any
function built from `+`, `*`, `sin`, `sqrt`, … and the chain rule does
itself: the value rides in $a$ and the exact derivative falls out in
$b$. Generalise $b$ from one number to an $N$-vector of partials and you
can track derivatives w.r.t. $N$ inputs at once. That generalised dual
number is [`Jet<N>`](../../include/tinyspatial/core/jet.hpp) — about 300
lines of header, modelled on Ceres Solver's `Jet`. It is a *drop-in
Eigen scalar*: `Eigen::Matrix<Jet<N>, …>` and even
`Eigen::Quaternion<Jet<N>>` just work.

This is **forward-mode** automatic differentiation. It's the right mode
for robotics: the input dimension (the number of joints, $n_v$) is
small, and the Jacobians we want are tall — $\partial(\text{pose})/
\partial q$, $\partial\tau/\partial q$ — which forward mode delivers
cheaply with $N = n_v$.

## How you actually use it

Every algorithm — `forward_kinematics`, `rnea`, `crba`, `aba` — is
templated on the scalar, with `double` the default. To differentiate,
do three things:

1. **Lift** your `double` model to the autodiff scalar with
   `model_cast<Jet<N>>(model)`. The robot's constants (link lengths,
   masses, joint axes) come along as values with *zero* derivative.
2. **Seed** the variables you're differentiating with respect to:
   `q(k) = Jet<N>(value, k)` plants a unit derivative in slot `k`.
3. **Run** the ordinary algorithm and **read** the partials out of the
   result's `.v` array.

Here is the gravity-torque gradient of a 2-link arm, in full
(`src/examples/differentiable_dynamics.cpp`):

```cpp
using J = Jet<2>;
const ModelT<J> ad_model = model_cast<J>(model);
DataT<J> data(ad_model);

typename Types<J>::VectorX q(2), v(2), a(2), tau(2);
for (int k = 0; k < 2; ++k) {
  q(k) = J(q_value(k), k);  // seed: ∂q_k/∂q_k = 1
  v(k) = J(0.0);
  a(k) = J(0.0);
}

rnea(ad_model, data, q, v, a, tau, gravity);
// tau(r).a    is the torque value;
// tau(r).v[k] is ∂τ_r / ∂q_k — the full gravity-stiffness matrix, exact.
```

No finite differences (which would force you to pick a step size and
eat truncation + round-off error), no second implementation to keep in
sync, no Python, no JAX. The *same* `rnea` you ship for production runs
differentiably.

## Why this is trustworthy

A from-scratch autodiff path is only worth anything if it's correct.
`tinyspatial` pins it down with the strongest test available: it already
ships **hand-written analytical derivatives** (chapter 10b's
Carpentier–Mansard RNEA derivatives, and the per-joint FK Jacobians),
so the autodiff result is checked against a completely independent
derivation:

- FK: the `Jet` Jacobian matches
  [`compute_joint_jacobians`](../../include/tinyspatial/diff/fk_derivatives.hpp)
  to $10^{-10}$.
- RNEA: `Jet` $\partial\tau/\partial q,\ \partial\tau/\partial v,\
  \partial\tau/\partial a$ match the analytical recursion, and
  $\partial\tau/\partial a$ independently equals the CRBA mass matrix
  $M(q)$.
- ABA: $\partial\ddot q/\partial\tau$ equals $M(q)^{-1}$ — forward
  dynamics' derivative tied back to CRBA.

Two (sometimes three) independently-derived derivative paths agreeing
to machine precision. That's a much stronger guarantee than "the test
passed."

## When to use which

| You want… | Use |
| --------- | --- |
| `∂q*/∂T*` for IK | the analytical IFT result (sub-chapters 1–4) — one Jacobian, one solve |
| `∂τ/∂q`, `∂τ/∂v`, `∂M/∂q`, … for dynamics | analytical (chapter 10b) for speed, or `Jet` for anything not hand-derived |
| the gradient of a *custom* function of `q` | `Jet` — it differentiates whatever you compose |

The analytical paths are faster; the `Jet` path is general and needs no
new math. They validate each other.

## Where this lives in the library

| Concept | File |
| ------- | ---- |
| Forward-mode autodiff scalar | [`include/tinyspatial/core/jet.hpp`](../../include/tinyspatial/core/jet.hpp) |
| Scalar-generic type aliases | [`include/tinyspatial/core/types.hpp`](../../include/tinyspatial/core/types.hpp) |
| Lift a model to a new scalar | [`model_cast` in `model.hpp`](../../include/tinyspatial/model/model.hpp) |
| Worked example | [`src/examples/differentiable_dynamics.cpp`](../../src/examples/differentiable_dynamics.cpp) |
| FK autodiff vs analytical | [`tests/unit/diff/test_fk_ad.cpp`](../../tests/unit/diff/test_fk_ad.cpp) |
| RNEA autodiff vs analytical + CRBA | [`tests/unit/diff/test_rnea_ad.cpp`](../../tests/unit/diff/test_rnea_ad.cpp) |
| CRBA / ABA autodiff | [`tests/unit/diff/test_dynamics_ad.cpp`](../../tests/unit/diff/test_dynamics_ad.cpp) |

## Further reading

- **Ceres Solver's `jet.h`** — the canonical small forward-mode `Jet`;
  ours is a trimmed cousin. Read it for the full math-function set.
- **Andreas Griewank & Andrea Walther**, *Evaluating Derivatives* — the
  textbook on automatic differentiation, forward and reverse modes.
- **Justin Carpentier & Nicolas Mansard**, "Analytical Derivatives of
  Rigid Body Dynamics Algorithms" (RSS 2018) — the *analytical* path the
  autodiff result is validated against.
