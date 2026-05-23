# Jacobians

Here's a subtlety that trips up everyone the first time. On a flat space,
$\exp(a+b) = \exp(a)\exp(b)$. On a curved Lie group it is **not**, because the
group doesn't commute. So if you nudge the tangent vector by a little $\delta$,
the change on the manifold is *not* simply $\exp(\delta)$ — it's distorted by the
curvature. The object that captures that distortion is the **Jacobian** of the
exponential map.

## What the Jacobian does

Concretely, for a small perturbation $\delta$ of the tangent vector $\omega$:

$$
\exp(\omega + \delta) \approx \exp(\omega)\,\exp\!\big(J_r(\omega)\,\delta\big).
$$

$J_r(\omega)$, the **right Jacobian**, is the linear map that tells you how a
wiggle in the tangent space *at the origin* shows up as a wiggle *at the point
$\exp(\omega)$*, measured on the right. There's a matching **left Jacobian**
$J_l$ for wiggles measured on the left:

$$
\exp(\omega + \delta) \approx \exp\!\big(J_l(\omega)\,\delta\big)\,\exp(\omega).
$$

They're simply related: $J_l(\omega) = J_r(\omega)^\top = J_r(-\omega)$.

## Why you care

Any time you **differentiate through** a rotation or transform, the Jacobian
appears:

- Optimising on the manifold (Gauss–Newton for IK, chapter 12) needs the
  derivative of the error with respect to a tangent step — that's a Jacobian.
- Propagating uncertainty (covariance) through a transform uses it.
- The *inverse* Jacobian converts a manifold error back into a tangent
  correction — the workhorse of on-manifold optimisation.

Near the identity the Jacobian is just $I$ (a tiny wiggle is undistorted); the
curvature corrections grow with the angle.

## How we know ours are right

These formulas are easy to mis-derive (a wrong sign on a $\theta^2$ term hides
for ages). So we don't trust the algebra alone — we **finite-difference** the
definition above and compare to the closed form. Pick $\omega$, perturb each
component, measure the actual change on the manifold, and check it matches
$J_r(\omega)$ to $10^{-7}$. That's `So3Test.JacobiansMatchFiniteDifference` and
its SE(3) sibling. The inverses are checked by $J \cdot J^{-1} = I$.

This "analytical formula, validated against finite differences" pattern is how
the whole library earns trust before it ever meets Pinocchio.

## SE(3): the extra term

The $SE(3)$ Jacobian is $6\times6$ and block-structured. The rotational blocks
are just the $SO(3)$ Jacobian, but there's an off-diagonal coupling block —
Barfoot's **Q matrix** — capturing how a rotational wiggle drags the translation
around. It's intricate, which is exactly why the finite-difference test is
non-negotiable.

## Where this lives in the library

| Concept | File · symbol |
| ------- | ------------- |
| SO(3) right/left Jacobians + inverses | [`so3.hpp`](../../include/tinyspatial/liegroup/so3.hpp) · `right_jacobian`, `left_jacobian`, `*_inverse` |
| SE(3) Jacobians (with Barfoot Q) | [`se3.hpp`](../../include/tinyspatial/liegroup/se3.hpp) · `left_jacobian`, `left_jacobian_q` |
| Finite-difference validation | [`test_so3.cpp`](../../tests/unit/liegroup/test_so3.cpp), [`test_se3.cpp`](../../tests/unit/liegroup/test_se3.cpp) |

Next: [The adjoint](05_the_adjoint.md).
