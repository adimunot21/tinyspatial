# Twists and wrenches

Two kinds of 6-vector appear constantly in spatial algebra. They look alike on
paper and crash into each other on the page; the goal of this chapter is to
keep them properly distinct.

## Twist: a spatial velocity

A **twist** $m = (\omega, v)$ is a velocity 6-vector — the instantaneous motion
of a rigid body. The angular part is its spin; the linear part is the velocity
of the body's reference point.

In code that's `tinyspatial::Motion`. It transforms under the SE(3) adjoint:

$$
m' = \mathrm{Ad}_T \, m.
$$

We met it in chapter 04. `Motion` wraps a `Vector6`, exposes `angular()` and
`linear()`, and supports the usual `+`, `-`, scalar multiplication.

## Wrench: a spatial force

A **wrench** $f = (\tau, F)$ is a force 6-vector — a moment $\tau$ and a linear
force $F$ acting on a body. In code that's `tinyspatial::Force`.

Wrenches do **not** transform with the adjoint. They transform with the *dual*
adjoint, $\mathrm{Ad}_T^{-\top}$:

$$
f' = \mathrm{Ad}_T^{-\top} \, f.
$$

The dual exists for a reason: it keeps **power frame-invariant.** If a wrench
$f$ does work against a twist $m$ at one frame at rate $f \cdot m$, the same
physical power must be measured at any other frame as
$(\mathrm{Ad}_T^{-\top} f) \cdot (\mathrm{Ad}_T \, m) = f \cdot m$ (you can see
the $\mathrm{Ad}_T^{-\top}\mathrm{Ad}_T = I$ cancellation explicitly). The
library test `DualityPower` checks this identity numerically — and if you ever
flipped the adjoint and its dual, that test would catch you instantly.

## Why two C++ types?

Mathematically, a twist and a wrench are both points in $\mathbb{R}^6$. Adding
them is a *type error* — you wouldn't compute "1 metre + 2 kilograms" — but in
plain C++ they'd both be a `Vector6` and the compiler couldn't tell.

`tinyspatial` makes them distinct types. Try writing `Motion m; Force f; m + f;`
and you get a compile error instead of nonsense. Cross products that take a
twist *and* a force return a force (`cross(Motion, Force) -> Force`); ones that
take two twists return a twist (`cross(Motion, Motion) -> Motion`). The
operator overloads enforce the algebra.

This costs ~50 lines per type and saves hours of debugging across the rest of
the library. It is the single highest-leverage type-safety decision in the
codebase.

## Where this lives in the library

| Concept | File · symbol |
| ------- | ------------- |
| Twist | [`spatial/motion.hpp`](../../include/tinyspatial/spatial/motion.hpp) · `class Motion` |
| Wrench | [`spatial/force.hpp`](../../include/tinyspatial/spatial/force.hpp) · `class Force` |
| Adjoint action on twist | `operator*(SE3, Motion)` |
| Dual-adjoint action on wrench | `operator*(SE3, Force)` |
| Typed cross products | [`spatial/cross.hpp`](../../include/tinyspatial/spatial/cross.hpp) · `cross()` |
| Power identity test | [`test_motion_force.cpp`](../../tests/unit/spatial/test_motion_force.cpp) · `DualityPower` |

Next: [The Plücker transform](03_plucker.md).
