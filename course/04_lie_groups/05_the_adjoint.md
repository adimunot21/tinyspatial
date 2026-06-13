# The adjoint

A velocity is only meaningful *relative to a frame*. The angular velocity of a
robot's forearm "expressed in the forearm frame" and "expressed in the base
frame" are different 6-vectors describing the same physical motion. The
**adjoint** is the map that re-expresses a twist from one frame in another.

## What it does

Given a rigid transform $T = (R, t)$ that relates frame B to frame A, the
adjoint $\mathrm{Ad}_T$ is a $6\times6$ matrix that converts a twist written in
B's frame into the same twist written in A's frame. In our angular-first
convention:

$$
\mathrm{Ad}_T = \begin{bmatrix} R & 0 \\ [t]_\times R & R \end{bmatrix}.
$$

Read the blocks: the angular part is rotated ($R$). The linear part is rotated
*and* picks up a $[t]_\times R\,\omega$ term, because a pure rotation viewed from a
translated origin also exhibits a linear velocity — a point on a spinning wheel
seen from the axle versus from the rim.

## In code: `SE3T::adjoint`

The matrix is assembled block by block, exactly matching the formula above:

```cpp
[[nodiscard]] Matrix6 adjoint() const {
  const Matrix3 r = rotation_.matrix();
  Matrix6 ad = Matrix6::Zero();
  ad.template topLeftCorner<3, 3>() = r;
  ad.template bottomRightCorner<3, 3>() = r;
  ad.template bottomLeftCorner<3, 3>() = skew(translation_) * r;
  return ad;
}
```

The two diagonal blocks are the rotation $R$; the single off-diagonal block is the
coupling term $[t]_\times R$. The angular-first ordering is what places that
coupling in the *bottom-left* (angular → linear) rather than the top-right; under
Pinocchio's linear-first convention the same physics lands in the opposite corner.
The inverse adjoint is obtained without a separate derivation, using
$\mathrm{Ad}_T^{-1} = \mathrm{Ad}_{T^{-1}}$:

```cpp
[[nodiscard]] Matrix6 adjoint_inverse() const { return inverse().adjoint(); }
```

## The defining identity

The adjoint is exactly what makes "change of frame" consistent with `exp`:

$$
\exp(\mathrm{Ad}_T\,\xi) = T\,\exp(\xi)\,T^{-1}.
$$

"Re-express the velocity, then move" equals "move, as seen through the
transform." The library checks this directly in `Se3Test.AdjointDefiningIdentity`
— a strong test, because it ties together `exp`, composition, inverse, *and* the
adjoint in one equation.

## The Lie bracket: the adjoint of a velocity

There's a tiny-motion version too. Differentiate $\mathrm{Ad}_{\exp(t\xi)}$ at
$t=0$ and you get a $6\times6$ matrix that depends linearly on the twist $\xi$ —
the **spatial cross product**, written $\xi\times$ (Featherstone's notation).
This is `cross_motion()` in the code, and it satisfies

$$
\left.\frac{d}{dt}\right|_{0}\mathrm{Ad}_{\exp(t\xi)} = \xi\times,
$$

which `CrossTest.CrossMotionIsAdjointDerivative` verifies by finite difference.
The cross product is the **Lie bracket** of $\mathfrak{se}(3)$: it measures how
two infinitesimal motions fail to commute, and it obeys the Jacobi identity
(also tested). There's a partner operator $\xi\times^* = -(\xi\times)^\top$ for
forces (wrenches) instead of motions.

## Why this matters for dynamics

When we get to Featherstone's algorithms (chapters 10–11), spatial velocities and
forces are constantly shuttled between link frames. *Every* one of those
shuttles is an adjoint, and the cross products appear in the velocity-product
(Coriolis/centrifugal) terms. Getting the adjoint and cross products right here,
with tests, is what makes those algorithms tractable later.

## Where this lives in the library

| Concept | File · symbol |
| ------- | ------------- |
| $\mathrm{Ad}_T$ and its inverse | [`se3.hpp`](../../include/tinyspatial/liegroup/se3.hpp) · `SE3::adjoint()`, `adjoint_inverse()` |
| Motion cross product $\xi\times$ | [`cross.hpp`](../../include/tinyspatial/spatial/cross.hpp) · `cross_motion()` |
| Force cross product $\xi\times^*$ | [`cross.hpp`](../../include/tinyspatial/spatial/cross.hpp) · `cross_force()` |
| Adjoint + cross tests | [`test_se3.cpp`](../../tests/unit/liegroup/test_se3.cpp), [`test_cross.cpp`](../../tests/unit/spatial/test_cross.cpp) |

Next: [Why robots need this](06_why_robots_need_this.md).
