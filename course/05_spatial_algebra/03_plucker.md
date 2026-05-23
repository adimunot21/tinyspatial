# The Plücker transform

A **Plücker transform** is the $6\times6$ matrix that re-expresses a spatial
6-vector when you change frames. It is what Featherstone calls $X$ in
*Rigid Body Dynamics Algorithms*. We met it in chapter 04 under a different
name — the SE(3) adjoint — and it is the *same object*.

So why the second name? Because the dynamics literature speaks Featherstone's
dialect, and `plucker_motion(T)` reads truer to the textbook than
`T.adjoint()`. Both call the same function.

## Two flavours, one transform

In our angular-first convention:

- **Motion Plücker** $X_m(T) = \mathrm{Ad}_T$ transports a twist.
- **Force Plücker** $X_f(T) = \mathrm{Ad}_T^{-\top}$ transports a wrench.

They differ by an inverse-transpose. In closed form, with $T = (R, t)$:

$$
X_m(T) = \begin{bmatrix} R & 0 \\ [t]_\times R & R \end{bmatrix},
\qquad
X_f(T) = \begin{bmatrix} R & [t]_\times R \\ 0 & R \end{bmatrix}.
$$

Same $R$ blocks, but the off-diagonal coupling lives in different corners. The
library provides both as one-liners:

```cpp
Matrix6 X  = motion_plucker(t);   // == t.adjoint()
Matrix6 Xs = force_plucker(t);    // == t.adjoint().transpose().inverse()
```

## What it costs to transport a 6-vector

A Plücker transform is **six** $3\times3$ multiplies and a few adds — cheaper
than a $6\times6$ multiply if you exploit the block-zero. Featherstone-style
algorithms apply Plückers tens of times per dynamics evaluation, so this is the
inner loop that matters. Our implementations are deliberately straightforward
right now; the performance phase (Phase 9) will revisit this hot path.

## Composition

Composing two transforms composes their Plückers: $X(T_1 T_2) = X(T_1)\,X(T_2)$.
That's just the homomorphism property of the adjoint. It means you can chain
frames by chaining transforms — exactly what forward kinematics will do.

## Where this lives in the library

| Concept | File · symbol |
| ------- | ------------- |
| Motion Plücker | [`plucker.hpp`](../../include/tinyspatial/spatial/plucker.hpp) · `motion_plucker()` |
| Force Plücker | [`plucker.hpp`](../../include/tinyspatial/spatial/plucker.hpp) · `force_plucker()` |
| Equivalent adjoint | [`se3.hpp`](../../include/tinyspatial/liegroup/se3.hpp) · `SE3::adjoint()` |
| Unit-twist test | [`test_inertia.cpp`](../../tests/unit/spatial/test_inertia.cpp) · `PluckerOfUnitTwistMatchesAdjoint` |

Next: [Spatial inertia](04_spatial_inertia.md).
