# Twists and wrenches

Two kinds of 6-vector appear constantly in spatial algebra. They look alike on
paper and crash into each other on the page; the goal of this chapter is to
keep them properly distinct.

## Twist: a spatial velocity

A **twist** $m = (\omega, v)$ is a velocity 6-vector — the instantaneous motion
of a rigid body. The angular part is its spin; the linear part is the velocity
of the body's reference point.

In code that is `tinyspatial::Motion` (the alias for `MotionT<double>`). It
transforms under the SE(3) adjoint:

$$
m' = \mathrm{Ad}_T \, m.
$$

The type wraps a single `Vector6` in angular-first order and exposes the parts:

```cpp
template <typename S>
class MotionT {
 public:
  MotionT(const Eigen::Ref<const Vector3>& angular, const Eigen::Ref<const Vector3>& linear) {
    data_.template head<3>() = angular;   // indices 0..2 — ω
    data_.template tail<3>() = linear;    // indices 3..5 — v
  }
  [[nodiscard]] Vector3 angular() const { return data_.template head<3>(); }
  [[nodiscard]] Vector3 linear() const { return data_.template tail<3>(); }
  [[nodiscard]] MotionT operator+(const MotionT& rhs) const { return MotionT(data_ + rhs.data_); }
  // …  operator-, unary minus, scalar *, +=, -=
 private:
  Vector6 data_;
};
using Motion = MotionT<double>;
```

The class adds nothing to a bare `Vector6` numerically — its value is the *type
distinction* developed below, and the angular-first storage that every algorithm
relies on.

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

## In code: the adjoint action, expanded

The SE(3) action on a motion is a free function, and it does **not** build the
$6\times6$ adjoint. It expands $\mathrm{Ad}_T = [[R, 0], [[t]_\times R, R]]$ by
hand:

```cpp
template <typename S>
[[nodiscard]] MotionT<S> operator*(const SE3T<S>& t, const MotionT<S>& m) {
  const typename Types<S>::Matrix3 r = t.rotation().matrix();
  const typename Types<S>::Vector3 new_w = r * m.angular();
  const typename Types<S>::Vector3 new_v = r * m.linear() + t.translation().cross(new_w);
  return MotionT<S>(new_w, new_v);
}
```

The angular part is rotated; the linear part is rotated and gains the coupling
term $t \times (R\omega)$. This is roughly 24 floating-point operations against
the ~72 of constructing the dense adjoint and multiplying — and it is on the hot
path, since forward kinematics and the dynamics algorithms apply this action once
per joint per call. The dual action on a `Force` is the matching expansion in
[`force.hpp`](../../include/tinyspatial/spatial/force.hpp).

## Why two C++ types

Mathematically a twist and a wrench are both points in $\mathbb{R}^6$. Adding them
is a category error — "1 metre + 2 kilograms" — but as bare `Vector6` values the
compiler cannot tell them apart. The library makes them distinct types: `Motion m;
Force f; m + f;` is a compile error rather than silent nonsense. The typed cross
products enforce the algebra in the same way — `cross_force(Motion, Force)`
returns a `Force`, `cross_motion(Motion, Motion)` returns a `Motion` (next
section). This costs roughly 50 lines per type and is the highest-leverage
type-safety decision in the codebase.

## Where this lives in the library

| Concept | File · symbol |
| ------- | ------------- |
| Twist | [`spatial/motion.hpp`](../../include/tinyspatial/spatial/motion.hpp) · `class Motion` |
| Wrench | [`spatial/force.hpp`](../../include/tinyspatial/spatial/force.hpp) · `class Force` |
| Adjoint action on twist | `operator*(SE3, Motion)` |
| Dual-adjoint action on wrench | `operator*(SE3, Force)` |
| Typed cross products | [`spatial/cross.hpp`](../../include/tinyspatial/spatial/cross.hpp) · `cross_motion`, `cross_force` |
| Power identity test | [`test_motion_force.cpp`](../../tests/unit/spatial/test_motion_force.cpp) · `DualityPower` |

Next: [The Plücker transform](03_plucker.md).
