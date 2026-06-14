# exp and log

There are two spaces to work in: the flat **tangent space** (velocities, twists),
where ordinary vector algebra applies, and the curved **manifold** (rotations and
transforms), where it does not. The two maps connecting them are the
**exponential** (`exp`, flat → curved) and the **logarithm** (`log`, curved →
flat).

## exp: integrate a constant velocity

Hold an angular velocity $\omega$ constant for one unit of time. Spinning at rate
$\|\omega\|$ about the axis $\omega/\|\omega\|$ for unit time produces a rotation
of angle $\|\omega\|$ about that axis — exactly the axis–angle rotation from
Chapter 03. Hence

$$
\exp(\omega) = \text{rotation by } \|\omega\| \text{ about } \omega/\|\omega\|,
$$

which is Rodrigues' formula: `exp` follows a velocity for unit time and reports
where the body lands, mapping a straight arrow in the tangent space onto the
curved manifold. For $SE(3)$, `exp` of a twist $\xi=(\omega,v)$ does the same for
the combined rotation-and-translation (a screw motion).

## log: recover the velocity that produced a transform

`log` inverts `exp`: given a rotation, it returns the constant velocity that,
held for unit time, produces it — the rotation vector $\omega$, the axis scaled by
the angle, with $\|\omega\| \le \pi$. For a transform, `log` returns the twist.
The two maps are genuine inverses on the natural domain:

$$
\log(\exp(\omega)) = \omega \;\; (\|\omega\| < \pi), \qquad \exp(\log(R)) = R.
$$

The library tests both directions to $10^{-12}$ (`So3Test.LogExpRoundTrip`,
`Se3Test.ExpLogRoundTrip`).

## In code: `SO3T::exp`

The implementation in [`so3.hpp`](../../include/tinyspatial/liegroup/so3.hpp)
builds the rotation as a quaternion rather than a matrix:

```cpp
[[nodiscard]] static SO3T exp(const Eigen::Ref<const Vector3>& omega) {
  using std::cos;
  using std::sin;
  using std::sqrt;
  const Scalar theta2 = omega.squaredNorm();
  const Scalar theta = sqrt(theta2);
  Quaternion q;
  if (theta > kSmallAngle) {
    const Scalar half = Scalar(0.5) * theta;
    q.w() = cos(half);
    q.vec() = (sin(half) / theta) * omega;  // sin(θ/2)/θ · ω
  } else {
    // sin(θ/2)/θ = 1/2 − θ²/48 + … ;  cos(θ/2) = 1 − θ²/8 + …
    q.w() = Scalar(1) - theta2 / Scalar(8);
    q.vec() = (Scalar(0.5) - theta2 / Scalar(48)) * omega;
  }
  return SO3T(q);
}
```

- The quaternion of a rotation by $\theta$ about a unit axis $\hat u$ is
  $(\cos\tfrac\theta2,\ \sin\tfrac\theta2\,\hat u)$. With $\omega = \theta\hat u$,
  the vector part is $\tfrac{\sin(\theta/2)}{\theta}\,\omega$ — the form on the
  `q.vec()` line.
- **The branch is the load-bearing detail.** As $\theta \to 0$, the factor
  $\sin(\theta/2)/\theta \to 1/2$ but is computed as $0/0$ in floating point. Below
  `kSmallAngle` ($10^{-3}$ rad) the code substitutes the Taylor expansions, so a
  rotation near the identity is exact instead of `NaN`.
- `theta` is compared against `kSmallAngle` by value. Under the autodiff scalar
  `Jet`, the derivative is carried through whichever branch the value selects, so
  the result is differentiable across the whole domain.

## In code: `SO3T::log`, and why it uses `asin`

Recovering the rotation vector from a *matrix* (the trace–`acos` formula) is
ill-conditioned near a half-turn. The library avoids it by reading the angle from
the stored quaternion:

```cpp
[[nodiscard]] Vector3 log() const {
  using std::asin;
  using std::min;
  // q is canonical (unit, w ≥ 0), so n = ‖vec‖ = sin(θ/2) and θ = 2·asin(n).
  const Vector3 vec = quat_.vec();
  const Scalar n = min(Scalar(vec.norm()), Scalar(1));  // clamp float drift
  Scalar factor(0);
  if (n > kSmallAngle) {
    factor = Scalar(2) * asin(n) / n;  // θ / sin(θ/2)
  } else {
    factor = Scalar(2) * (Scalar(1) + n * n / Scalar(6));
  }
  return factor * vec;
}
```

Because the quaternion is canonical (unit, $w \ge 0$, from Chapter 03), its vector
part has norm $n = \sin(\theta/2)$, so $\theta = 2\arcsin n$ directly — no matrix,
no trace, no cancellation as $\theta \to \pi$. The `min(..., 1)` clamp guards
against floating-point drift pushing $n$ just above 1, which would make `asin`
return `NaN`. This is the single most important numerical decision in `so3.hpp`;
`So3Corner.PiAngle` checks it at the half-turn.

## In code: `SE3T::exp` reuses the SO(3) Jacobian

The transform exponential is built directly on the rotation one:

```cpp
[[nodiscard]] static SE3T exp(const Eigen::Ref<const Vector6>& xi) {
  const Vector3 omega = xi.template head<3>();
  const Vector3 v = xi.template tail<3>();
  return SE3T(SO3::exp(omega), SO3::left_jacobian(omega) * v);
}
```

The rotational part is `SO3::exp(omega)`. The translation is **not** simply `v`:
the linear velocity acts while the frame is rotating, and the net displacement is
$J_l(\omega)\,v$, where $J_l$ is the SO(3) left Jacobian. Those Jacobian
coefficients are the subject of the next section. Note the angular-first slicing —
`head<3>()` is $\omega$, `tail<3>()` is $v$ — the convention fixed in
[`se3.hpp`](../../include/tinyspatial/liegroup/se3.hpp) and used by every spatial
quantity downstream.

## Where this lives in the library

| Concept | File · symbol |
| ------- | ------------- |
| $\exp$ for rotations | [`so3.hpp`](../../include/tinyspatial/liegroup/so3.hpp) · `SO3::exp()` |
| $\log$ for rotations (quaternion-based, stable at $\pi$) | [`so3.hpp`](../../include/tinyspatial/liegroup/so3.hpp) · `SO3::log()` |
| $\exp/\log$ for transforms | [`se3.hpp`](../../include/tinyspatial/liegroup/se3.hpp) · `SE3::exp()`, `SE3::log()` |
| Round-trip tests | [`test_so3.cpp`](../../tests/unit/liegroup/test_so3.cpp), [`test_se3.cpp`](../../tests/unit/liegroup/test_se3.cpp) |

Next: [Jacobians](04_jacobians.md).
