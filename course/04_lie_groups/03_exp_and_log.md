# exp and log

We have two worlds: the flat **tangent space** (velocities, twists — easy to do
algebra in) and the curved **manifold** (actual rotations and transforms). We
need to travel between them. The two maps that do it are the **exponential**
(`exp`, flat → curved) and the **logarithm** (`log`, curved → flat).

## exp: integrate a constant velocity

Take an angular velocity $\omega$ and hold it constant for one unit of time.
Where does the body end up? Spinning at rate $\|\omega\|$ about axis
$\omega/\|\omega\|$ for unit time gives a rotation of angle $\|\omega\|$ about
that axis — exactly the axis–angle rotation from chapter 03! So

$$
\exp(\omega) = \text{rotation by } \|\omega\| \text{ about } \omega/\|\omega\|,
$$

which is Rodrigues' formula. **`exp` is "follow this velocity for one second and
see where you land."** It wraps the straight arrow in the tangent space onto the
curved manifold. For $SE(3)$, `exp` of a twist $\xi=(\omega,v)$ does the same for
the combined spin-and-slide motion (a screw motion).

## log: recover the velocity that got you there

`log` is the inverse: given a rotation, what constant velocity (held for unit
time) produces it? That's the rotation vector $\omega$ — the axis scaled by the
angle, with $\|\omega\| \le \pi$. For a transform, `log` returns the twist.

These are genuine inverses on a sensible domain:

$$
\log(\exp(\omega)) = \omega \;\; (\|\omega\| < \pi), \qquad \exp(\log(R)) = R.
$$

The library tests both directions to $10^{-12}$ (`So3Test.LogExpRoundTrip`,
`Se3Test.ExpLogRoundTrip`).

## The two hard cases (and how the code handles them)

1. **Near zero.** As $\|\omega\| \to 0$, the closed forms divide by the angle.
   The code switches to Taylor series below `kSmallAngle`. Without this, a
   rotation near the identity returns `NaN`.
2. **Near $\pi$ (a half-turn).** Recovering the axis from a *matrix* is
   ill-conditioned here. We dodge it: `SO3::log()` computes the angle from the
   stored quaternion as $\theta = 2\arcsin\|q_{\text{vec}}\|$, which stays
   well-behaved all the way to $\pi$. (`So3Corner.PiAngle` checks it.) This is
   the single most important numerical decision in `so3.hpp`.

## Mental model

Think of `exp` and `log` as a chart-and-territory pair:

- **`log`**: unroll the curved manifold onto the flat map near the identity.
- **`exp`**: roll the flat map back onto the manifold.

Do your hard work (averaging, optimising, finite-differencing) on the flat map;
travel with `exp`/`log`.

## Where this lives in the library

| Concept | File · symbol |
| ------- | ------------- |
| $\exp$ for rotations | [`so3.hpp`](../../include/tinyspatial/liegroup/so3.hpp) · `SO3::exp()` |
| $\log$ for rotations (quaternion-based, stable at $\pi$) | [`so3.hpp`](../../include/tinyspatial/liegroup/so3.hpp) · `SO3::log()` |
| $\exp/\log$ for transforms | [`se3.hpp`](../../include/tinyspatial/liegroup/se3.hpp) · `SE3::exp()`, `SE3::log()` |
| Round-trip tests | [`test_so3.cpp`](../../tests/unit/liegroup/test_so3.cpp), [`test_se3.cpp`](../../tests/unit/liegroup/test_se3.cpp) |

Next: [Jacobians](04_jacobians.md).
