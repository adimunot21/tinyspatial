# Axis–angle

Here is the most *physical* way to describe a rotation: give an **axis** to spin
about (a unit vector $\hat{u}$) and an **angle** $\theta$ to spin by. Every 3-D
rotation can be written this way — a fact called Euler's rotation theorem.

We usually pack the two into one vector:

$$
\omega = \theta\,\hat{u} \in \mathbb{R}^3,
$$

the **rotation vector**. Its direction is the axis, its length is the angle.
This is beautiful for three reasons:

- it's exactly three numbers for three degrees of freedom — no redundancy;
- $\omega = 0$ is the identity (no axis, no angle), with no special-casing;
- it's the natural input to the exponential map (chapter 04), the bridge to
  matrices and quaternions.

## From axis–angle to a matrix: Rodrigues' formula

Given $\omega = \theta\hat{u}$, the rotation matrix is

$$
R = I + \sin\theta\,[\hat{u}]_\times + (1-\cos\theta)\,[\hat{u}]_\times^2,
$$

where $[\hat{u}]_\times$ is the **skew-symmetric** (or "hat") matrix built from
$\hat{u}$:

$$
[u]_\times = \begin{bmatrix} 0 & -u_z & u_y \\ u_z & 0 & -u_x \\ -u_y & u_x & 0 \end{bmatrix}.
$$

The hat matrix is just the cross product in disguise: $[u]_\times\,w = u \times
w$. It shows up everywhere in robotics, so we give it a name in code: `skew()`.

## The catch: small and large angles

Rodrigues' formula has $\theta$ in denominators once you express it in terms of
$\omega$ (because $\hat u = \omega/\theta$). When $\theta \to 0$, you're dividing
by something tiny — numerically dangerous. The fix is a **Taylor expansion**
near zero: for small $\theta$, $\sin\theta/\theta \approx 1 - \theta^2/6$ and so
on, which has no division blow-up.

You'll see this pattern — "closed form for normal angles, Taylor series for tiny
angles" — all through `so3.hpp`. It's not optional polish; without it the
library would return `NaN` for rotations near the identity, which happen
constantly.

The *other* delicate spot is $\theta$ near $\pi$ (a half-turn). Recovering the
axis from a matrix there is ill-conditioned. We sidestep it entirely by storing
rotations as quaternions and taking the log from the quaternion — see the next
chapter and `SO3::log()`.

## Where this lives in the library

| Concept | File · symbol |
| ------- | ------------- |
| The hat operator $[u]_\times$ | [`so3.hpp`](../../include/tinyspatial/liegroup/so3.hpp) · `skew()` |
| Its inverse | [`so3.hpp`](../../include/tinyspatial/liegroup/so3.hpp) · `unskew()` |
| Rotation vector → rotation (with Taylor guard) | [`so3.hpp`](../../include/tinyspatial/liegroup/so3.hpp) · `SO3::exp()` |
| Small-angle threshold | [`so3.hpp`](../../include/tinyspatial/liegroup/so3.hpp) · `kSmallAngle` |

Next: [Quaternions](04_quaternions.md).
