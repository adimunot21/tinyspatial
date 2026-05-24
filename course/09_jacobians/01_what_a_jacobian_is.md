# What a Jacobian is

You have a robot. You move the motors at velocities $\dot q$. The hand
moves with spatial velocity $v$ (a twist; chapters 04 and 05). The
relationship between them is *linear* at any instant:

$$
v \;=\; J(q)\,\dot q.
$$

The matrix $J(q)$ is the **Jacobian** of the link's pose w.r.t. the joint
configuration `q`. Its size is $6 \times n_v$: six rows (one per twist
coordinate, angular-first in our convention), and one column per velocity
coordinate.

## Each column has a meaning

Column $j$ of $J(q)$ is the spatial velocity the link would acquire if joint
$j$ moved at unit velocity and every other joint stayed still. That's the
**screw axis** of joint $j$, expressed in the chosen reference frame.

This makes Jacobians much easier to reason about than they look at first:
each column has a direct physical reading.

- **Revolute joint** column: a rotation about the joint axis (angular part)
  with the linear part being "how the rotation about that axis drags the
  link's reference point through space."
- **Prismatic joint** column: a pure translation along the joint axis (linear
  part), with zero angular part.
- **Fixed joint** column: nothing — fixed joints don't appear in `J`.

## It's the derivative of FK

Formally, $J(q)$ is the differential of $\mathrm{FK}_L : q \mapsto T_L(q)
\in \mathrm{SE}(3)$, with the manifold structure of SE(3) taken into account
(the rows are a twist, not the entries of a 4×4 matrix). The library tests
that explicitly:

```
J_local(q) · δ  ≈  log( T(q)^{-1} · T(q + δ) )    for small δ
```

— a finite-difference statement of "$J$ is the derivative of $\log \circ \mathrm{FK}$
at the identity." Pass that check at `1e-6` for 1000 random `q` per robot
and you can be reasonably sure your analytical Jacobian is correct *before*
you reach for Pinocchio.

## Why this matters

Anywhere you'd want to "go in the direction of the gradient" of an end-
effector quantity in joint space, you use the Jacobian (or its inverse /
pseudo-inverse / damped-pseudo-inverse) to convert. Concretely:

- **Inverse kinematics** (chapter 12): take a Cartesian error, ask "how
  should I move the joints to reduce it?" — that's $J^{+}$ times the error.
- **Resolved-rate motion**: command "move the hand at this twist" — that's
  $J^{+}$ times the desired twist.
- **Force / torque mapping**: a wrench at the end-effector projects to joint
  torques by $J^\top$ (a fact that follows from duality, chapter 05).

Every one of these starts with the Jacobian at the current configuration.

## Where this lives in the library

| Concept | File · symbol |
| ------- | ------------- |
| Jacobian function | [`jacobian.hpp`](../../include/tinyspatial/algo/jacobian.hpp) · `compute_jacobian()` |
| Storage type | [`core/types.hpp`](../../include/tinyspatial/core/types.hpp) · `Matrix6X` |
| Definition + FD test | [`test_jacobian.cpp`](../../tests/unit/algo/test_jacobian.cpp) · `LocalAgreesWithFiniteDifference` |

Next: [Geometric vs analytical](02_geometric_vs_analytical.md).
