# The equation of motion

For any rigid-body system with $n$ joints, the laws of motion can be written
in joint space as

$$
\tau \;=\; M(q)\,\ddot q \;+\; C(q, \dot q)\,\dot q \;+\; g(q).
$$

Three groups of forces, all expressed as joint torques. Let's read them one
at a time.

## $M(q)\,\ddot q$ — the inertial term

If you push the joints with acceleration $\ddot q$, you have to fight the
inertia of every link the joint is carrying. The **joint-space inertia
matrix** $M(q) \in \mathbb{R}^{n_v \times n_v}$ captures exactly how that
inertia couples joints to each other. It is:

- **Symmetric**: $M(q) = M(q)^\top$.
- **Positive definite**: $\dot q^\top M(q) \dot q > 0$ for any nonzero
  $\dot q$ (this is just twice the kinetic energy).
- **Configuration-dependent**: the inertia your motors fight changes as the
  robot reshapes.

Diagonal entries are "the inertia I see at joint $j$ when I move only joint
$j$." Off-diagonal entries are coupling: moving joint $j$ requires torque at
joint $k$ even at $\ddot q_k = 0$, because joint $k$ has to push joint $j$'s
mass around in a particular way.

We compute $M(q)$ explicitly in chapter 11 (CRBA). For RNEA we don't need it
in closed form — we evaluate $M(q)\,\ddot q$ directly.

## $C(q, \dot q)\,\dot q$ — Coriolis and centrifugal

Velocities couple too, and they do it nonlinearly. Imagine spinning a robot's
shoulder while the elbow is moving: there is a force on the elbow link that
exists *only because both joints are moving*. That's centrifugal /
Coriolis force, and $C(q, \dot q)\,\dot q$ collects all of it.

A precise property: this term is *exactly* what you get when you plug $\ddot q
= 0$, $\dot q = $ something, and $g = 0$ into RNEA. Velocity-coupled forces
*are* the inward propagation of $v \times^* (I\,v)$ — the "this body's
angular momentum changes direction because the body is spinning"
contribution. Section 03 will show it in code.

## $g(q)$ — gravity

If everything's stationary and you let go, the arm falls. The torques the
motors would need to *hold* the arm against gravity are $g(q)$: the
gravitational bias.

Setting $\dot q = \ddot q = 0$ in RNEA produces exactly $g(q)$ — and again,
not as a side product, but as the natural output of the algorithm given that
input.

## Why bother factoring it this way?

You almost never *want* $M$, $C$, $g$ separately. What you want, depending
on the task:

- **Inverse dynamics** ($\tau$ from $q, \dot q, \ddot q$): you want the
  whole thing in one shot. That's what RNEA produces, in $O(n)$ time.
- **Forward dynamics** ($\ddot q$ from $\tau$): solve the equation backwards.
  RNEA gives you $h(q, \dot q) := C\dot q + g$ via a $\ddot q = 0$ call; CRBA
  gives you $M$; then $\ddot q = M^{-1}(\tau - h)$. Or use ABA (chapter 11)
  to do all of it in $O(n)$.
- **Control law design** ($M$, $C$ stability properties): you sometimes need
  the structure abstractly — *e.g.* knowing that $\dot M - 2C$ is
  skew-symmetric is the basis of several passivity-based controllers. Then
  CRBA + a Christoffel-symbol RNEA variant give you what you need.

For everything in this library, RNEA and ABA together cover the dynamics.

## A sanity test you can do right now

`tests/unit/algo/test_rnea.cpp` has the simplest possible RNEA check: a
two-link planar arm, both links unit mass, link COMs at $(0,0,0)$ in each
link's own frame, with gravity in $-y$. With $q = \dot q = \ddot q = 0$ the
"required torque" is exactly the static gravitational moment, and it's
hand-computable:

$$
\tau_1 \;=\; -\sum_i r_i \times (m_i g) \cdot \hat z
\;=\; (1)(9.81) + (2)(9.81) \;=\; 29.43\ \mathrm{Nm}.
$$

The library returns `29.4300000000…` to ~`1e-14`. Run it (`ctest -R
HandComputedGravityCompensation`) and watch the recursion produce the
high-school-physics answer.

> ## Where this lives in the library
>
> | Concept | File / line |
> | ------- | ----------- |
> | $\tau = M\ddot q + h$ in code | [`rnea.hpp`](../../include/tinyspatial/algo/rnea.hpp) |
> | Two-link gravity-compensation example | [`test_rnea.cpp`](../../tests/unit/algo/test_rnea.cpp) `HandComputedGravityCompensation` |
