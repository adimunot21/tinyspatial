# Geometric vs analytical

There are two different objects called "the Jacobian" in robotics literature,
and they are not the same. Distinguishing the two avoids a
class of "the rotation is upside down" bugs.

## Geometric Jacobian (what we compute)

The **geometric Jacobian** $J_g$ maps joint velocities to a **spatial
velocity** — a 6-vector twist where the angular part is the angular velocity
$\omega$ (an axis-angle rate, not a derivative of any particular parameter):

$$
v = \begin{bmatrix} \omega \\ v_{\text{lin}} \end{bmatrix} \;=\; J_g(q)\,\dot q.
$$

This is the natural object on the SE(3) manifold. Its columns are screw
axes. It composes cleanly with the adjoint under frame changes. It is what
the rest of this library uses.

## Analytical Jacobian (what we *don't* compute)

The **analytical Jacobian** $J_a$ maps joint velocities to the derivative of
some specific *minimal* parameterisation of the link pose — e.g. the
derivative of roll-pitch-yaw angles $(\dot\phi, \dot\theta, \dot\psi)$,
followed by the linear velocity $(\dot x, \dot y, \dot z)$:

$$
\begin{bmatrix} \dot \phi \\ \dot \theta \\ \dot \psi \\ \dot x \\ \dot y \\ \dot z \end{bmatrix}
\;=\; J_a(q)\,\dot q.
$$

This looks tempting because the rows are derivatives of *real numbers*
(plottable angles), not abstract twist components. But:

- $J_a$ has **singularities of its own**, separate from the physical
  singularities of the robot. RPY angles famously hit gimbal lock at
  $\theta = \pm\pi/2$. IK then fails not because the arm is stuck, but
  because the *parameterisation* is.
- Different choices of minimal parameterisation give different $J_a$'s;
  there's no canonical one.
- Composing $J_a$'s under frame changes requires non-trivial transformations
  that the geometric version avoids.

The geometric form has none of these problems because the angular part of a
twist isn't a derivative of any angle — it's a derivative on the rotation
*group*, which is smooth everywhere.

## When $J_a$ is wanted

For visualising quantities along a chosen parameterisation, or for plugging into
a controller that explicitly accepts RPY-velocity feedback. In both cases
$J_g$ converts to $J_a$ by left-multiplication with a rotation
parameterisation Jacobian. This library does not ship that conversion;
the IK chapter explains why it is almost never
needed.

## A footnote on "analytic"

Some texts use "analytical Jacobian" to mean "the closed-form expression of
the geometric Jacobian, as opposed to a finite-difference approximation."
That's a *different* use of the word, and it has nothing to do with
parameterisation. In this library, "analytical" is the closed form, and
"geometric vs analytical" is the parameterisation distinction — a genuine
source of confusion. The course uses "twist Jacobian" or "RPY Jacobian"
where an unambiguous term is required.

## Where this lives in the library

| Concept | File · symbol |
| ------- | ------------- |
| Geometric Jacobian | [`jacobian.hpp`](../../include/tinyspatial/algo/jacobian.hpp) · `compute_jacobian()` |
| FD validation of the geometric form | [`test_jacobian.cpp`](../../tests/unit/algo/test_jacobian.cpp) |

Next: [Reference frames](03_reference_frames.md).
