# Why six-vectors?

You could describe a rigid body's velocity with two separate 3-vectors: angular
velocity $\omega$ and linear velocity $v$. Many textbooks do exactly that. So
why bother packing them into a single 6-vector?

## The reason: they transform *together*

Watch what happens when you change frames. Suppose the body has angular velocity
$\omega_B$ and linear velocity $v_B$ measured at the body-frame origin, and you
want to express the same physical motion at a point offset by $t$ (still in the
parent's orientation $R = I$ for now). The new linear velocity is

$$
v_A = v_B + \omega_B \times (-t) = v_B + t \times \omega_B,
$$

— the offset point sees an *extra* linear velocity because the body is also
*rotating*, and the rotation drags the offset point sideways. Linear velocity
alone doesn't transform cleanly; it mixes with angular velocity. That mixing is
unavoidable, and it's exactly what the SE(3) **adjoint** captures in one matrix
multiply.

In angular-first ordering, the adjoint is the $6\times6$

$$
\mathrm{Ad}_T \;=\; \begin{bmatrix} R & 0 \\ [t]_\times R & R \end{bmatrix}.
$$

The $[t]_\times R$ block is the angular→linear coupling above. Once you write
velocities as 6-vectors, transforming them is one matrix–vector multiply. Try
keeping $\omega$ and $v$ separate and the same operation needs explicit cross
products everywhere. Multiply that bookkeeping across a 7-link arm and you see
why Featherstone reaches for 6-vectors.

## The same trick works for forces, accelerations, momenta, …

Every per-link kinematic and dynamic quantity in our story is a 6-vector:

- **Velocity** (a *twist*) — angular + linear.
- **Acceleration** — angular + linear, transforms like velocity except for a
  velocity-product correction.
- **Force** (a *wrench*) — moment + linear-force.
- **Momentum** — angular + linear, related to velocity by inertia.

Six-vectors are the common currency. Operations like "transport this to the
parent frame," "add these together," "compute the kinetic energy" all become
straight-line linear algebra on a unified data type.

## The cost

The price is one ordering convention to remember (angular-first or
linear-first), and one footgun: motion-like and force-like 6-vectors are not the
same thing. They transform under *different* matrices — the adjoint vs its dual.
We use the type system to keep them straight: `Motion` and `Force` are distinct
C++ types (next chapter), so the compiler catches the most common spatial-algebra
bug before you even reach the tests.

## Where this lives in the library

| Concept | File |
| ------- | ---- |
| The 6-vector alias | [`core/types.hpp`](../../include/tinyspatial/core/types.hpp) · `Vector6` |
| The angular→linear coupling block | [`liegroup/se3.hpp`](../../include/tinyspatial/liegroup/se3.hpp) · `SE3::adjoint()` |

Next: [Twists and wrenches](02_twists_and_wrenches.md).
