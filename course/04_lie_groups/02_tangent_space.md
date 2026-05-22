# The tangent space

Imagine standing at one point on a globe and laying a flat sheet of glass so it
just touches there. That sheet is the **tangent plane**: a flat space attached
to the curved surface at a single point. Directions you could start walking map
to arrows on the glass.

Lie groups have exactly this. At the identity of the group sits a flat vector
space called the **Lie algebra**, and the arrows in it are precisely the
*velocities* a transform can have.

## Angular velocity is a tangent vector

Spin a body. At an instant, its motion is described by an **angular velocity**
$\omega \in \mathbb{R}^3$: direction = axis of spin, length = rate of spin. This
$\omega$ is *not* a rotation — you can't `act()` with it on a point. It's an
arrow in the tangent space at the identity of $SO(3)$. That tangent space is
called $\mathfrak{so}(3)$ (lowercase, "little so-three").

Here's the neat part: $\mathfrak{so}(3)$ is exactly the set of skew-symmetric
matrices $[\omega]_\times$ from chapter 03. So the hat operator `skew()` is the
map from "angular velocity as 3 numbers" to "element of the Lie algebra as a
matrix." The same three numbers we used for axis–angle.

## For SE(3): the twist

The tangent space of $SE(3)$ is $\mathfrak{se}(3)$, and its arrows are
**twists**: a 6-vector

$$
\xi = (\omega,\; v),
$$

stacking an angular velocity $\omega$ and a linear velocity $v$. A twist is the
instantaneous velocity of a rigid body — how it's spinning *and* translating at
once.

> **Convention alert.** tinyspatial stacks twists **angular-first**: $\omega$ in
> slots 0–2, $v$ in slots 3–5. This follows Featherstone. Pinocchio stacks them
> linear-first. Neither is "right" — but mixing them silently is the #1 way to
> get plausible-looking wrong answers, so we state ours loudly and convert at the
> Pinocchio boundary (see chapter 15). Whenever a result looks transposed or
> swapped, suspect ordering first.

## Why a flat space is such a relief

The tangent space is an ordinary vector space: you can add twists, scale them,
take their average, run least-squares on them, feed them to an optimiser — all
the flat-space tools that *don't* work directly on the curved manifold. The
strategy throughout the library is: **lift to the tangent space, do the linear
algebra there, then map back to the manifold.** The map back is `exp`, next
chapter.

## Where this lives in the library

| Concept | File · symbol |
| ------- | ------------- |
| Angular velocity → Lie algebra ($[\omega]_\times$) | [`so3.hpp`](../../include/tinyspatial/liegroup/so3.hpp) · `skew()` |
| Twist 6-vector (angular-first) | [`core/types.hpp`](../../include/tinyspatial/core/types.hpp) · `Vector6` |

Next: [exp and log](03_exp_and_log.md).
