# The manifold idea

Recall the unit quaternion: four numbers with the constraint $w^2+x^2+y^2+z^2=1$.
That constraint says rotations don't fill up all of 4-D space — they live on the
*surface* of a sphere in 4-D. A surface is curved. That curvature is the whole
story of this chapter.

## What "manifold" means

A **manifold** is a space that looks flat if you zoom in close enough, even
though it's curved overall. The Earth's surface is the classic example: locally
it looks like a flat map, but globally it wraps around and you can't flatten it
without distortion.

The set of rotations $SO(3)$ is a 3-dimensional curved manifold. The set of
rigid transforms $SE(3)$ is a 6-dimensional one. "3-dimensional" is the precise
meaning of "a rotation has three degrees of freedom" — you need three numbers to
say where you are *on* the manifold, even though you might *store* four (a
quaternion) or nine (a matrix).

## Why you can't just use three angles

A tempting shortcut is Euler angles: roll, pitch, yaw — three numbers, done. The
trouble is that no choice of three angles covers the whole rotation manifold
smoothly. There's always a configuration ("gimbal lock") where two of your
angles control the same motion and you lose a degree of freedom, and where small
real motions cause huge jumps in the angles. It's the same reason every flat map
of the Earth distorts something: you cannot wrap a flat sheet around a curved
thing without tearing or stretching.

So we don't try. We store rotations on the manifold (as quaternions) and do our
*calculus* in a flat space attached to it — the tangent space, next chapter.

## "Group" + "manifold" = "Lie group"

$SO(3)$ and $SE(3)$ are special: they're both **groups** (chapter 03 — you can
compose, invert, there's an identity) *and* smooth **manifolds**, and the two
structures are compatible (composing and inverting are smooth operations). A
space with both is a **Lie group** (after Sophus Lie). That marriage is what lets
us do something remarkable: capture an entire curved group of transformations
using only the flat tangent space at one point plus a single map (`exp`) that
wraps the flat space onto the curve.

## The payoff, previewed

- A **velocity** of a rotating body is *not* a rotation; it's an arrow in the
  tangent space (chapter 02).
- To *integrate* a velocity into an orientation, you wrap it onto the manifold
  with `exp` (chapter 03).
- To *differentiate* — needed for optimisation and IK — you use the group's
  Jacobians (chapter 04).

## Where this lives in the library

| Concept | File · symbol |
| ------- | ------------- |
| A point on the SO(3) manifold | [`so3.hpp`](../../include/tinyspatial/liegroup/so3.hpp) · `class SO3` |
| A point on the SE(3) manifold | [`se3.hpp`](../../include/tinyspatial/liegroup/se3.hpp) · `class SE3` |

Next: [The tangent space](02_tangent_space.md).
