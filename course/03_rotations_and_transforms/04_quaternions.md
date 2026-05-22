# Quaternions

A **unit quaternion** is a four-number representation of a rotation:

$$
q = (w,\; x,\; y,\; z), \qquad w^2 + x^2 + y^2 + z^2 = 1.
$$

It looks mysterious but comes straight from axis–angle. For a rotation of angle
$\theta$ about unit axis $\hat{u}$:

$$
q = \Big(\cos\tfrac{\theta}{2},\; \sin\tfrac{\theta}{2}\,\hat{u}\Big).
$$

So the scalar part $w$ encodes the angle, the vector part $(x,y,z)$ encodes the
axis (scaled). Notice the **half-angle**: a full $360°$ turn gives
$\cos 180° = -1$, not $+1$. Hold that thought.

## Why we store rotations as quaternions

`SO3` in tinyspatial keeps a quaternion as its single member. Three reasons:

1. **Compact:** four numbers, only one redundant (the unit-norm constraint),
   versus nine for a matrix.
2. **Drift is trivial to fix:** keeping a quaternion valid means re-normalising
   to unit length — one `sqrt` and a divide. Re-orthonormalising a $3\times3$
   matrix is far messier.
3. **Composition is cheap:** the quaternion product implements rotation
   composition in 16 multiplies, fewer than a $3\times3$ matrix product.

We convert to a matrix only when we need to rotate a vector or hand data to an
algorithm that wants a matrix.

## The sign trap: $q$ and $-q$ are the same rotation

Because of the half-angle, $q$ and $-q$ describe the **identical** rotation
(rotating by $\theta$ about $\hat u$ equals rotating by $-\theta$ about
$-\hat u$). This double-cover is a real footgun: two different-looking
quaternions, same physical orientation. If you compare quaternions naively, or
interpolate without care, you get sign-flip glitches.

**tinyspatial's rule (matching Pinocchio):** always store the representative with
$w \ge 0$. Our constructor normalises and flips the sign if needed, so every
rotation has *one* quaternion. See `SO3::canonical()`. Whenever you wonder "why
is there a `if (w < 0) negate` in here," this is why.

## Acting on a vector

To rotate a vector $v$ with a quaternion you sandwich it: $v' = q\,v\,q^{-1}$
(treating $v$ as a quaternion with zero scalar part). Eigen does this for us
behind `quat * v`, which is what `SO3::act()` calls.

## Where this lives in the library

| Concept | File · symbol |
| ------- | ------------- |
| Quaternion storage | [`so3.hpp`](../../include/tinyspatial/liegroup/so3.hpp) · `SO3::quat_` |
| The $w \ge 0$ canonical form | [`so3.hpp`](../../include/tinyspatial/liegroup/so3.hpp) · `SO3::canonical()` |
| Rotate a vector ($q\,v\,q^{-1}$) | [`so3.hpp`](../../include/tinyspatial/liegroup/so3.hpp) · `SO3::act()` |
| `Quaternion` alias | [`core/types.hpp`](../../include/tinyspatial/core/types.hpp) |

Next: [Rigid transforms](05_rigid_transforms.md).
