# Rigid transforms

A rotation re-orients things about a fixed point. But a robot link is also
*displaced* from its neighbour. The full description of "where and how oriented"
is a **rigid transform**: a rotation $R$ together with a translation $t$.

Applied to a point $p$:

$$
T(p) = R\,p + t.
$$

Rotate, then shift. The set of all such transforms is the group $SE(3)$ — the
**S**pecial **E**uclidean group in 3-D — and just like $SO(3)$ it has
composition, an identity, and inverses.

## The homogeneous matrix

A tidy way to write a rigid transform as a single matrix is the $4\times4$
**homogeneous** form:

$$
T = \begin{bmatrix} R & t \\ 0\;0\;0 & 1 \end{bmatrix}.
$$

Tack a $1$ onto a point, $\tilde p = (p, 1)$, and then $T\tilde p = (Rp + t, 1)$
— the rotate-then-shift falls out of plain matrix multiplication. The bottom row
$(0,0,0,1)$ is what makes composition work: multiplying two such matrices gives
another one of the same shape. This is `SE3::matrix()`.

## Composition and inverse

Composing transforms follows from the homogeneous product:

$$
T_a T_b = \begin{bmatrix} R_a R_b & R_a t_b + t_a \\ 0 & 1 \end{bmatrix}.
$$

In words: the rotations multiply, and the second translation gets *rotated by*
$R_a$ before being added. This is easy to get subtly wrong by hand, which is why
we test it (`Se3Test.GroupAxioms`). The inverse is

$$
T^{-1} = \begin{bmatrix} R^\top & -R^\top t \\ 0 & 1 \end{bmatrix},
$$

— note you can't just negate $t$; you must rotate it back too.

## Storage choice

We *could* store the $4\times4$ matrix. Instead `SE3` stores an `SO3` (the
quaternion) and a `Vector3`, for the same drift/compactness reasons as before,
and builds the matrix on demand. The `se3_basics` example
([`src/examples/se3_basics.cpp`](../../src/examples/se3_basics.cpp)) composes two
transforms and prints the result — run it and match it against the formula above.

## Looking ahead

Rigid transforms are the joints-and-links glue of the whole library: each joint
carries an $SE(3)$ placing one link relative to its parent (chapter 06), and
forward kinematics is just composing a chain of them (chapter 08). The
*velocity* version of $SE(3)$ — twists — is chapter 05's spatial algebra.

## Where this lives in the library

| Concept | File · symbol |
| ------- | ------------- |
| Rigid transform type | [`se3.hpp`](../../include/tinyspatial/liegroup/se3.hpp) · `class SE3` |
| `act(p) = R p + t` | [`se3.hpp`](../../include/tinyspatial/liegroup/se3.hpp) · `SE3::act()` |
| 4×4 homogeneous matrix | [`se3.hpp`](../../include/tinyspatial/liegroup/se3.hpp) · `SE3::matrix()` |
| Composition / inverse | [`se3.hpp`](../../include/tinyspatial/liegroup/se3.hpp) · `operator*`, `inverse()` |
| Worked example | [`se3_basics.cpp`](../../src/examples/se3_basics.cpp) |

Next: the [exercises](exercises.md), then [chapter 04 — Lie groups](../04_lie_groups/README.md).
