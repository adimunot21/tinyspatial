# Rotation matrices

The most direct way to write a rotation is a **3×3 matrix** $R$ that you
multiply a vector by: $v' = R\,v$. For $R$ to be a genuine rotation it must be

- **orthonormal**: its columns are unit vectors, mutually perpendicular
  ($R^\top R = I$), and
- **special**: $\det R = +1$ (the $+1$, not $-1$, rules out reflections).

The columns have a lovely interpretation: column $i$ of $R$ is *where the
$i$-th basis axis lands* after the rotation. So a rotation matrix is literally a
picture of the rotated coordinate frame, written out as three columns.

## The group SO(3)

The set of all such matrices is called $SO(3)$ — the **S**pecial **O**rthogonal
group in 3 dimensions. "Group" is a precise word; it means rotations come with:

- a way to **combine** them: matrix product $R_1 R_2$ is again a rotation
  (do $R_2$ first, then $R_1$);
- an **identity**: the matrix $I$, "do nothing";
- an **inverse**: $R^{-1} = R^\top$ (transpose!), because $R^\top R = I$.

These three facts — closure, identity, inverse — plus associativity, are the
group axioms. We'll lean on them constantly. (In the library, the test
`So3Test.GroupAxioms` checks all three numerically.)

That $R^{-1} = R^\top$ is worth savouring: undoing a rotation is as cheap as
flipping the matrix across its diagonal. No expensive inversion needed.

## Strengths and weaknesses

**Good:** unambiguous, composes by plain matrix multiply, acts on a vector by
plain matrix–vector multiply. Hardware and Eigen love them.

**Awkward:** nine numbers to store three degrees of freedom — six redundant.
Worse, if you keep multiplying matrices, tiny floating-point errors accumulate
and $R$ slowly stops being orthonormal ("drift"). You'd have to periodically
re-orthonormalise. This is the main reason we *store* rotations as quaternions
(chapter 04) and only convert to a matrix when we need to act on a vector.

## Worked example: rotation about z

A rotation by angle $\theta$ about the $z$-axis is

$$
R_z(\theta) = \begin{bmatrix} \cos\theta & -\sin\theta & 0 \\ \sin\theta & \cos\theta & 0 \\ 0 & 0 & 1 \end{bmatrix}.
$$

Check the column story: the first column $(\cos\theta, \sin\theta, 0)$ is where
the $x$-axis $(1,0,0)$ lands — swung up by $\theta$ in the $xy$-plane. The third
column is $(0,0,1)$: the $z$-axis is the rotation axis, so it doesn't move.

## Where this lives in the library

| Concept | File · symbol |
| ------- | ------------- |
| Get the 3×3 matrix from an `SO3` | [`so3.hpp`](../../include/tinyspatial/liegroup/so3.hpp) · `SO3::matrix()` |
| Inverse via transpose/conjugate | [`so3.hpp`](../../include/tinyspatial/liegroup/so3.hpp) · `SO3::inverse()` |
| Group axioms test | [`tests/unit/liegroup/test_so3.cpp`](../../tests/unit/liegroup/test_so3.cpp) · `GroupAxioms` |

Next: [Axis–angle](03_axis_angle.md).
