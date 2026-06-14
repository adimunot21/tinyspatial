# Chapter 02 — Linear algebra used by the library

As with Chapter 01, this is a reference rather than a tutorial. It lists the
linear-algebra topics the library actually uses and points to the standard free
resources for review.

For a ground-up treatment,
[3Blue1Brown's *Essence of Linear Algebra*](https://www.3blue1brown.com/topics/linear-algebra)
builds geometric intuition, and Strang's *Introduction to Linear Algebra* (with
the free MIT OCW course) provides the formal development; chapters 1–5 cover
everything used here. For depth on the numerical side, Trefethen & Bau,
*Numerical Linear Algebra*.

## What the library uses, by frequency

1. **Matrix-vector and matrix-matrix products.** The substrate of every algorithm.
2. **Transposes**, particularly the identity $(AB)^\top = B^\top A^\top$, which
   recurs in the dual (force) transforms and in derivative proofs.
3. **Inverses** — mostly as a concept rather than a computation: $R^\top = R^{-1}$
   for a rotation, the inverse Plücker transform, the configuration that undoes a
   transform. Explicit inversion is confined to the IK and dynamics solvers.
4. **The cross product** $\mathbf a \times \mathbf b$, in both its vector form and
   its skew-symmetric matrix form $[\mathbf a]_\times \mathbf b$. The equivalence
   is central to spatial algebra and is developed in Chapter 03.
5. **Determinants**, only to the extent that $\det R = +1$ characterises a rotation
   and $\det J = 0$ signals a singularity (Chapter 09).
6. **Eigenvalues**, only for the positive-definiteness check on the mass matrix
   (Chapter 11): a real symmetric matrix has real eigenvalues; symmetric
   positive-definite means all positive.

## Why the matrices stay small

Robotics computation lives in $\mathbb R^3$, $\mathbb R^4$, and $\mathbb R^6$; the
matrices are $3\times3$, $4\times4$, and $6\times6$. They are small enough to write
out, trace entry by entry, and verify by hand. The library exposes this directly:
fixed-size Eigen types (`Matrix3`, `Matrix6`, `Vector6`) carry their dimensions in
the type, so the compiler knows the size and the reader can reason about each
matrix concretely.

## Eigen

The linear-algebra backend is [Eigen 3.4](https://eigen.tuxfamily.org). The subset
the course relies on:

- Construction: `Vector3 v(1.0, 2.0, 3.0);`, `Matrix3 m; m << 1, 0, 0, /*…*/;`
- Coefficient access: `v(0)`, `m(0, 1)`.
- Products: `m * v`, `m.transpose() * m`.
- Blocks and slices: `v.head<3>()`, `m.topRightCorner<3, 3>()`,
  `v.segment(i, n)`.

The [Eigen quick reference](https://eigen.tuxfamily.org/dox/group__QuickRefPage.html)
is the one page worth keeping open.

## Deferred topics

The singular value decomposition becomes relevant for the damped-least-squares IK
in Chapter 12 and is introduced there. Abstract vector spaces, Gram-Schmidt, and
tensor/index notation are not used by the library.

Next: [Chapter 03 — Rotations and transforms](../03_rotations_and_transforms/README.md).
