# Chapter 02 — Linear algebra

Same approach as chapter 01: we don't want to rewrite the world's
existing excellent linear-algebra resources. This chapter tells you
which ones to use, which topics to focus on for robotics, and what
counts as "enough" to start reading the actual robotics chapters.

> **The bar for this course is lower than you might fear.** You don't
> need to be fluent in eigendecompositions or tensor analysis. You need
> to be comfortable with: vectors, matrices, matrix multiplication, the
> dot product, the cross product, transposes, inverses, and rotations.
> If those sound familiar, you're past the bar.

## If you're starting from zero

Watch all of **[3Blue1Brown's *Essence of Linear Algebra*](https://www.3blue1brown.com/topics/linear-algebra)**
on YouTube. About 4 hours total. It's the single best introduction in
existence and it's free.

After that, work through **Strang's *Introduction to Linear Algebra*** —
the MIT OCW course based on it is free and excellent. You don't need
the whole book; chapters 1–5 are more than enough for everything in this
course.

By the time you can do these without notes, you're ready for chapter 03:

- Multiply two $3 \times 3$ matrices by hand.
- Compute the cross product $\mathbf a \times \mathbf b$ from
  components.
- Solve a $3 \times 3$ linear system in your head for a structured case
  (diagonal, triangular).
- Explain what "$A$ is symmetric" means and why $A^\top A$ always is.

## What you'll keep using in this library

A short list, ordered by frequency:

1. **Matrix-vector and matrix-matrix products.** Every algorithm is built
   on these.
2. **Transposes** ($A^\top$). Especially the identity $(AB)^\top = B^\top
   A^\top$ — surprising, useful, and appears in proofs.
3. **Inverses.** We rarely *compute* a full inverse, but the *concept*
   shows up constantly: a Plücker transform's "inverse Plücker," a
   rotation's $R^\top = R^{-1}$, a configuration "undoing" itself.
4. **The cross product** $\mathbf a \times \mathbf b$, written either as
   a vector or as the skew-symmetric matrix $[\mathbf a]_\times \mathbf
   b$. Both forms come up; chapter 03 introduces the equivalence.
5. **Determinants — only barely.** You need to know that $\det R = +1$
   for a rotation, and that $\det = 0$ means singular (chapter 09's
   topic). You will not be computing $4 \times 4$ determinants by hand.
6. **Eigenvalues — even more barely.** They show up once, when we
   numerically check that the mass matrix is positive definite (chapter
   11). Just remember: real symmetric matrix → real eigenvalues; SPD →
   all positive.

## What you don't need to know yet

Some staples of an undergrad linear algebra course don't show up much in
this library:

- **Vector spaces and bases at the abstract level.** Useful for theory;
  not necessary to read this codebase. We always work in concrete
  $\mathbb R^n$.
- **Gram-Schmidt orthogonalisation.** Conceptually interesting; only
  appears in the validation harness, marginally.
- **Singular value decomposition.** Will become relevant for chapter 12
  (inverse kinematics) — you can pick it up then.
- **Tensor analysis / index notation / Einstein summation.** Not used
  here.

## Why robotics is "easy" linear algebra

Most of robotics lives in $\mathbb R^3$, $\mathbb R^4$, $\mathbb R^6$.
The matrices involved are usually $3 \times 3$, $4 \times 4$, $6 \times
6$. These are small enough that you can:

- Write them out by hand on paper.
- Trace where each entry came from.
- Multiply them and verify the result entry-by-entry.

That's a huge pedagogical advantage. The library leans on it: we use
`Eigen::Matrix3d`, `Eigen::Matrix6d`, *etc.* with their fixed sizes
visible right in the type. When you see `Matrix6` in the code, you know
it's a $6 \times 6$ matrix of `double`, full stop, and you can think
about it as a thing you could compute by hand if you needed to.

## A library-specific note: Eigen

We use **[Eigen](https://eigen.tuxfamily.org)** as our linear-algebra
backend. You don't need to learn its full surface area — only enough to:

- Construct a vector / matrix: `Vector3 v(1.0, 2.0, 3.0);`,
  `Matrix3 m; m << 1, 0, 0, 0, 1, 0, 0, 0, 1;`
- Read a coefficient: `v(0)` or `v[0]`, `m(0, 1)`.
- Multiply: `m * v`, `m.transpose() * m`.
- Slice: `v.head<3>()`, `m.topRightCorner<3, 3>()`.

The official **[Eigen quickref](https://eigen.tuxfamily.org/dox/group__QuickRefPage.html)**
is the one page worth printing out.

## Where to go for more depth

When you want to *understand* (not just use) the linear algebra:

- **Strang's *Linear Algebra and Its Applications*** — same author as
  above, but slightly more advanced. Read it after the intro.
- **Boyd & Vandenberghe, *Introduction to Applied Linear Algebra***
  (free PDF). Engineering-flavoured; complements Strang.
- **Trefethen & Bau, *Numerical Linear Algebra*** — for when "the matrix
  exists" turns into "how do I compute with it accurately?"

## Next

When you're comfortable with the basics:
[Chapter 03 — Rotations and transforms](../03_rotations_and_transforms/README.md).
