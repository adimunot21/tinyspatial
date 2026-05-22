# Exercises — Chapter 03

Work these with pen and paper first, then check with the library. To experiment
in C++, copy the pattern in
[`src/examples/se3_basics.cpp`](../../src/examples/se3_basics.cpp).

## 1. Non-commutativity by hand

Let $R_z$ be $+90°$ about $z$ and $R_x$ be $+90°$ about $x$.

(a) Compute $R_z R_x$ and $R_x R_z$ as $3\times3$ matrices.
(b) Apply each to the point $(1, 0, 0)$. Confirm the two orders give different
results.

<details><summary>Hint</summary>
Write out $R_z(90°)$ and $R_x(90°)$ from the worked example in
[chapter 02](02_rotation_matrices.md), then multiply.
</details>

## 2. Columns tell a story

A rotation matrix has first column $(0, 1, 0)$ and third column $(0, 0, 1)$.

(a) What is the second column? (Use orthonormality and $\det = +1$.)
(b) Which axis is this a rotation about, and by what angle?

## 3. Axis–angle ↔ quaternion

A rotation is $120°$ about the axis $\hat u = \tfrac{1}{\sqrt3}(1,1,1)$.

(a) Write the rotation vector $\omega$.
(b) Write the unit quaternion $q$.
(c) Verify $\|q\| = 1$.

## 4. The sign trap

You compute a quaternion and get $q = (-0.5,\, 0.5,\, 0.5,\, 0.5)$.

(a) Is this in tinyspatial's canonical form? If not, give the canonical
representative.
(b) What rotation (axis and angle) does it represent?

## 5. Compose two transforms

Let $T_a$ rotate $+90°$ about $z$ with translation $(1,0,0)$, and $T_b$ have no
rotation with translation $(0, 1, 0)$.

(a) Compute $T_a T_b$ by hand using the composition formula.
(b) Where does the origin $(0,0,0)$ end up under $T_a T_b$?
(c) Check against the library by adapting `se3_basics.cpp`.

## 6. Inverse, carefully

For $T_a$ from exercise 5, compute $T_a^{-1}$. Verify $T_a T_a^{-1} = I$ by
showing both the rotation and translation parts come out to the identity. (This
is exactly what `Se3Test.GroupAxioms` checks numerically.)

---

Solutions are intentionally not committed here yet — try the library to check
yourself. If you get stuck, open a discussion on the repository.
