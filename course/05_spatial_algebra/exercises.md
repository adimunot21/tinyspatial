# Exercises — Chapter 05

Work the small calculations with pen and paper, then check against the library.

## 1. Adjoint blocks by hand

A rigid transform has $R = I$ (no rotation) and $t = (0, 0, 1)$ (one metre
along $z$).

(a) Write the $6\times6$ motion adjoint $\mathrm{Ad}_T$ explicitly.
(b) A body has twist $m = (0, 0, 1;\, 0, 0, 0)$ (pure spin about $z$, no
linear velocity at its origin). Apply $\mathrm{Ad}_T$ and interpret the linear
part of the result.

<details><summary>Hint</summary>
The $[t]_\times R$ block is $[t]_\times$. With $t = (0,0,1)$, this is
$\begin{bmatrix}0 & -1 & 0 \\ 1 & 0 & 0 \\ 0 & 0 & 0\end{bmatrix}$.
</details>

## 2. Force does *not* transform with $\mathrm{Ad}_T$

Take the same $T$ as above. Apply the *motion* adjoint to a pure linear force
$f = (0,0,0;\, 1, 0, 0)$ instead of the *force* adjoint. Show that the result
gives a wrong moment, and explain physically why.

(Then check using `force_plucker(t)` that the correct result has zero moment.)

## 3. Point mass, by hand

A point mass $m = 2$ kg sits at $c = (0, 1, 0)$ in the body frame. There is no
rotational inertia about the point itself ($\bar I = 0$).

(a) Compute $\bar I_O$, the rotational inertia about the body-frame origin.
(b) Write the $6\times6$ spatial inertia matrix.
(c) Verify with `SpatialInertia(2.0, Vector3(0,1,0), Matrix3::Zero()).matrix6()`.

## 4. Composite COM

Two equal point masses sit at $(1, 0, 0)$ and $(-1, 0, 0)$.

(a) Where is the composite centre of mass?
(b) Where is the composite rotational inertia about the new COM? Use the
parallel-axis theorem.
(c) Compare with `SpatialInertia(...) + SpatialInertia(...)`.

## 5. The power identity

Pick any nonzero `Motion m` and `Force f`, and any `SE3 t`. Compute
$f \cdot m$ in the body frame and
$(\mathrm{Ad}_T^{-\top}\,f) \cdot (\mathrm{Ad}_T\,m)$ after transporting to
another frame. Confirm they're equal. (The `DualityPower` test does exactly
this; reproduce the calculation with pen and paper for one configuration.)

## 6. Type safety in your head

Predict whether each of these compiles. (Then try them in C++ to confirm.)

```cpp
Motion v;  Force f;
auto a = v + v;        // (a)
auto b = v + f;        // (b)
auto c = cross(v, v);  // (c)  — what type is `c`?
auto d = cross(v, f);  // (d)  — what type is `d`?
```

(Answers: (a) compiles, type `Motion`; (b) compile error — no `Motion + Force`;
(c) `Motion`; (d) `Force`.)

---

Solutions aren't committed yet. The relevant tests
(`test_motion_force.cpp`, `test_inertia.cpp`) are excellent sanity checks.
