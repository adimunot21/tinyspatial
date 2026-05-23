# Exercises — Chapter 04

These build intuition for the tangent space, `exp`/`log`, and the adjoint. Check
numerically by adapting [`src/examples/se3_basics.cpp`](../../src/examples/se3_basics.cpp)
or by reading the matching tests.

## 1. exp/log round-trip

(a) Take $\omega = (0, 0, \pi/2)$. What rotation is $\exp(\omega)$? Write its
matrix.
(b) Confirm $\log(\exp(\omega)) = \omega$ by reasoning about angle and axis.
(c) Now take $\omega' = (0, 0, 2\pi)$. What is $\exp(\omega')$, and what does
$\log$ of *that* return? Why isn't it $\omega'$?

## 2. The chart breaks at $\pi$

Explain in one or two sentences why recovering the rotation *axis* from a $180°$
rotation is ambiguous, and how storing the rotation as a quaternion (and reading
the angle as $2\arcsin\|q_{\text{vec}}\|$) sidesteps the problem.

## 3. Jacobian near the identity

Without computing anything, what is $J_r(0)$ (the right Jacobian at zero)? Why?
What does that tell you about $\exp(\delta)$ for very small $\delta$?

## 4. Left vs right Jacobian

Using the identity $J_l(\omega) = J_r(\omega)^\top$, and knowing
$J_r(\omega) = I - A[\omega]_\times + B[\omega]_\times^2$, write $J_l(\omega)$.
(Hint: what is $[\omega]_\times^\top$?)

## 5. The adjoint moves a velocity

A frame B is offset from frame A by pure translation $t = (0, 0, 1)$ (no
rotation). A body has angular velocity $\omega = (0,0,1)$ and zero linear
velocity *in B*. Using $\mathrm{Ad}_T$ with $R = I$, what twist does A see?
Interpret the linear part physically.

<details><summary>Hint</summary>
With $R=I$, the adjoint is $\begin{bmatrix} I & 0 \\ [t]_\times & I\end{bmatrix}$.
The new linear part is $[t]_\times\,\omega = t \times \omega$.
</details>

## 6. Self-cross is zero

Show that $\xi \times \xi = 0$ for any twist $\xi$ (i.e. `cross_motion(xi) * xi`
is the zero vector), and explain why this is the spatial analogue of
$v \times v = 0$ for ordinary vectors. (See `CrossTest.SelfCrossIsZero`.)

---

Solutions aren't committed yet — verify against the library and the tests named
in chapters 03–05.
