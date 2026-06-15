# Exercises

---

### 1. Derive the DLS step from the regularised least-squares problem

Show that the minimiser of

$$
\Phi(\delta q) \;=\; \tfrac{1}{2}\|J \delta q - e\|^2 \;+\; \tfrac{1}{2}\lambda^2 \|\delta q\|^2
$$

is exactly $\delta q^* = J^\top (J J^\top + \lambda^2 I)^{-1} e$. Use the
*matrix inversion lemma* (Woodbury identity).

> *Hint:* set $\nabla \Phi = 0$ and solve. Manipulate
> $(J^\top J + \lambda^2 I)^{-1} J^\top$ into the right-pseudoinverse
> form.

---

### 2. Why is the error in body-local frame?

The library defines the IK error as

$$
e \;=\; \log_{\mathrm{SE}(3)}\!\bigl(T_{\mathrm{current}}^{-1} T_{\mathrm{target}}\bigr).
$$

What changes if the *world-frame* error

$$
e' \;=\; \log_{\mathrm{SE}(3)}\!\bigl(T_{\mathrm{target}} T_{\mathrm{current}}^{-1}\bigr)
$$

is used instead? Which Jacobian frame goes with it? Verify the two solvers
produce the same final $q^*$ (they should — the choice of frame is just
a convention) by writing a quick test that solves IK both ways on
`franka_fr3.urdf`.

---

### 3. The DLS-to-Jacobian-transpose limit

In the limit $\lambda \to \infty$, the DLS step approaches a scaled
Jacobian-transpose step:

$$
\delta q_{\mathrm{DLS}} \;\xrightarrow{\lambda \to \infty}\; \lambda^{-2} J^\top e.
$$

Show this algebraically. Then run a small numerical experiment: at a
fixed configuration on `franka_fr3`, compute $\delta q$ for $\lambda \in
\{10^{-3}, 10^{-2}, 10^{-1}, 1, 10\}$ and plot the magnitude. Does the
prediction hold?

---

### 4. Measuring the nullspace dimension

For the 7-DoF `franka_fr3.urdf` at a random configuration, compute the
SVD of $J_{\mathrm{LOCAL}}$ and count the number of singular values
below $10^{-6}$. That count is the dimension of $\mathrm{null}(J)$.
Repeat at a nominally "singular" configuration (e.g. one with two joint
angles set such that consecutive axes align). Does the count change?

---

### 5. Building a random-restart wrapper

Implement the `solve_ik_with_restarts` sketch from sub-chapter 06.
Run it 100 times on random (seed, target) pairs on `franka_fr3` with
0, 1, 3, and 10 restarts. Tabulate the success rate.

Empirically: how many restarts does it take to push the success rate
above 99%?

---

### 6. Posture attraction with joint-limit avoidance

A useful production secondary task is "avoid joint limits." Define
$\Phi_2(q) = \sum_i (q_i - q_{\mathrm{mid},i})^2 / (q_{\mathrm{max},i}
- q_{\mathrm{min},i})^2$ where $q_{\mathrm{mid}}$ is the midpoint of
each joint's range. Its gradient is

$$
\nabla \Phi_2(q)_i \;=\; \frac{2 (q_i - q_{\mathrm{mid},i})}{(q_{\mathrm{max},i} - q_{\mathrm{min},i})^2}.
$$

Modify `solve_ik_nullspace` (or write a new variant) using this gradient
instead of $q_{\mathrm{rest}} - q$. Verify on `franka_fr3` that the
resulting configurations have all joints further from their limits than
plain DLS (assume joint limits of $\pm \pi$).

---

### 7. Bug hunt

Suppose someone changes the inner Jacobian frame:

```cpp
// was: compute_jacobian(model, data, link_id, j, JacobianFrame::kLocal);
compute_jacobian(model, data, link_id, j, JacobianFrame::kWorld);
```

without changing how the error is computed. Will the solver converge?
Trace through the algebra: with body-frame error $e$ and world-frame
Jacobian $J_W$, what does the step
$\delta q = J_W^\top (J_W J_W^\top + \lambda^2 I)^{-1} e$ produce?

> *Hint:* The body-frame and world-frame Jacobians differ by a 6×6
> adjoint matrix on the left. What happens to the formula?
