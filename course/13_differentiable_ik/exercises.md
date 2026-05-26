# Exercises

---

### 1. The implicit theorem on the unit circle

Starting from $F(x, y) = x^2 + y^2 - 1 = 0$, compute $\partial x / \partial y$
by the implicit theorem and confirm it agrees with directly differentiating
$x = \pm \sqrt{1 - y^2}$.

At what value of $y$ does the implicit theorem fail? What does the
direct-differentiation approach say there?

---

### 2. Derive $\partial r / \partial q$ carefully

In sub-chapter 03 we claimed

$$
\frac{\partial r}{\partial q}\bigg|_{(q^*, T^*)} \;=\; -J_L(q^*).
$$

This came from a first-order expansion that dropped Baker-Campbell-Hausdorff
corrections (because $r = 0$ at $q^*$). Verify that the BCH-correction
term is indeed *proportional to* $r$, so at $q^*$ where $r = 0$ it
vanishes. Reference: BCH formula on $\mathrm{SE}(3)$, chapter 04.

---

### 3. World-frame derivative numerically

In sub-chapter 03 we stated that the world-frame derivative is

$$
\frac{\partial q^*}{\partial T^*}\bigg|_{\mathrm{world}} \;=\; \frac{\partial q^*}{\partial T^*}\bigg|_{\mathrm{body}} \cdot \mathrm{Ad}_{T^{*-1}}.
$$

Write a quick Python script (or pen-and-paper) that does:

1. Compute the body-frame derivative.
2. Compute the world-frame derivative by post-multiplying.
3. Compute a "manual" world-frame FD by perturbing the target via
   $T^* \to \exp(\eta) \cdot T^*$ and re-running IK.

Verify (1) + (2) agrees with (3) to FD precision.

---

### 4. Singularity behaviour

At a singular configuration (smallest singular value of $J_L$ near zero),
the damped pseudoinverse blows up much less than the exact pseudoinverse
would. Run the implicit derivative on `franka_fr3` at a deliberately
singular configuration (try `q = [0, 0, 0, π/2, 0, 0, 0]` — two
consecutive axes align) with `damping = 1e-5` and `damping = 1e-1`.

How does the norm of the derivative differ? Which one would you use in
a production setting?

---

### 5. Gradient of a quadratic loss through IK

Define a loss $\mathcal{L}(q) = \tfrac{1}{2} \|q - q_{\mathrm{rest}}\|^2$
where $q_{\mathrm{rest}}$ is your favourite "comfortable" configuration.

Use the chain rule (sub-chapter 01) to compute $\nabla_{T^*} \mathcal{L}$
at the IK solution for some target. Then verify against FD by perturbing
the target along each of 6 axes, re-running IK, and computing the
finite-difference gradient of $\mathcal{L}$ w.r.t. the target.

How tight is the agreement? At what FD step size $\epsilon$ is it best?

---

### 6. Going beyond IK

Find another place in the library where you might want a derivative
through a fixed-point computation. Hint: think about
[`include/tinyspatial/algo/aba.hpp`](../../include/tinyspatial/algo/aba.hpp)
— forward dynamics also has an implicit definition ($M(q) \ddot q =
\tau - h$). Sketch what `∂ddq/∂tau` would look like via the implicit
theorem.

> *Bonus:* Pinocchio implements exactly this in
> `algorithm/aba-derivatives.hxx`. Worth comparing notes.
