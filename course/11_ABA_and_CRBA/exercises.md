# Exercises

---

### 1. CRBA on a 2-DOF arm by hand

For `simple_arm.urdf` at $q = 0$:

- Link 1: mass 1, COM at $(0, 0, 0)$ in its own frame, inertia
  $\bar I = \mathrm{diag}(0.01, 0.01, 0.005)$ kg·m².
- Link 2: same; joint 2 is at $(1, 0, 0)$ in link 1's frame.
- Both joints rotate about $z$.

Compute $M(0)$ by hand using the composite-inertia recursion. You should
get a $2 \times 2$ symmetric positive-definite matrix; the $M_{11}$ entry
should be larger than $M_{22}$ (link 1 carries both link masses, link 2
carries only itself).

Verify with `ts.crba(model, np.zeros(2))`.

---

### 2. The forward / inverse round-trip on paper

In chapter 03 we claimed

$$
\mathrm{ABA}(q, \dot q, \mathrm{RNEA}(q, \dot q, a)) \;=\; a
$$

is *structurally* true — not just a numerical coincidence.

Sketch why this must be true, treating $M, h$ as known black-box outputs
of CRBA and RNEA respectively. (You don't need to derive ABA's
formula from scratch — just argue that *if* ABA is correctly solving
forward dynamics, then composing with RNEA gives identity.)

---

### 3. Compare ABA and CRBA+RNEA empirically

Write a Python script that does the following for `franka_fr3.urdf`:

1. Sample 100 random $(q, v, \tau)$.
2. For each, compute $\ddot q_{\mathrm{ABA}} = $ `ts.aba(model, q, v, tau)`.
3. For each, compute $M = $ `ts.crba(model, q)`, $h = $ `ts.rnea(model, q,
   v, zeros)`, and $\ddot q_{\text{long}} = M^{-1}(\tau - h)$.
4. Plot the histogram of $\| \ddot q_{\mathrm{ABA}} - \ddot q_{\text{long}}
   \|_\infty$ across the 100 samples.

What does the spread of the histogram tell you about ABA's numerical
behaviour vs the explicit-inverse formulation?

---

### 4. Articulated inertia of a planar arm

Take a planar 2-DOF arm with both joints revolute. Pick numbers (e.g. unit
masses, unit lengths, $q = 0$). Compute by hand:

1. $I^A_2$ (the articulated inertia at joint 2) — for a leaf joint this
   equals the body's spatial inertia, because there's no subtree below.
2. The reduction $I^A_2 - U_2 D_2^{-1} U_2^\top$.
3. Transport it into joint 1's frame and add joint 1's own inertia: that's
   $I^A_1$ before its own reduction.

You should observe that $I^A_1$ has *less* effective inertia along the
$z$-axis than the composite inertia $I^c_1$ from CRBA. Why?

> *Hint:* the "missing" inertia is exactly what's "absorbed" by joint
> 2's freedom to rotate.

---

### 5. Floating bases

The library supports `JointFloating` but our fixture robots don't use one.
Sketch (don't implement) what changes when the root is a floating joint:

- $S_{\text{root}}$ becomes the $6 \times 6$ identity.
- $D_{\text{root}}$ becomes the $6 \times 6$ articulated inertia of the
  whole robot.
- $D_{\text{root}}^{-1}$ requires inverting that.

What would the gravity trick look like for a floating-base humanoid?
(Hint: nothing changes — the trick is *exactly* designed for this.)

---

### 6. Performance prediction

ABA is $O(n)$. RNEA + CRBA + matrix solve is $O(n^2)$ in the dense case,
$O(n)$ for serial chains. Predict (in floating-point operations per call):

- Approximate FLOP count for ABA on a 7-DoF arm.
- Approximate FLOP count for CRBA + RNEA + dense-LDLT solve on the same.

Then time both on `franka_fr3` using `bench_rnea` as a starting point (you
can add a `bench_aba` and `bench_crba`). How does your back-of-envelope
estimate compare to reality?
