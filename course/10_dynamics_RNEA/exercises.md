# Exercises

These exercises stress the *intuition* behind RNEA rather than testing
your C++. If you can do them on paper, you have understood the algorithm.

---

### 1. Gravity-only torques on a 2-DOF arm

Take `simple_arm.urdf`. With $q = 0$, $\dot q = 0$, $\ddot q = 0$ and
gravity $g = (0, -9.81, 0)$ m/s², hand-compute $\tau_1$ and $\tau_2$.

> *Hint:* both links are unit mass, with link 1's COM at $(0,0,0)$ in
> its own frame (joint 1 origin is at world origin) and link 2's COM at
> $(0,0,0)$ in its frame (joint 2 origin is at $(1,0,0)$ in world).
> Z-axis joints. The torques are the gravitational moments about each
> joint axis.

Verify with `ctest -R HandComputedGravityCompensation`.

---

### 2. Why the velocity-product term has the form it does

In the outward pass:

```
a[i] = a_parent_in_i + a_j + cross(data.v[i], v_j)
```

The `cross(v[i], v_j)` correction looks mysterious. Show that it is just
the spatial chain rule:

$$
\frac{d}{dt} \big( \mathrm{Ad}_{T_{p \to i}(t)} \, v_p \big)
\bigg|_{\text{moving frame contribution}}
= v_i \times (S_i \dot q_i).
$$

> *Hint:* $\mathrm{Ad}_T$ depends on $q$ through $T_{p \to i}$. Differentiate
> along the body's motion, use $\frac{d}{dt} \mathrm{Ad}_{\exp(\xi t)} = \mathrm{ad}_\xi$
> from chapter 04.

---

### 3. The CRBA → RNEA identity

In chapter 03 we claimed:

$$
\text{column } j \text{ of } M(q) = \mathrm{RNEA}(q, 0, e_j; g=0) - \mathrm{RNEA}(q, 0, 0; g=0)
$$

(both calls with gravity off). Write a Python script using the tinyspatial
binding that computes a full $M$ matrix this way for `franka_fr3.urdf` at
$q = $ a random configuration, then compares it to `ts.crba(model, q)`.
Confirm agreement to $10^{-12}$.

---

### 4. RNEA on a single floating body

Take a `JointFloating` joint with mass-1, identity inertia. Pick
$v = (0.1, 0, 0; \,0, 0, 0)$ (angular about $x$, no linear). Pick
$\ddot q = 0$, $g = 0$.

By hand, compute the wrench RNEA returns. (Hint: $v \times^{*} (I v) = $ a
gyroscopic torque; for $v_{\text{linear}} = 0$ this is $\omega \times I \omega$
which is zero for symmetric $I$.) Then run it in code.

What changes if $I$ is *not* symmetric — say $I = \mathrm{diag}(1, 2, 3)$?

---

### 5. The cost of the gravity trick

Imagine you wanted gravity *off* on just one body (say, you're modeling a
helium-balloon-supported link). Sketch how you'd modify the outward pass to
allow per-body gravity overrides. Would this require throwing away the
gravity trick? Why or why not?

---

### 6. A bug-hunt

Suppose someone swapped the order of the two arguments to `cross` in the
outward pass:

```cpp
data.a[i] = a_parent_in_i + a_j + cross(v_j, data.v[i]);  // bug: swapped
```

Will `NoGravityNoMotionGivesZeroTorque` still pass? Will
`HandComputedGravityCompensation`? Will the Pinocchio cross-check at
random configurations? Use the antisymmetry of the spatial cross to
predict; then make the change locally and run the tests to confirm.
