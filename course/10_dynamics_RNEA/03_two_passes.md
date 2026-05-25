# The two passes

The full RNEA is just chapter 02's single-body recursion wrapped around a
kinematic tree. Two sweeps — one outward, one inward — and we're done.

## Pass 1: outward (base → leaves)

For each joint $i$ in topological order, given parent $p$'s body-frame
spatial velocity $v_p$ and acceleration $a_p$:

1. **Transport the parent's twist into body $i$'s frame.** With $T_{ip} = $
   `pose_in_parent[i].inverse()` (parent's frame mapped into body $i$'s),
   $$
   v_{p \to i} = \mathrm{Ad}_{T_{ip}}\,v_p, \qquad
   a_{p \to i} = \mathrm{Ad}_{T_{ip}}\,a_p.
   $$
2. **Add the joint's contribution.** A joint with motion subspace $S_i$
   (chapter 06, 09) and velocity slice $\dot q_i$ adds
   $$
   v_i = v_{p \to i} + S_i \dot q_i, \qquad
   a_i = a_{p \to i} + S_i \ddot q_i + v_i \times (S_i \dot q_i).
   $$
   The $v_i \times (S_i \dot q_i)$ correction is the spatial chain rule —
   the parent's velocity makes $S_i$ itself move, which adds to $a_i$ on
   top of the constant-$S_i$ contribution.
3. **Compute the net wrench on body $i$.**
   $$
   f_i = I_i\,a_i + v_i \times^{*} (I_i\,v_i).
   $$
   This is the spatial Newton–Euler of chapter 02, no surprises.

For the root, the parent is the world. The "world's velocity" is zero, and
its acceleration is the gravity trick — $\mathrm{Ad}_{T_{i0}} \cdot (-g)$ as
a spatial acceleration in body $i$'s frame.

In `rnea.hpp`:

```cpp
if (model.parent[i] == -1) {
  data.v[i] = v_j;
  data.a[i] = (i_from_parent * neg_gravity_world) + a_j + cross(data.v[i], v_j);
} else {
  const Motion v_parent_in_i = i_from_parent * data.v[model.parent[i]];
  const Motion a_parent_in_i = i_from_parent * data.a[model.parent[i]];
  data.v[i] = v_parent_in_i + v_j;
  data.a[i] = a_parent_in_i + a_j + cross(data.v[i], v_j);
}

const Force inertia_a = model.inertia[i] * data.a[i];
const Force inertia_v = model.inertia[i] * data.v[i];
data.f[i] = inertia_a + cross(data.v[i], inertia_v);
```

After this sweep, every body has its $v_i$, $a_i$, and "net force needed to
make it move that way" $f_i$.

## Pass 2: inward (leaves → base)

The net force $f_i$ on body $i$ is the sum of *all* forces acting on it:
the parent pushing it through joint $i$, plus joint $i$'s motor torque
projected back to a wrench, plus whatever its children push back on it.
By Newton's third law, children push back with the opposite of what their
own joints push them with — so we just walk inward, accumulating
contributions.

For joint $i$ in reverse topological order:

1. **Project onto the joint's subspace.** The motor torque needed on joint
   $i$ is
   $$
   \tau_i = S_i^\top f_i.
   $$
   For a revolute joint with axis $a$, this is $a \cdot f_{\text{angular}}$ —
   pick out the angular component along the joint axis. (The full
   per-joint-type dispatch is in `joint_subspace_project()`.)
2. **Push the wrench up to the parent.** Body $i$'s wrench, expressed in
   its parent's frame, adds to the parent's net wrench:
   $$
   f_p \mathrel{+}= X^{*}_{T_{pi}}\,f_i,
   $$
   where $X^{*}_{T_{pi}} = $ `force_plucker(pose_in_parent[i])` is the
   *dual* (force) Plücker transform of the parent-from-body pose. This is
   why the library types `Motion` and `Force` separately — `pose * f`
   automatically uses the dual adjoint.

In `rnea.hpp`:

```cpp
for (int i = model.njoints() - 1; i >= 0; --i) {
  // ...
  detail::joint_subspace_project(j, data.f[i], tau.segment(model.idx_v[i], joint_nv));
  if (model.parent[i] != -1) {
    data.f[model.parent[i]] += data.pose_in_parent[i] * data.f[i];
  }
}
```

That's it.

## What did we compute?

Setting $\dot q = \ddot q = 0$ with non-zero $g$: $\tau = g(q)$.

Setting $\ddot q = 0$ with $\dot q, g$: $\tau = C(q, \dot q) \dot q + g(q)
=: h(q, \dot q)$.

Setting general $\dot q, \ddot q, g$: $\tau = M(q) \ddot q + h(q, \dot q)$
— *with* the inertial term, *without* ever writing $M$ down. That's the
magic.

## A property we will use later

For any $\dot q$, $g = 0$:

$$
\text{column } j \text{ of } M(q) \;=\; \mathrm{RNEA}(q, 0, e_j) \;-\; \mathrm{RNEA}(q, 0, 0)
$$

i.e. RNEA at unit acceleration on each joint, with the gravity-bias
subtracted, *gives you* the columns of $M$. The library's
`test_crba.cpp:ConsistentWithRneaUnitVectors` runs this on every fixture and
checks that CRBA's output agrees to $10^{-10}$. It's a useful sanity check
the first time you have both algorithms.

## When does this fail?

- **Tree not topologically sorted.** Children must appear after parents in
  the index order. The URDF loader guarantees this; if you build a `Model`
  by hand and violate it, the outward pass will read uninitialised data.
- **Inconsistent units.** Gravity in `m/s²` but inertia in `g·cm²`: garbage.
  The library uses SI everywhere; mixing isn't supported.
- **Loops.** The recursion assumes a tree. A closed kinematic loop (e.g. a
  parallel mechanism) needs constraint forces; that's not implemented here.

> ## Where this lives in the library
>
> | Concept | File / line |
> | ------- | ----------- |
> | Outward pass | [`rnea.hpp:99-122`](../../include/tinyspatial/algo/rnea.hpp#L99-L122) |
> | Inward pass | [`rnea.hpp:125-134`](../../include/tinyspatial/algo/rnea.hpp#L125-L134) |
> | Subspace project (per joint type) | [`rnea.hpp:63-81`](../../include/tinyspatial/algo/rnea.hpp#L63-L81) |
