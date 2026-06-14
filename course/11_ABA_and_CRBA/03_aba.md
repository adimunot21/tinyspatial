# Articulated bodies (ABA)

ABA is the kind of algorithm that looks impossible the first time you see
it. *Of course* you can compute $\ddot q$ — solve a linear system,
$\ddot q = M^{-1}(\tau - h)$, done. But ABA does it without ever forming
$M$, without ever doing a matrix inversion, in $O(n)$ time. The trick is
a redefinition: the **articulated-body inertia**.

## The redefinition

Consider just one joint $i$ and the subtree below it. From joint $i$'s
point of view, that whole subtree is "the thing that resists when I
accelerate." If the subtree below were *rigidly attached*, it would resist
acceleration with its composite inertia (chapter "CRBA"). But the joints
below are *not* rigid — they're free to move, governed by their own
torques. So the subtree resists less.

That "less" is exactly the **articulated-body inertia** $I^A_i$ — the
effective 6×6 inertia matrix that joint $i$ sees, accounting for the
fact that joints below can themselves move. It's smaller than the
composite inertia by exactly the contribution that gets "absorbed" by the
freedom of joints below.

## The formula

For a joint $i$ with motion subspace $S_i$ (size $6 \times n_{v,i}$),
inputs from its child(ren) give a partial articulated inertia $I'_i$ and
bias force $p'_i$. The joint then "absorbs" some of the inertia,
because joint $i$ is free in the $S_i$ directions:

$$
I^A_i = I'_i - U_i D_i^{-1} U_i^\top, \qquad U_i = I'_i S_i, \qquad D_i = S_i^\top U_i.
$$

The $U_i D_i^{-1} U_i^\top$ piece is a rank-$n_{v,i}$ subtraction —
it says "in the direction joint $i$ is free, the subtree below joint $i$
provides no resistance, because joint $i$ can just move." Substract that
component and you have the inertia the *parent* of $i$ sees.

The bias force gets a similar correction:

$$
p^A_i = p'_i + I^A_i c_i + U_i D_i^{-1} u_i, \qquad u_i = \tau_i - S_i^\top p'_i,
$$

where $c_i = v_i \times (S_i \dot q_i)$ is the velocity-product
acceleration (the same one from RNEA's outward pass).

Once $(I^A_i, p^A_i)$ are known, they are transported to the parent's
frame by the standard 6×6 transforms (motion adjoint for $I^A$,
force-Plücker dual for $p^A$) and added to the parent's accumulators.

After this inward sweep reaches the root, the *root's* articulated
inertia and bias know exactly what the whole tree feels like. Then a
second outward sweep propagates accelerations:

$$
\ddot q_i = D_i^{-1} (u_i - U_i^\top \alpha_{p \to i}), \qquad
\alpha_i = \alpha_{p \to i} + c_i + S_i \ddot q_i,
$$

where $\alpha_{p \to i}$ is the parent's acceleration transported into
joint $i$'s frame. At the root, $\alpha_{p \to i}$ is the gravity-trick
contribution ($-g$ as a spatial acceleration), so gravity enters exactly
once.

## Three passes total

1. **Outward (base → leaves).** Same as RNEA: compute every body's twist
   $v_i$. Also seed $I^A_i := I_i$ (the body's own spatial inertia) and
   $p^A_i := v_i \times^{*} (I_i v_i)$ (the velocity-product bias).
   Record $c_i$ for use in pass 3.

2. **Inward (leaves → base).** For each joint, compute $U_i, D_i, u_i$,
   then *reduce* $I^A$ and $p^A$ by the rank-$n_{v,i}$ subtraction, and
   transport into the parent's frame.

3. **Outward (base → leaves).** Solve for $\ddot q_i$ at each joint, then
   propagate the body's acceleration $\alpha_i$.

In `aba.hpp`, these are three top-level `for` loops, all $O(n)$.

## Why does this work?

Featherstone's derivation is a little dense, but the idea is intuitive
once you see it: the kinematic chain *below* a joint can be replaced, for
the purpose of computing what joint $i$ feels, by a single articulated
"super-rigid-body" with effective inertia $I^A_i$ and bias $p^A_i$. Each
inward step replaces a parent + reduced-subtree by a new articulated body
whose parameters absorb the joint.

So the iteration is: keep reducing the tree to its articulated parent,
one joint at a time. By the time you reach the base, you've collapsed
everything down to a (fictitious) free-floating body whose acceleration
follows from $-g$. Then the outward sweep "unrolls" the reductions in
reverse, dropping each joint's $\ddot q_i$ out as it goes.

It's beautiful, and it's exactly $O(n)$. A 30-DoF humanoid simulator,
done in a fraction of a millisecond.

## The inertia transport

ABA transports articulated inertia between frames via the force
Plücker. If $T = $ `pose_in_parent[i]` (child-to-parent), and we let
$X^{*} = $ `force_plucker(T)` (the force adjoint), then

$$
I^A_{\text{parent}} \mathrel{+}= X^{*} \, I^A_{\text{child-reduced}} \, X^{*\top}.
$$

This is a symmetric congruence — $X^{*\top}$ is the *inverse* motion
adjoint, and the formula $X^{*} I X^{*\top}$ is the spatial-inertia
analog of $R \Sigma R^\top$ for an ordinary rotational change of basis.
It preserves the symmetry and positive-(semi)definiteness of $I^A$ by
construction.

For $p^A$ the transport is the simpler $X^{*} \, p^A_{\text{child-reduced}}$
— a force just transforms with the force adjoint.

```cpp
const Matrix6 x_force = force_plucker(data.pose_in_parent[i]);
i_a[model.parent[i]].noalias() += x_force * ia_articulated * x_force.transpose();
p_a[model.parent[i]].noalias() += x_force * pa_articulated;
```

## The "small D" question

$D_i = S_i^\top I^A_i S_i$ is the *joint-space* articulated inertia at
joint $i$ — the inertia the motor at joint $i$ actually feels. For a
1-DOF joint it's a scalar; for a floating joint it's a 6×6 matrix. The
algorithm needs $D_i^{-1}$, but only in small blocks:

- Revolute / prismatic: $D_i^{-1}$ is one division.
- Floating: $D_i^{-1}$ is the inverse of a 6×6 SPD matrix; Eigen handles
  it with an LLT in a handful of microseconds.

There is *never* an $n \times n$ matrix inversion. That's how you get
$O(n)$.

> ## Where this lives in the library
>
> | Concept | File · symbol |
> | ------- | ------------- |
> | Pass 1 (kinematics + seeding) | [`aba.hpp`](../../include/tinyspatial/algo/aba.hpp) · the first `for (int i = 0; …)` loop |
> | Pass 2 (articulated reduction) | `aba.hpp` · the `for (int i = njoints-1; …)` loop |
> | Pass 3 (acceleration propagation) | `aba.hpp` · the final `for (int i = 0; …)` loop |
