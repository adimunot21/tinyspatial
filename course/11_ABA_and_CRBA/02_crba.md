# Composite rigid bodies (CRBA)

CRBA computes the joint-space inertia matrix $M(q)$ directly. The
core idea is one sentence:

> *The inertia your motor at joint $i$ has to fight is the composite inertia
> of every body below it, transported into joint $i$'s frame, projected
> onto joint $i$'s motion subspace.*

That's $M_{ii}$. Off-diagonal entries are the analogous "this is the
inertia that joint $j$ has to push when joint $i$ accelerates" cross-terms.

## The composite inertia

Define, for each joint $i$,

$$
I^c_i \;:=\; \text{sum of (transported) spatial inertias of every body in the subtree rooted at } i,
$$

all expressed in body $i$'s own frame. Then for a serial chain (no
branching) with only revolute / prismatic joints, $M_{ii} = S_i^\top I^c_i
S_i$. Off-diagonal entries follow a similar form with appropriate
transports — see section "filling $M$" below.

Composite inertias are easy to compute recursively, leaves-to-root:

```
ic[i] := I[i]              # start with the body itself
for i from N-1 down to 1:
    ic[parent[i]] += pose_in_parent[i] · ic[i]   # transport child into parent's frame, add
```

The transport `T · I` of a spatial inertia is the SE(3) action defined in
chapter 05. The library stores `SpatialInertia` in *separable* form (mass,
COM, inertia about COM), so the transport is exact: no numerical drift
from repeatedly rotating a 6×6 matrix.

In `crba.hpp`:

```cpp
std::vector<SpatialInertia> ic = model.inertia;
for (int i = njoints - 1; i > 0; --i) {
  const int p = model.parent[i];
  if (p >= 0) {
    ic[p] = ic[p] + (data.pose_in_parent[i] * ic[i]);
  }
}
```

After this loop, `ic[i]` is the spatial inertia of the entire subtree
rooted at $i$.

## Filling $M$

For each joint $i$:

1. **Motion subspace.** $S_i$ is a $6 \times n_{v,i}$ matrix; column $k$ is
   "the twist produced by unit velocity along DOF $k$ of joint $i$."
   For a revolute joint about axis $\hat a$, $S_i$ is the column $[\hat a;
   0]$ (angular-first). For prismatic, $[0; \hat a]$. For floating,
   $S_i$ is just the identity.

2. **Diagonal block.** The "force on body $i$ if I accelerate at unit
   velocity along DOF $k$ of joint $i$, holding everything below
   rigidly" is $F = I^c_i \cdot S_i$, and the torque projected back
   onto $S_i$ is

   $$
   M_{ii} = S_i^\top F = S_i^\top I^c_i S_i.
   $$

3. **Off-diagonal blocks.** Walk up the parent chain. At each ancestor
   $j$, transport $F$ into $j$'s frame using the **force** Plücker
   transform, then project onto $S_j$:

   $$
   F_j = X^{*}_{T_{j(c)}} \cdot F_{c}, \qquad
   M_{ji} = S_j^\top F_j.
   $$

   By symmetry, $M_{ij} = M_{ji}^\top$.

In `crba.hpp`:

```cpp
Eigen::Matrix<Scalar, 6, Eigen::Dynamic> f = ic[i].matrix6() * s_i;
m_out.block(model.idx_v[i], model.idx_v[i], nv_i, nv_i) = s_i.transpose() * f;
int child = i;
int j = model.parent[i];
while (j >= 0) {
  f = force_plucker(data.pose_in_parent[child]) * f;
  const auto s_j = detail::joint_motion_subspace(model.joints[j]);
  // ... fill block M[j, i] and its mirror M[i, j] ...
  child = j;
  j = model.parent[j];
}
```

The cost is $O(n_{v,i} \cdot \mathrm{depth}(i))$ per joint $i$; summed
over the tree this gives $O(n^2)$ for a chain and slightly better for a
balanced tree.

## Why $M(q)$ is symmetric positive definite

Two free properties we can confirm in code:

- **Symmetry.** The CRBA recursion produces $M$ entries via
  $S_j^\top F$ where $F$ depends on $i$, and the mirroring step
  $M_{ij} = M_{ji}^\top$ is exact. So symmetry is *structural*, not just
  numerical.

- **Positive definiteness.** Kinetic energy is
  $T = \tfrac{1}{2} \dot q^\top M \dot q$, and any motion of a non-zero-mass
  system has $T > 0$. So $M$ has no zero-energy direction:
  $\dot q^\top M \dot q > 0$ for $\dot q \neq 0$, which is the definition
  of positive definite.

`test_crba.cpp:MassMatrixIsSymmetricAndPSD` checks both on every fixture
at five random configurations. The symmetry holds to $10^{-12}$ and
$\lambda_{\min}(M) > 10^{-9}$ — both well-conditioned.

## What if a joint is fixed?

Fixed joints contribute $S_i = $ a $6 \times 0$ matrix. The diagonal block
is empty (`nv_i = 0`), and the up-walk skips writing any off-diagonal
entries through this joint. But composite-inertia transport still happens
— a welded link adds to its parent's composite inertia.

> ## Where this lives in the library
>
> | Concept | File / line |
> | ------- | ----------- |
> | Composite-inertia recursion | [`crba.hpp:91-98`](../../include/tinyspatial/algo/crba.hpp#L91-L98) |
> | Diagonal block | [`crba.hpp:104-116`](../../include/tinyspatial/algo/crba.hpp#L104-L116) |
> | Off-diagonal walk | [`crba.hpp:118-139`](../../include/tinyspatial/algo/crba.hpp#L118-L139) |
> | Motion subspace dispatch | [`crba.hpp:50-76`](../../include/tinyspatial/algo/crba.hpp#L50-L76) |
