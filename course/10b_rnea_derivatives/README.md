# Chapter 10b — Analytical derivatives of RNEA (advanced)

> **Advanced material.** This chapter assumes RNEA (Chapter 10) and derives how
> the library computes the analytical partials `∂τ/∂q`, `∂τ/∂v`, `∂τ/∂a` in
> O(N²) time. The function itself — `rnea_derivatives(model, data, q, v, a, …)`,
> writing three `nv × nv` matrices — can be used without reading the derivation.

The partials matter in three settings:

- **Gradient-based trajectory optimisation.** Methods like DDP, iLQR,
  and SLQ need `∂τ/∂q` and `∂τ/∂v` at every step of a rollout. Computing
  them by finite difference costs `O(n · N)` per step (one extra RNEA call
  per partial); the analytical version costs the equivalent of `O(N)`
  RNEA calls in total — a 10–30× speedup on a 7-DoF arm.
- **Sensitivity analysis.** "If joint $k$'s position is off by $\delta q_k$,
  how much does my motor torque at joint $j$ change?" — that's
  $\partial \tau_j / \partial q_k$, a single entry of the matrix.
- **Model learning.** Auto-differentiable physics simulators (Brax, MuJoCo
  XLA) need exactly these partials to back-propagate through dynamics.

## The reference

The algorithm we implement is from:

> J. Carpentier & N. Mansard, **"Analytical Derivatives of Rigid Body
> Dynamics Algorithms"**, *Robotics: Science and Systems*, 2018.
> [PDF on hal.science](https://hal.science/hal-01790971/document).

It's a short, dense paper that's worth reading directly. Most of what
follows here is a translation into our body-fixed angular-first
conventions, with the algebraic identities spelled out.

## The structure of the algorithm

We piggy-back on RNEA's two passes. During the **outward** sweep, alongside
each body's `v[i]` and `a[i]`, we also accumulate four 6×nv matrices:

| Quantity | Meaning |
| -------- | ------- |
| `dv_dq[i]` | `∂v[i] / ∂q`, body-frame |
| `dv_dv[i]` | `∂v[i] / ∂v` — *equals the per-joint Jacobian* $J_i$ |
| `da_dq[i]` | `∂a[i] / ∂q` |
| `da_dv[i]` | `∂a[i] / ∂v` |

(We don't store `da_da[i]` explicitly — it equals `dv_dv[i] = J_i` because
`v[i]` is `a`-independent. That fact is *also* why `∂τ/∂a ≡ M(q)`, which
the tests check against CRBA.)

The **inward** pass turns these into per-body wrench derivatives `df_d*[i]`
and finally projects onto each joint's motion subspace to get
`∂τ/∂*` row blocks.

## Two identities make it O(N²)

Without these, the inner work at each body would be O(nv²) — one matrix
multiplication per column — and the total cost would be O(N³).

### Identity 1: cross-motion antisymmetry

For any motion 6-vectors $m_1, m_2$:

$$
m_1 \times m_2 \;=\; - m_2 \times m_1
$$

(proven in chapter 05, exercise 2). In matrix form, with
`cross_motion(m)` being the 6×6 matrix that does $m \times \cdot$:

$$
\mathrm{cross\_motion}(x) \cdot y \;=\; -\mathrm{cross\_motion}(y) \cdot x.
$$

This turns the per-column accumulation

$$
\mathrm{cross\_motion}(dv\_dq[i]\,col\,k) \cdot (S_i v_{slice,i})
$$

(which would require one cross-motion-of-a-different-vector per column) into
a single matrix product:

$$
-\mathrm{cross\_motion}(S_i v_{slice,i}) \cdot dv\_dq[i].
$$

One 6×6 matrix multiplied by a 6×nv matrix — a single BLAS-3 call per
joint per derivative input. That is why the algorithm is O(N²) and not
O(N³). In [`rnea_derivatives.hpp`](../../include/tinyspatial/diff/rnea_derivatives.hpp)
the entire correction, applied to every column of both matrices at once, is two
lines:

```cpp
const Matrix6 m_sv = cross_motion(v_j.vector());
da_dq[i].noalias() -= m_sv * dv_dq[i];
da_dv[i].noalias() -= m_sv * dv_dv[i];
```

`m_sv` is $\mathrm{cross\_motion}(S_i v_{\mathrm{slice}\,i})$, built once per body;
the `.noalias()` tells Eigen the product has no aliasing so it can write straight
into the destination without a temporary. Done column-by-column instead, this
block would be the O(N³) inner loop.

### Identity 2: force-cross-motion duality

For a fixed force 6-vector $f = (\tau; v_f)$, the map $m \mapsto m \times^*
f$ is linear in $m$ and equals $\Phi(f) \cdot m$ with

$$
\Phi(f) \;=\; \begin{bmatrix} -[\tau]_\times & -[v_f]_\times \\ -[v_f]_\times & 0 \end{bmatrix}.
$$

(Exercise 1 below derives this from the cross-force formula in chapter 05.) The
implementation is a private helper:

```cpp
/// `Phi(f)` such that `Phi(f) · m == cross_force(m) · f` for any motion m.
/// For `f = (τ; v_f)`: `Phi = [[-[τ]_×, -[v_f]_×]; [-[v_f]_×, 0]]`.
[[nodiscard]] inline Matrix6 force_cross_motion_phi(const Eigen::Ref<const Vector6>& f) {
  const Matrix3 tau_x = skew(f.head<3>());
  const Matrix3 vf_x = skew(f.tail<3>());
  Matrix6 phi = Matrix6::Zero();
  phi.topLeftCorner<3, 3>() = -tau_x;
  phi.topRightCorner<3, 3>() = -vf_x;
  phi.bottomLeftCorner<3, 3>() = -vf_x;
  return phi;
}
```

This converts the velocity-coupling term `cross_force(v[i]) · I · v[i]` into a
matrix that is built once per body and then multiplied by `dv_dq[i]` for all
columns at once. The backward pass uses it directly:

```cpp
const Vector6 iv = i_mat * data.v[i].vector();
const Matrix6 phi_iv = detail::force_cross_motion_phi(iv);
const Matrix6 cf_v = cross_force(data.v[i].vector());
const Matrix6 vel_coupling = phi_iv + cf_v * i_mat;
df_dq[i].noalias() = i_mat * da_dq[i] + vel_coupling * dv_dq[i];
```

## The forward-pass recurrences

In body-frame, angular-first, for a 1-DOF joint with axis $\hat a$
(revolute) or $\hat a$ in linear (prismatic), and motion subspace $S_i \in
\mathbb{R}^{6 \times n_{v,i}}$:

For each joint $i$ in topological order, with parent $p$, child-to-parent
transform $T_{ip}$, motion adjoint $X_{pi} := \mathrm{Ad}_{T_{ip}^{-1}}$
(parent→body):

**Inherit ancestor columns from parent:**

$$
dv\_dq[i] = X_{pi}\,dv\_dq[p], \qquad dv\_dv[i] = X_{pi}\,dv\_dv[p], \qquad \text{(same for da)}.
$$

(For joint $i$'s own slice columns, parent's matrix has zeros, so this
leaves them untouched until the next step.)

**Overwrite joint $i$'s own slice columns** with the closed-form formulas:

$$
\frac{\partial v[i]}{\partial q_{\mathrm{slice}\,i, k}} = -[S_{i,k}]_\times \cdot v_{\mathrm{parent\to i}}
$$

$$
\frac{\partial v[i]}{\partial v_{\mathrm{slice}\,i, k}} = S_{i,k}
$$

$$
\frac{\partial a[i]}{\partial q_{\mathrm{slice}\,i, k}} = -[S_{i,k}]_\times \cdot a_{\mathrm{parent\to i}}
$$

$$
\frac{\partial a[i]}{\partial v_{\mathrm{slice}\,i, k}} = [v[i]]_\times \cdot S_{i,k}.
$$

The first three come from differentiating $X_{pi}(q)$ through the joint's
exponential map; the fourth comes from the velocity-cross term in $a[i]$.

The gravity trick keeps the formula uniform: the root's `a_parent_in_i` is
$X_{p_{\mathrm{world}\to i}} \cdot (-g)$, so the same `-[S]_× · a_parent`
expression carries gravity-dependence into the root's `∂a/∂q` partials
automatically.

**Universal cross-coupling correction.** Add the term coming from
$\mathrm{cross\_motion}(v[i]) \cdot S_i v_{\mathrm{slice}\,i}$ in the
chain rule, applied uniformly to *every* column via Identity 1:

$$
da\_dq[i] \mathrel{-}= \mathrm{cross\_motion}(S_i v_{\mathrm{slice}\,i}) \cdot dv\_dq[i],
$$

$$
da\_dv[i] \mathrel{-}= \mathrm{cross\_motion}(S_i v_{\mathrm{slice}\,i}) \cdot dv\_dv[i].
$$

For columns *in joint $i$'s slice*, this completes the formula. For
ancestor columns, this is the entire derivative formula (the inheritance
step already added the $X_{pi} \cdot$ parent term).

## The backward-pass recurrences

The per-body wrench is `f[i] = I_i · a[i] + cross_force(v[i]) · I_i · v[i]`.
Differentiating column-by-column with Identity 2:

$$
df\_dq[i] = I_i \cdot da\_dq[i] + \big[\Phi(I_i v[i]) + \mathrm{cross\_force}(v[i]) \cdot I_i\big] \cdot dv\_dq[i],
$$

and analogously for `df_dv`. For `df_da`, since `∂a[i]/∂a = dv_dv[i] = J_i`:

$$
df\_da[i] = I_i \cdot J_i.
$$

Then in reverse topological order:

1. **Project to row block.** $\partial \tau_{\mathrm{slice}\,i}/\partial * = S_i^\top \cdot df\_d*[i]$.

2. **Transport to parent.** Same dual Plücker as RNEA's inward pass:
   $df\_d*[p] \mathrel{+}= Y_{ip} \cdot df\_d*[i]$ where $Y_{ip} = \mathrm{force\_plucker}(T_{ip})$.

3. **Add the dual-Plücker derivative term at joint $i$'s own columns.**
   The transport $Y_{ip}$ itself depends on $q_{\mathrm{slice}\,i}$:
   $\partial Y_{ip}/\partial q_{\mathrm{slice}\,i, k} = Y_{ip} \cdot \mathrm{cross\_force}(S_{i,k})$.
   So:

   $$
   df\_dq[p].col(\mathrm{idx\_v}_i + k) \mathrel{+}= Y_{ip} \cdot \mathrm{cross\_force}(S_{i,k}) \cdot f[i].
   $$

That's the whole algorithm.

## The triangulation

Three independent algorithms all agree at machine precision:

- **RNEA**: forward computation of $\tau$.
- **CRBA**: forward computation of $M(q)$.
- **rnea_derivatives** (this chapter): forward computation of all three
  partials including $\partial \tau / \partial a$.

The identity $\partial \tau / \partial a \equiv M(q)$ holds *structurally*,
not just numerically — but the fact that two completely different
implementations land on the same matrix to 1e-10 is the strongest possible
internal check.

The fourth independent check is Pinocchio. The full parity table in
[`docs/PINOCCHIO_PARITY.md`](../../docs/PINOCCHIO_PARITY.md) shows
agreement at ~1e-13 against `pin.computeRNEADerivatives` across all four
fixtures.

## Where to read further

- **The Carpentier-Mansard paper.** It's six pages, dense but not opaque.
  The notation differs from ours (spatial-frame vs body-frame) but the
  recurrences are recognisable side by side.
- **Pinocchio's source.** `src/algorithm/rnea-derivatives.hxx` in the
  Pinocchio repo. Their implementation works in the spatial (world-aligned)
  frame and is more performance-tuned; ours is body-fixed and prioritises
  readability.
- **DDP / iLQR papers.** The use-case end of the pipeline. Tassa et al.
  (2014) is a good entry point.

## Exercises

### 1. Prove Identity 2

Given `cross_force(m) · f` for $m = (\omega, v_m)$ and $f = (\tau, v_f)$,
expand the matrix product and rearrange as a linear function of $m$ with
the matrix $\Phi(f)$ acting from the left. Verify the block form
$\begin{bmatrix} -[\tau]_\times & -[v_f]_\times \\ -[v_f]_\times & 0 \end{bmatrix}$.

### 2. Why is ∂τ/∂a symmetric?

Show that the algorithm above structurally produces a symmetric
$\partial \tau / \partial a$ matrix (not just numerically). *Hint*: trace
where the contribution to $\partial \tau_j / \partial a_k$ comes from
when $j$ is an ancestor of $k$, and compare with the path from $k$ to $j$.

### 3. Code-reading exercise

In `include/tinyspatial/diff/rnea_derivatives.hpp`, find the line that
implements the "universal cross-coupling correction" using Identity 1.
What would the algorithm's complexity be if you instead did the
correction column-by-column?

> ## Where this lives in the library
>
> | Concept | File · symbol |
> | ------- | ------------- |
> | Function signature | [`rnea_derivatives.hpp`](../../include/tinyspatial/diff/rnea_derivatives.hpp) · `rnea_derivatives` |
> | Identity 2 (`Phi`) | `rnea_derivatives.hpp` · `detail::force_cross_motion_phi` |
> | Forward-pass recurrences | `rnea_derivatives.hpp` · the `for (int i = 0; …)` outward loop |
> | Identity 1 use-site | `rnea_derivatives.hpp` · the `m_sv * dv_d*` subtractions |
> | Backward pass | `rnea_derivatives.hpp` · the `for (int i = njoints - 1; …)` loop |
> | Unit tests | [`tests/unit/diff/test_rnea_derivatives.cpp`](../../tests/unit/diff/test_rnea_derivatives.cpp) |
> | Pinocchio parity | [`docs/PINOCCHIO_PARITY.md`](../../docs/PINOCCHIO_PARITY.md) |
