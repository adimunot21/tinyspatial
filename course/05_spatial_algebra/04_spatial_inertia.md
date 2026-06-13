# Spatial inertia

A rigid body's mass distribution is captured by three things: its **mass** $m$,
its **centre of mass** $c$, and the $3\times3$ **rotational inertia tensor**
$\bar I$ about the centre of mass. Together they say "how hard is it to
accelerate this body, and in what direction?"

In spatial algebra we package the three into a single $6\times6$ **spatial
inertia matrix** $I_s$ that maps a twist to a momentum:

$$
\mathbf{h} \;=\; I_s\,m \quad\text{(momentum 6-vector = spatial inertia · velocity 6-vector)}.
$$

The momentum is a *force-like* 6-vector — same dimensions, transforms with the
dual adjoint. So `inertia * Motion` returns a `Force` in our type system, and
the compiler enforces the dimensional analysis for you.

## The matrix form (angular-first)

Putting it on paper, with $c_\times = [c]_\times$ the skew matrix of the COM:

$$
I_s = \begin{bmatrix} \bar I + m\,(\|c\|^2 I - c\,c^\top) & m\,c_\times \\
                       m\,c_\times^\top                    & m\,I_3 \end{bmatrix}.
$$

The off-diagonal blocks differ by sign because $c_\times^\top = -c_\times$, and
that sign is exactly what keeps the whole matrix symmetric. (The whole *thing*
must be symmetric: $I_s$ is the Hessian of kinetic energy $\tfrac12 m^\top I_s
m$.)

## Why it is stored separably

`SpatialInertiaT` does **not** store the $6\times6$ matrix. It stores the three
measurable quantities and constructs the $6\times6$ on demand:

```cpp
template <typename S>
class SpatialInertiaT {
  // …
 private:
  S mass_;
  Vector3 com_;
  Matrix3 inertia_com_;   // Ī about the centre of mass
};
```

The payoff is that an SE(3) transform acts element-wise on the stored parameters:

```cpp
template <typename S>
[[nodiscard]] SpatialInertiaT<S> operator*(const SE3T<S>& t, const SpatialInertiaT<S>& i) {
  const typename Types<S>::Matrix3 r = t.rotation().matrix();
  return SpatialInertiaT<S>(i.mass(), t.act(i.com()), r * i.inertia_com() * r.transpose());
}
```

The mass is invariant; the COM moves as a point ($c' = R\,c + t$, via `t.act`);
the inertia about COM rotates ($\bar I' = R\,\bar I\,R^\top$). The equivalent on
the $6\times6$ form is the congruence $I_s' = X^{-\top} I_s X^{-1}$ — correct, but
every multiply contaminates the symmetry of the result with floating-point noise,
so a stored matrix must be periodically re-symmetrised. The separable form has no
symmetry to lose. `Se3TransformMatchesCongruence` confirms the two agree.

When the $6\times6$ form is genuinely needed (for a parity comparison, say), it is
assembled from the same parts:

```cpp
[[nodiscard]] Matrix6 matrix6() const {
  const Matrix3 cx = skew(com_);
  Matrix6 i = Matrix6::Zero();
  i.template topLeftCorner<3, 3>() = inertia_origin();      // Ī_O = Ī_com − m·c×·c×
  i.template topRightCorner<3, 3>() = mass_ * cx;
  i.template bottomLeftCorner<3, 3>() = mass_ * cx.transpose();
  i.template bottomRightCorner<3, 3>() = mass_ * Matrix3::Identity();
  return i;
}
```

The off-diagonal blocks are $m\,c_\times$ and $m\,c_\times^\top = -m\,c_\times$ —
the sign relation that makes $I_s$ symmetric, here a consequence of the
construction rather than something to maintain by hand.

## The composite-body trick

When two rigid bodies are *welded together* (a common idealisation in CRBA —
the composite-rigid-body algorithm, chapter 11), their spatial inertias add:

$$
I_{\text{total}} = I_A + I_B
$$

— *in the same frame*. The mass adds, the COM is the mass-weighted average,
and the rotational inertia about the new COM comes from the parallel-axis
theorem applied to each part. `SpatialInertia::operator+` does this for you,
and `CompositeInertiaMatches6x6Sum` checks the linearity in the $6\times6$
representation.

## Where this lives in the library

| Concept | File · symbol |
| ------- | ------------- |
| Spatial inertia | [`inertia.hpp`](../../include/tinyspatial/spatial/inertia.hpp) · `SpatialInertiaT`, `SpatialInertia` |
| 6×6 form | `inertia.hpp` · `SpatialInertiaT::matrix6` |
| Inertia · motion → force | `inertia.hpp` · `operator*(SpatialInertiaT, MotionT)` |
| Frame change | `inertia.hpp` · `operator*(SE3T, SpatialInertiaT)` |
| Composite-body addition | `inertia.hpp` · `SpatialInertiaT::operator+` |

Next: [How Featherstone thinks](05_how_featherstone_thinks.md).
