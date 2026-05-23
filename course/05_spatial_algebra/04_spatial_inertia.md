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

## Why we store it separably

`tinyspatial::SpatialInertia` does **not** store the $6\times6$ matrix. It
stores the three things you'd actually measure on the body — mass, COM,
inertia-about-COM — and constructs the $6\times6$ on demand via `matrix6()`.

The payoff is that SE(3) transforms become cleanly element-wise:

- **mass** is invariant,
- **COM** moves as a point: $c' = T\cdot c = R\,c + t$,
- **inertia about COM** rotates: $\bar I' = R\,\bar I\,R^\top$.

That's three lines of code. The equivalent on the $6\times6$ form is a
congruence $I_s' = X^{-\top} I_s X^{-1}$ — correct, but every multiply
contaminates the symmetry of the result with floating-point noise. After many
transforms the stored matrix slowly stops being symmetric, and you have to
periodically re-symmetrise. The separable form has nothing to lose. The library
test `Se3TransformMatchesCongruence` confirms the two agree.

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
| Spatial inertia | [`inertia.hpp`](../../include/tinyspatial/spatial/inertia.hpp) · `class SpatialInertia` |
| 6×6 form | [`inertia.hpp`](../../include/tinyspatial/spatial/inertia.hpp) · `SpatialInertia::matrix6()` |
| Inertia · motion → force | `operator*(SpatialInertia, Motion)` |
| Frame change | `operator*(SE3, SpatialInertia)` |
| Composite-body addition | `SpatialInertia::operator+` |

Next: [How Featherstone thinks](05_how_featherstone_thinks.md).
