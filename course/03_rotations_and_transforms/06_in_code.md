# In code: `SO3T`

The preceding sections established four representations of a rotation. The library
commits to one for storage — the **unit quaternion** — and derives the others on
demand. This section reads the relevant parts of
[`liegroup/so3.hpp`](../../include/tinyspatial/liegroup/so3.hpp) and explains the
decisions encoded there.

## The class and its storage

```cpp
template <typename S>
class SO3T {
 public:
  using Scalar = S;
  using Vector3 = typename Types<S>::Vector3;
  using Matrix3 = typename Types<S>::Matrix3;
  using Quaternion = typename Types<S>::Quaternion;

  SO3T() : quat_(Quaternion::Identity()) {}
  explicit SO3T(const Quaternion& q) : quat_(canonical(q)) {}
  explicit SO3T(const Eigen::Ref<const Matrix3>& rotation)
      : quat_(canonical(Quaternion(rotation))) {}
  // …
 private:
  Quaternion quat_;
};

using SO3 = SO3T<double>;
```

Three points carry most of the design:

- **A single data member, `Quaternion quat_`.** A rotation is four `double`s, not
  the nine of a $3\times3$ matrix. Composition is a quaternion product (16
  multiplies) rather than a matrix product (27), and — decisively — repeated
  composition does not accumulate the loss of orthonormality that drifts a stored
  matrix off SO(3). The matrix form is produced only when asked for, via
  `matrix()`.

- **The class is templated on the scalar `S`.** `SO3 = SO3T<double>` is the type
  the public API and every algorithm are written against; this course teaches
  against that alias. The template exists so the same code can be instantiated on
  the forward-mode autodiff scalar `Jet<N>`, making rotations differentiable
  (Chapter 13). The two constructors taking external data both route through
  `canonical(...)`.

- **`explicit` on the value constructors.** A `Quaternion` or a `Matrix3` does not
  silently become an `SO3T`; the conversion is a deliberate, named step, because
  it carries a precondition (the matrix must be orthonormal with $\det = +1$) and
  performs normalisation.

## Canonicalisation: the `q = −q` trap, handled once

A unit quaternion and its negation represent the *same* rotation. Left unmanaged,
that ambiguity leaks into every comparison and every derivative. The library
resolves it at the single point where quaternions enter the type:

```cpp
/// Normalise to a unit quaternion with w ≥ 0 (unique representative).
[[nodiscard]] static Quaternion canonical(Quaternion q) {
  q.normalize();
  if (q.w() < Scalar(0)) {
    q.coeffs() *= Scalar(-1);
  }
  return q;
}
```

Every constructor calls `canonical`, so an `SO3T` always holds the unit, $w \ge 0$
representative. This is the same convention Pinocchio uses, which is what lets the
validation suite compare quaternions component-wise without first reconciling
signs. Note the branch tests `q.w() < Scalar(0)` — a comparison on the scalar
*value*; under autodiff the derivative is carried through whichever branch the
value selects.

## Reading a rotation out

```cpp
[[nodiscard]] const Quaternion& quaternion() const { return quat_; }
[[nodiscard]] Matrix3 matrix() const { return quat_.toRotationMatrix(); }
[[nodiscard]] Vector3 act(const Eigen::Ref<const Vector3>& p) const {
  return quat_ * p;
}
```

`quaternion()` returns the stored representative by const reference (no copy).
`matrix()` synthesises the $3\times3$ form when an algorithm needs to multiply it
into a block. `act(p)` rotates a point directly through the quaternion — `quat_ * p`
applies the rotation without ever forming the matrix, which is both faster and free
of matrix round-off for the common "rotate a vector" case.

Composition and inverse follow the group structure exactly:

```cpp
[[nodiscard]] SO3T operator*(const SO3T& rhs) const { return SO3T(quat_ * rhs.quat_); }
[[nodiscard]] SO3T inverse() const { return SO3T(quat_.conjugate()); }
```

The inverse of a rotation is the quaternion conjugate — for a unit quaternion the
conjugate equals the inverse, mirroring $R^{-1} = R^\top$ for matrices, at a
fraction of the cost.

The exponential and logarithm maps — how a rotation vector becomes an `SO3T` and
back — are the subject of Chapter 04, where the same header is read again for its
Lie-group operations.

---

### Where this lives in the library

| Concept | File · symbol |
| ------- | ------------- |
| The rotation type and its storage | [`liegroup/so3.hpp`](../../include/tinyspatial/liegroup/so3.hpp) · `SO3T`, `quat_` |
| Canonicalisation (`w ≥ 0`) | `so3.hpp` · `SO3T::canonical` |
| Quaternion / matrix accessors | `so3.hpp` · `SO3T::quaternion`, `SO3T::matrix` |
| Acting on a point, composition, inverse | `so3.hpp` · `SO3T::act`, `operator*`, `SO3T::inverse` |
| `skew` / `unskew` (hat / vee) | `so3.hpp` · `skew`, `unskew` |
