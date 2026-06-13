# Jacobians

On a flat space, $\exp(a+b) = \exp(a)\exp(b)$. On a curved Lie group this fails,
because the group does not commute. A perturbation $\delta$ of the tangent vector
therefore does not produce a manifold change of simply $\exp(\delta)$; it is
distorted by the curvature. The **Jacobian** of the exponential map is the linear
operator that captures that distortion.

## What the Jacobian does

Concretely, for a small perturbation $\delta$ of the tangent vector $\omega$:

$$
\exp(\omega + \delta) \approx \exp(\omega)\,\exp\!\big(J_r(\omega)\,\delta\big).
$$

$J_r(\omega)$, the **right Jacobian**, is the linear map that tells you how a
wiggle in the tangent space *at the origin* shows up as a wiggle *at the point
$\exp(\omega)$*, measured on the right. There's a matching **left Jacobian**
$J_l$ for wiggles measured on the left:

$$
\exp(\omega + \delta) \approx \exp\!\big(J_l(\omega)\,\delta\big)\,\exp(\omega).
$$

They're simply related: $J_l(\omega) = J_r(\omega)^\top = J_r(-\omega)$.

## Where the Jacobian appears

Any differentiation *through* a rotation or transform invokes the Jacobian:
manifold optimisation (Gauss–Newton for IK, Chapter 12) needs the derivative of
the error with respect to a tangent step; covariance propagation through a
transform uses it; and the *inverse* Jacobian converts a manifold error back into
a tangent correction — the core operation of on-manifold optimisation. Near the
identity the Jacobian is $I$ (an infinitesimal step is undistorted); the curvature
corrections grow with the angle.

## In code: the closed forms

The SO(3) Jacobians share a single coefficient pair and differ only in the sign
of the linear term, exactly as the algebra predicts ($J_l = J_r^\top$, and
$S^\top = -S$):

```cpp
[[nodiscard]] static Matrix3 left_jacobian(const Eigen::Ref<const Vector3>& omega) {
  const Matrix3 s = skew(omega);
  Scalar a(0), b(0);
  ab_coeffs(omega.squaredNorm(), a, b);
  // Jl(ω) = Jr(ω)ᵀ; since Sᵀ = −S this flips the sign of the S term only.
  return Matrix3::Identity() + a * s + b * (s * s);
}

[[nodiscard]] static Matrix3 right_jacobian(const Eigen::Ref<const Vector3>& omega) {
  const Matrix3 s = skew(omega);
  Scalar a(0), b(0);
  ab_coeffs(omega.squaredNorm(), a, b);
  return Matrix3::Identity() - a * s + b * (s * s);
}
```

with $J = I \pm A\,[\omega]_\times + B\,[\omega]_\times^2$ and the coefficients
$A = (1-\cos\theta)/\theta^2$, $B = (\theta-\sin\theta)/\theta^3$. As with `exp`,
`ab_coeffs` carries Taylor fallbacks below `kSmallAngle` (there $A \to 1/2$,
$B \to 1/6$); without them, every Jacobian near the identity would be `NaN`. The
single source of $A$ and $B$ for both Jacobians is deliberate — it keeps the left
and right forms provably consistent.

## How the formulas are validated

A wrong sign on a $\theta^2$ term can hide for a long time, so the closed forms
are not trusted on derivation alone. The library **finite-differences** the
defining relation — perturb each component of $\omega$, measure the actual change
on the manifold, and compare to $J_r(\omega)$ to $10^{-7}$
(`So3Test.JacobiansMatchFiniteDifference` and its SE(3) sibling). The inverses are
checked by $J\,J^{-1} = I$. This "analytical formula, validated against finite
differences" pattern is how each piece earns trust before the library is ever
compared to Pinocchio.

## SE(3): the coupling block

The $SE(3)$ Jacobian is $6\times6$ and block-structured. The diagonal blocks are
the $SO(3)$ Jacobian; the off-diagonal block is Barfoot's **Q matrix**, which
captures how a rotational perturbation drags the translation. In
[`se3.hpp`](../../include/tinyspatial/liegroup/se3.hpp) it assembles as:

```cpp
[[nodiscard]] static Matrix6 left_jacobian(const Eigen::Ref<const Vector6>& xi) {
  const Vector3 omega = xi.template head<3>();
  const Vector3 v = xi.template tail<3>();
  const Matrix3 jl = SO3::left_jacobian(omega);
  Matrix6 j = Matrix6::Zero();
  j.template topLeftCorner<3, 3>() = jl;
  j.template bottomRightCorner<3, 3>() = jl;
  j.template bottomLeftCorner<3, 3>() = left_jacobian_q(omega, v);
  return j;
}
```

The private `left_jacobian_q` is the intricate part — a three-coefficient
expansion in $[\omega]_\times$ and $[v]_\times$ (Barfoot, *State Estimation for
Robotics*, eq. 7.86), reordered for the angular-first convention. Its complexity
is exactly why the finite-difference test on the full $6\times6$ map is
non-negotiable.

## Where this lives in the library

| Concept | File · symbol |
| ------- | ------------- |
| SO(3) right/left Jacobians + inverses | [`so3.hpp`](../../include/tinyspatial/liegroup/so3.hpp) · `right_jacobian`, `left_jacobian`, `*_inverse` |
| SE(3) Jacobians (with Barfoot Q) | [`se3.hpp`](../../include/tinyspatial/liegroup/se3.hpp) · `left_jacobian`, `left_jacobian_q` |
| Finite-difference validation | [`test_so3.cpp`](../../tests/unit/liegroup/test_so3.cpp), [`test_se3.cpp`](../../tests/unit/liegroup/test_se3.cpp) |

Next: [The adjoint](05_the_adjoint.md).
