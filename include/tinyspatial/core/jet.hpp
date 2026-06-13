/// \file jet.hpp
/// \brief Forward-mode automatic-differentiation scalar (a dual number).
///
/// `Jet<N>` carries a value `a` and an `N`-vector of partial derivatives `v`
/// with respect to `N` seeded inputs. Arithmetic and the math functions below
/// propagate `v` by the chain rule, so evaluating any function built from them
/// at a `Jet` yields both the value and its exact gradient — no finite
/// differencing, no tape, no external dependency.
///
/// It is a drop-in Eigen scalar: with the `Eigen::NumTraits` specialisation and
/// the namespaced math functions here, `Eigen::Matrix<Jet<N>, …>` and even
/// `Eigen::Quaternion<Jet<N>>` work. That is what lets the library's algorithms
/// (written against `Scalar = double`) run on `Jet` once they are templated on
/// the scalar — see `core/types.hpp`.
///
/// Forward mode is the right choice here: robot configuration dimension is small
/// (`nv ≲ 30`), and the Jacobians we want are tall (`∂pose/∂q`, `∂τ/∂q`), which
/// forward mode delivers cheaply with `N = nv`.
///
/// Design follows Ceres Solver's `Jet` (BSD-3); kept minimal and self-contained.
#ifndef TINYSPATIAL_CORE_JET_HPP
#define TINYSPATIAL_CORE_JET_HPP

#include <cmath>
#include <limits>

#include <Eigen/Core>

namespace tinyspatial {

/// A dual number with `N` partial-derivative slots.
template <int N>
struct Jet {
  using Partials = Eigen::Matrix<double, N, 1>;

  double a = 0.0;                 ///< value f(x)
  Partials v = Partials::Zero();  ///< gradient ∂f/∂x_k, k = 0..N-1

  Jet() = default;

  /// A constant: value with zero derivative. Non-explicit so Eigen's
  /// `Scalar(literal)` constructions compile.
  Jet(double value) : a(value) {}  // NOLINT(google-explicit-constructor)

  /// A seeded independent variable: value with a unit partial in slot `k`.
  Jet(double value, int k) : a(value) { v[k] = 1.0; }

  /// Full construction from value and gradient.
  Jet(double value, const Partials& deriv) : a(value), v(deriv) {}

  Jet& operator+=(const Jet& y) {
    a += y.a;
    v += y.v;
    return *this;
  }
  Jet& operator-=(const Jet& y) {
    a -= y.a;
    v -= y.v;
    return *this;
  }
  Jet& operator*=(const Jet& y) {
    v = a * y.v + y.a * v;
    a *= y.a;
    return *this;
  }
  Jet& operator/=(const Jet& y) {
    const double inv = 1.0 / y.a;
    a *= inv;
    v = (v - a * y.v) * inv;
    return *this;
  }
  Jet& operator+=(double s) {
    a += s;
    return *this;
  }
  Jet& operator-=(double s) {
    a -= s;
    return *this;
  }
  Jet& operator*=(double s) {
    a *= s;
    v *= s;
    return *this;
  }
  Jet& operator/=(double s) {
    const double inv = 1.0 / s;
    a *= inv;
    v *= inv;
    return *this;
  }
};

// --- Unary ----------------------------------------------------------------

template <int N>
Jet<N> operator+(const Jet<N>& f) {
  return f;
}
template <int N>
Jet<N> operator-(const Jet<N>& f) {
  return Jet<N>(-f.a, -f.v);
}

// --- Jet ⊗ Jet ------------------------------------------------------------

template <int N>
Jet<N> operator+(const Jet<N>& f, const Jet<N>& g) {
  return Jet<N>(f.a + g.a, f.v + g.v);
}
template <int N>
Jet<N> operator-(const Jet<N>& f, const Jet<N>& g) {
  return Jet<N>(f.a - g.a, f.v - g.v);
}
template <int N>
Jet<N> operator*(const Jet<N>& f, const Jet<N>& g) {
  return Jet<N>(f.a * g.a, f.a * g.v + g.a * f.v);
}
template <int N>
Jet<N> operator/(const Jet<N>& f, const Jet<N>& g) {
  // (f/g)' = (f' g - f g') / g^2 = f'/g - (f/g) g'/g
  const double inv = 1.0 / g.a;
  const double q = f.a * inv;
  return Jet<N>(q, (f.v - q * g.v) * inv);
}

// --- Jet ⊗ double (and the reverse) ---------------------------------------

template <int N>
Jet<N> operator+(const Jet<N>& f, double s) {
  return Jet<N>(f.a + s, f.v);
}
template <int N>
Jet<N> operator+(double s, const Jet<N>& f) {
  return Jet<N>(f.a + s, f.v);
}
template <int N>
Jet<N> operator-(const Jet<N>& f, double s) {
  return Jet<N>(f.a - s, f.v);
}
template <int N>
Jet<N> operator-(double s, const Jet<N>& f) {
  return Jet<N>(s - f.a, -f.v);
}
template <int N>
Jet<N> operator*(const Jet<N>& f, double s) {
  return Jet<N>(f.a * s, f.v * s);
}
template <int N>
Jet<N> operator*(double s, const Jet<N>& f) {
  return Jet<N>(f.a * s, f.v * s);
}
template <int N>
Jet<N> operator/(const Jet<N>& f, double s) {
  const double inv = 1.0 / s;
  return Jet<N>(f.a * inv, f.v * inv);
}
template <int N>
Jet<N> operator/(double s, const Jet<N>& g) {
  const double inv = 1.0 / g.a;
  const double q = s * inv;
  return Jet<N>(q, (-q * inv) * g.v);
}

// --- Comparisons (on the value only) --------------------------------------
//
// Branch predicates compare the value, never the whole Jet (CLAUDE.md §15 note
// for the Scalar-generic path: a Taylor/closed-form branch is selected on `.a`).

#define TINYSPATIAL_JET_CMP(op)                        \
  template <int N>                                     \
  bool operator op(const Jet<N>& f, const Jet<N>& g) { \
    return f.a op g.a;                                 \
  }                                                    \
  template <int N>                                     \
  bool operator op(const Jet<N>& f, double s) {        \
    return f.a op s;                                   \
  }                                                    \
  template <int N>                                     \
  bool operator op(double s, const Jet<N>& f) {        \
    return s op f.a;                                   \
  }
TINYSPATIAL_JET_CMP(<)
TINYSPATIAL_JET_CMP(<=)
TINYSPATIAL_JET_CMP(>)
TINYSPATIAL_JET_CMP(>=)
TINYSPATIAL_JET_CMP(==)
TINYSPATIAL_JET_CMP(!=)
#undef TINYSPATIAL_JET_CMP

// --- Math functions (found by ADL; Eigen calls these unqualified) ---------

template <int N>
Jet<N> abs(const Jet<N>& f) {
  return f.a < 0.0 ? -f : f;
}
template <int N>
Jet<N> fabs(const Jet<N>& f) {
  return abs(f);
}
template <int N>
Jet<N> sqrt(const Jet<N>& f) {
  const double t = std::sqrt(f.a);
  return Jet<N>(t, f.v * (0.5 / t));
}
template <int N>
Jet<N> exp(const Jet<N>& f) {
  const double t = std::exp(f.a);
  return Jet<N>(t, f.v * t);
}
template <int N>
Jet<N> log(const Jet<N>& f) {
  return Jet<N>(std::log(f.a), f.v * (1.0 / f.a));
}
template <int N>
Jet<N> sin(const Jet<N>& f) {
  return Jet<N>(std::sin(f.a), f.v * std::cos(f.a));
}
template <int N>
Jet<N> cos(const Jet<N>& f) {
  return Jet<N>(std::cos(f.a), f.v * (-std::sin(f.a)));
}
template <int N>
Jet<N> tan(const Jet<N>& f) {
  const double c = std::cos(f.a);
  return Jet<N>(std::tan(f.a), f.v * (1.0 / (c * c)));
}
template <int N>
Jet<N> asin(const Jet<N>& f) {
  return Jet<N>(std::asin(f.a), f.v * (1.0 / std::sqrt(1.0 - f.a * f.a)));
}
template <int N>
Jet<N> acos(const Jet<N>& f) {
  return Jet<N>(std::acos(f.a), f.v * (-1.0 / std::sqrt(1.0 - f.a * f.a)));
}
template <int N>
Jet<N> atan(const Jet<N>& f) {
  return Jet<N>(std::atan(f.a), f.v * (1.0 / (1.0 + f.a * f.a)));
}
template <int N>
Jet<N> atan2(const Jet<N>& g, const Jet<N>& f) {
  // d atan2(g, f) = (f dg - g df) / (f^2 + g^2)
  const double denom = 1.0 / (f.a * f.a + g.a * g.a);
  return Jet<N>(std::atan2(g.a, f.a), (f.a * g.v - g.a * f.v) * denom);
}
template <int N>
Jet<N> pow(const Jet<N>& f, double p) {
  const double t = std::pow(f.a, p - 1.0);
  return Jet<N>(t * f.a, f.v * (p * t));
}
template <int N>
Jet<N> hypot(const Jet<N>& f, const Jet<N>& g) {
  return sqrt(f * f + g * g);
}
template <int N>
Jet<N> fmin(const Jet<N>& f, const Jet<N>& g) {
  return f.a < g.a ? f : g;
}
template <int N>
Jet<N> fmax(const Jet<N>& f, const Jet<N>& g) {
  return f.a > g.a ? f : g;
}
template <int N>
const Jet<N>& min(const Jet<N>& f, const Jet<N>& g) {
  return f.a < g.a ? f : g;
}
template <int N>
const Jet<N>& max(const Jet<N>& f, const Jet<N>& g) {
  return f.a > g.a ? f : g;
}
template <int N>
Jet<N> floor(const Jet<N>& f) {
  return Jet<N>(std::floor(f.a));  // derivative is a.e. zero
}
template <int N>
Jet<N> ceil(const Jet<N>& f) {
  return Jet<N>(std::ceil(f.a));
}
template <int N>
bool isfinite(const Jet<N>& f) {
  return std::isfinite(f.a) && f.v.allFinite();
}
template <int N>
bool isnan(const Jet<N>& f) {
  return std::isnan(f.a);
}
template <int N>
bool isinf(const Jet<N>& f) {
  return std::isinf(f.a);
}

}  // namespace tinyspatial

// --- Eigen scalar traits --------------------------------------------------

namespace Eigen {

template <int N>
struct NumTraits<tinyspatial::Jet<N>> {
  using Real = tinyspatial::Jet<N>;
  using NonInteger = tinyspatial::Jet<N>;
  using Literal = tinyspatial::Jet<N>;
  using Nested = tinyspatial::Jet<N>;

  static tinyspatial::Jet<N> dummy_precision() {
    return tinyspatial::Jet<N>(NumTraits<double>::dummy_precision());
  }
  static tinyspatial::Jet<N> epsilon() { return tinyspatial::Jet<N>(NumTraits<double>::epsilon()); }
  static tinyspatial::Jet<N> highest() { return tinyspatial::Jet<N>(NumTraits<double>::highest()); }
  static tinyspatial::Jet<N> lowest() { return tinyspatial::Jet<N>(NumTraits<double>::lowest()); }
  static int digits10() { return NumTraits<double>::digits10(); }

  enum {
    IsComplex = 0,
    IsInteger = 0,
    IsSigned = 1,
    ReadCost = 1,
    AddCost = 1 + N,
    MulCost = 1 + 2 * N,
    RequireInitialization = 1,
  };
};

// Mixing Jet with a raw double in Eigen expressions yields a Jet.
template <int N, typename BinOp>
struct ScalarBinaryOpTraits<tinyspatial::Jet<N>, double, BinOp> {
  using ReturnType = tinyspatial::Jet<N>;
};
template <int N, typename BinOp>
struct ScalarBinaryOpTraits<double, tinyspatial::Jet<N>, BinOp> {
  using ReturnType = tinyspatial::Jet<N>;
};

}  // namespace Eigen

#endif  // TINYSPATIAL_CORE_JET_HPP
