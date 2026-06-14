# Chapter 01 — C++ features used by the library

This chapter is a reference, not a tutorial. It assumes working C++ and catalogues
the specific C++20 features `tinyspatial` relies on, so that the implementation
chapters read without friction. From Chapter 03 onward the language is the
vehicle, not the subject: the content is robotics, expressed in C++.

For the language itself, the canonical reference is
[cppreference.com](https://en.cppreference.com/w/cpp). For a structured
introduction, [learncpp.com](https://www.learncpp.com) is thorough and free;
[*A Tour of C++*](https://www.stroustrup.com/tour3.html) (Stroustrup) is the
condensed overview, and *Effective Modern C++* (Meyers) covers idiomatic use.

## C++20 features the library uses

- **`std::variant` + `std::visit`.** Joint types are a closed, tagged union
  (`JointRevoluteT`, `JointPrismaticT`, `JointFixedT`, `JointFreeFlyerT`)
  dispatched with `std::visit`. This is the central data-structure decision of the
  model layer; see Chapter 06 and
  [cppreference's `std::visit` page](https://en.cppreference.com/w/cpp/utility/variant/visit).
- **`if constexpr`.** Compile-time branching, used inside the `std::visit` lambdas
  and in the scalar-generic code paths to specialise on the joint or scalar type
  without runtime cost.
- **`concepts`.** Template constraints, applied lightly to the scalar type so that
  the algorithms accept both `double` and the autodiff scalar `Jet<N>` (Chapter 13).
- **`std::expected`.** Returned at the public API for fallible operations such as
  URDF parsing. Exceptions are reserved for programmer errors — precondition
  violations — not for expected runtime failure.

## Two codebase-specific conventions

These two patterns recur throughout the source and are worth internalising before
reading the algorithms:

1. **Eigen reference arguments.** Inputs are passed by
   `const Eigen::Ref<const MatT>&` and outputs by `Eigen::Ref<MatT>`. This accepts
   any Eigen expression of the right shape — a `Map`, a `.segment(...)` view, a
   block — without copying, while making the input/output direction explicit in
   the signature.

2. **`std::expected` over exceptions.** A public API that can fail returns
   `std::expected<T, Error>`. The one documented exception is the iterative IK
   family, which returns a result struct with a `converged` flag, because
   "ran N iterations, here is the best answer and whether it met tolerance" is the
   honest shape of that operation.

## Out of scope

The library does not use, and these chapters do not require: multithreading,
coroutines, custom allocators, networking, regex, or non-trivial stream I/O.

Next: [Chapter 02 — Linear algebra](../02_linear_algebra/README.md).
