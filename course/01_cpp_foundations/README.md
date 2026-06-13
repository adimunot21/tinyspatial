# Chapter 01 — C++ foundations

This chapter is short on purpose. There already exist excellent, free,
patiently-paced introductions to C++, and rewriting them here would be a
waste of your time and ours. What this chapter *does* do is tell you
which resources to use, which parts to focus on for this codebase, and
what minimum subset will let you start reading `tinyspatial`.

> **You do not need to be a C++ expert to read this library.** The
> chapters from 03 onward read more like a math book than a C++ book; the
> language is the *vehicle*, not the subject. What you need is enough C++
> to follow a function signature and trace a loop.

## If you've never written a line of code

Go through **[learncpp.com](https://www.learncpp.com)** chapters 1–6.
That's enough to:

- Read a function signature.
- Trace a `for` loop.
- Understand what a class is.

Don't try to memorise it. Run the examples, change one line, see what
breaks. Come back to this course when you can read this without blinking:

```cpp
double dot(const Vector3& a, const Vector3& b) {
  double sum = 0.0;
  for (int i = 0; i < 3; ++i) {
    sum += a(i) * b(i);
  }
  return sum;
}
```

(If you can: you're ready.)

## If you know an older C++

The library uses **C++20**, which has a few features you may not have
seen:

- **`concepts`** (constraints on template parameters). Mostly avoidable;
  we use them lightly.
- **`std::variant`** + **`std::visit`** for tagged unions. We use this
  heavily for the joint types (chapter 06). Read
  [cppreference's `std::visit` page](https://en.cppreference.com/w/cpp/utility/variant/visit).
- **`if constexpr`** for compile-time branching. Always appears with
  `std::visit` in this codebase.
- **`std::expected`** (C++23, with a header polyfill). Used at the public
  API for fallible operations like URDF parsing.

The cppreference site is the canonical reference. Bookmark
[cppreference.com](https://en.cppreference.com/w/cpp).

## If you know modern C++ from another project

You can probably skim this chapter and move on. The two things specific
to this codebase that might surprise you:

1. **Eigen idioms.** We pass matrices by `const Eigen::Ref<const MatT>&`
   for inputs and `Eigen::Ref<MatT>` for outputs. This accepts any Eigen
   expression with the right shape (a slice, a column block, a `Map`)
   without copying.

2. **`std::expected` instead of exceptions.** Public APIs that can fail
   return `std::expected<T, Error>`. Exceptions are reserved for
   programmer errors (precondition violations).

## What you don't need to know

Plenty of C++ has nothing to do with this library. You can safely skip,
for now:

- Multi-threading (`std::thread`, `std::mutex`, `std::atomic`).
- Coroutines.
- Streams (`std::cout` is fine; we don't do fancy I/O).
- Networking, regex, the filesystem library (other than minor uses).
- Custom allocators.

## Where to go for more depth

When you're comfortable reading the code and you want to *write* in
this style:

- **A Tour of C++** (Stroustrup) — the language designer's high-level
  overview. Read it after you can write small programs.
- **Effective Modern C++** (Meyers) — practical guidelines. Less useful
  the first year, indispensable the second.

## Next

When you're ready: [Chapter 02 — Linear algebra](../02_linear_algebra/README.md).
