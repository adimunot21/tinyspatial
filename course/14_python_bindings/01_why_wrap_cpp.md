# Why we wrap C++ in Python

If C++ is faster, why doesn't *everyone* just write robotics code in C++?

The short answer: **the iteration loop matters more than peak speed for 95%
of users.** A typical robotics workflow is

1. Load a URDF.
2. Try a configuration.
3. Plot something.
4. Realise it's wrong.
5. Go to step 2.

In C++ that loop is: edit code, save, run CMake, link, run binary, look at
stdout. Each cycle is 5–30 seconds. In Python with a Jupyter notebook,
the cycle is sub-second.

The right answer is: **the library lives in C++, but the user lives in
Python.** The numerics — RNEA, ABA, IK, FK — run as compiled C++ inside a
single Python function call. The plotting, gluing, prototyping live in
Python where they belong. Done well, this gives you 100× the user base
of a C++-only library at maybe 5% overhead.

## Speed isn't the only argument

Some users *do* want to use the C++ API directly. Three audiences in
particular:

- **Embedded / real-time** — when you're running a 1 kHz control loop on
  a robot's onboard computer, every microsecond of Python overhead is a
  problem.
- **Other C++ projects** — Drake, MuJoCo, ROS components that want to
  consume `tinyspatial` as a header-mostly drop-in.
- **Performance engineers** — people who want to profile, vectorise, or
  inline by hand.

Those users skip Python entirely. The same library serves them; they
include `<tinyspatial/...>` and link.

The Python binding is the *front door*. The C++ library is the *engine
room*.

## Why nanobind, not pybind11

[pybind11](https://github.com/pybind/pybind11) is the historical default.
[nanobind](https://github.com/wjakob/nanobind), by the same author, is the
modern follow-up — same surface API but smaller, faster, with cleaner
C++17/20 ergonomics.

Concrete differences that matter:

| Concern             | pybind11                          | nanobind                       |
| ------------------- | --------------------------------- | ------------------------------ |
| Binary size         | ~1.5 MB per module                | ~150 KB per module             |
| Compile time        | Slow templated headers            | ~3× faster                     |
| Eigen integration   | Works but copies                  | First-class, zero-copy in many paths |
| C++ standard        | C++14 minimum                     | C++17 minimum                  |
| stable ABI          | Optional                          | First-class (`STABLE_ABI` flag) |

For a portfolio library that aspires to compile in under 90 seconds in CI,
nanobind is the obvious choice. (CLAUDE.md §10 sets that bar; see for
yourself — `cmake --build build/validation -j` finishes in 30–40 s on a
modest machine.)

## What 'zero-copy' actually means

When you write
```python
J = ts.compute_jacobian(model, q, link_id, ts.LOCAL)
```
the underlying C++ function fills an `Eigen::Matrix<double, 6, Eigen::Dynamic>`.
Eigen's default storage is **column-major**; NumPy's is **row-major**. So
strictly speaking, every `Eigen::Matrix → numpy.ndarray` handoff *could*
require a copy + transpose.

In practice nanobind's `nanobind/eigen/dense.h` is clever enough to
construct a NumPy view directly on top of the Eigen storage with the
right strides — *if* the Eigen object's lifetime outlives the NumPy
object. For us, the Jacobian is allocated inside the binding lambda and
returned by value, so nanobind moves it into a managed buffer and the
NumPy array owns it. One allocation, no copy, no transpose.

The other direction (NumPy → Eigen input) is also zero-copy *if* the
NumPy array is C-contiguous and the right dtype, and the parameter is
declared as `Eigen::Ref<const ...>`. Pass a non-contiguous slice and
nanobind will allocate and copy silently. Watch for this when you have
huge inputs.

## When is the binding the bottleneck?

Almost never. The Python overhead per call (parsing arguments, walking
references, releasing the GIL) is typically 1–10 microseconds. The
*computation* inside RNEA on a Franka is ~5 microseconds. So if you call
`ts.rnea(...)` in a tight Python `for` loop, yes — you're maybe paying
2× overhead vs C++.

The fix is to batch: do many configurations per C++ call. Phase 9
explores `rnea_batch(model, qs, vs, as)` for exactly this. For now, the
single-call API matches Pinocchio's and that's the right baseline.

## What you'll see next

Open `src/bindings/main.cpp`. It's ~300 lines. The next sub-chapter walks
through it block by block.
