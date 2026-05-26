# 14 · Python bindings

C++ is fast and explicit. Python is slow, dynamic, and *full of libraries you
actually want to use*. If you're going to ship a robotics library to a wide
audience, the binding layer is non-negotiable — most users want to
prototype in Jupyter, plot with matplotlib, train networks in PyTorch, and
generally not write CMake.

This chapter is about how that interface is built and what it costs.

## What you'll learn

After this chapter you'll be able to:

- Explain *why* a header-mostly C++ library still ships a Python module.
- Read the binding glue in `src/bindings/main.cpp` line by line.
- Use `tinyspatial` from Python — load a robot, run FK, run RNEA, solve IK.
- Avoid the three big nanobind footguns: ownership, GIL, Eigen layout.

## The chapters

1. [Why we wrap C++ in Python](01_why_wrap_cpp.md) — the speed-vs-ergonomics
   tradeoff, why nanobind beats pybind11, and what 'zero-copy' actually means
   for a NumPy ↔ Eigen handoff.
2. [Reading the nanobind glue](02_reading_the_glue.md) — anatomy of
   `src/bindings/main.cpp`: `NB_MODULE`, `nb::class_<T>`, operator overloads,
   lambdas as trampolines, optional arguments.
3. [Using `tinyspatial` from Python](03_using_from_python.md) — a guided tour
   through the three example notebooks in `python/examples/`.
4. [Pitfalls and good habits](04_pitfalls.md) — Eigen storage order,
   reference lifetime under `rv_policy`, the GIL during long C++ calls,
   and how to debug a `SIGSEGV` from inside a Jupyter kernel.

## Where this lives in the library

| Concept                  | File path                                      |
| ------------------------ | ---------------------------------------------- |
| Binding entrypoint       | [`src/bindings/main.cpp`](../../src/bindings/main.cpp) |
| Python package init      | [`python/tinyspatial/__init__.py`](../../python/tinyspatial/__init__.py) |
| Type stubs               | [`python/tinyspatial/__init__.pyi`](../../python/tinyspatial/__init__.pyi) |
| Example notebooks        | [`python/examples/`](../../python/examples/)   |
| Python parity tests      | [`python/tests/test_parity.py`](../../python/tests/test_parity.py) |
