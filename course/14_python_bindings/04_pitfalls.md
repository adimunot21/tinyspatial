# Pitfalls and good habits

Three categories of bug appear *only* in the Python ↔ C++ handoff, never
in pure-C++ code. If you're building your own bindings, you'll hit
these.

## 1. Eigen storage order and NumPy

Eigen defaults to **column-major** storage; NumPy defaults to
**row-major** (C-order). When you pass a NumPy matrix into a C++
function expecting `Eigen::Matrix3d`, nanobind has to either reinterpret
strides or copy.

The two cases:

- **C-contiguous, double, right shape** → zero-copy, just a strided view.
- **Anything else (non-contiguous slice, wrong dtype, transposed view)** →
  silent copy + transpose, then your function runs on the copy.

What 'wrong dtype' looks like in practice:

```python
q = np.array([1, 2, 3])                    # dtype=int64 — will be copied
q = np.array([1, 2, 3], dtype=np.float32)  # also copied (need float64)
q = np.array([1, 2, 3], dtype=np.float64)  # correct, zero-copy
```

If you're seeing unexpected slowness in a tight Python loop, profile
with `py-spy` and look for unexpected `memcpy` lines.

The mitigation is to always explicitly type-annotate arrays you pass to
C++:

```python
q = np.zeros(model.nq, dtype=np.float64)   # explicit
target = np.eye(4, dtype=np.float64)
```

## 2. Reference lifetime

A common nanobind trap:

```cpp
.def("rotation", &SE3::rotation)   // returns const SO3&
```

By default, nanobind's `rv_policy::automatic` will *copy* the returned
`SO3` into a new Python object. That's safe but wastes work. If you
want to share storage (`rv_policy::reference_internal`), then the
returned Python object holds a *reference* to the parent's memory — and
*you* are responsible for ensuring the parent outlives it.

```python
T = ts.SE3.from_matrix(np.eye(4))   # parent SE3
R = T.rotation()                     # is R safe to use after T is dead?
del T                                # if reference_internal, R is now dangling
```

For `tinyspatial`, we default to `rv_policy::automatic`. Things are
copied. It's slightly slower but always safe. Don't change this unless
you've profiled and *know* you need to.

## 3. The GIL during long C++ calls

Python's Global Interpreter Lock (GIL) is acquired automatically when
you call into Python from a thread, and held until the function returns.
For a 5-microsecond RNEA call this is fine. For an IK solve that runs
200 iterations of damped Newton, this might be 0.5 ms — and if you're
trying to drive a 1 kHz control loop from Python, that's a problem.

nanobind lets you *release* the GIL during a long C++ call:

```cpp
m.def("slow_thing", [](...) {
  return nb::call_guard<nb::gil_scoped_release>(),
         /* the actual call */;
}, ...);
```

We currently don't release the GIL anywhere. The reason: the same
algorithm called from a Python thread *with* the GIL held is also slow
because of the GIL itself — you can't actually use Python threads to
parallelise this kind of work. The right tool is `multiprocessing` (one
Python process per CPU core, each running C++) or directly calling the
C++ API from a C++ thread.

If you find yourself needing GIL release, you're probably better served
by the C++ API or by a vectorised batch entry point (Phase 9 plan).

## 4. The 'wrong index' problem with Pinocchio

The biggest validation footgun in this codebase: Pinocchio includes a
'universe' joint at index 0 and lists movable joints at indices 1..N.
`tinyspatial` lists every joint from 0.

If you compare directly, your last-joint pose from pinocchio is at
index `pin_model.njoints - 1`, but your last-joint pose from tinyspatial
is at index `ts_model.njoints - 1` (one smaller). The Python parity
tests in [`python/tests/test_parity.py`](../../python/tests/test_parity.py)
handle this; if you're rolling your own comparison, watch out.

## 5. Quaternion sign ambiguity

`tinyspatial` normalises to `w >= 0` (CLAUDE.md §15). When you read a
quaternion out of `SO3.quaternion()`, it's the canonical representative,
not necessarily the one you put in:

```python
R = ts.SO3.from_quaternion(-1.0, 0.0, 0.0, 0.0)  # identity (with negated sign)
print(R.quaternion())   # [1.0, 0.0, 0.0, 0.0]  — flipped to canonical
```

For visualisation this never matters (same rotation). For control
schemes that interpolate raw quaternion components, it absolutely does.
Use `SLERP` on rotations or work in the rotation-vector tangent space
(`SO3.log()`) and you avoid the issue entirely.

## 6. Debugging a SIGSEGV from a Jupyter kernel

If you hit a segfault inside C++, the Jupyter kernel just dies — no
backtrace. Two debugging strategies:

1. **Build with sanitizers.** `cmake --preset=debug` enables ASan and
   UBSan; the C++ tests catch most of these. If a Jupyter call crashes
   but the unit test passes, the bug is in the binding glue or in how
   you're calling from Python.
2. **Run under `gdb`** with `gdb --args python my_script.py`, then `run`,
   then when it crashes, `bt`. You'll see the full C++ stack trace
   leading to the crash.

## 7. Memory growth with NumPy round-trips

Every `ts.forward_kinematics(...)` call allocates a fresh
`vector<Matrix4>` on the C++ side, which becomes a Python `list[ndarray]`
of length `njoints`. If you loop this 100,000 times, you're allocating
~70 MB of Python garbage. Python's garbage collector will catch up
*eventually*, but if you see RSS climbing, you're spending a lot of
time in allocation. The C++ benchmarks bypass this entirely by reusing
a single `Data` instance.

## What good habits look like

- Always `dtype=np.float64`, `order='C'` for matrices you pass in.
- Default options first; only override `damping`, `tolerance`, etc. when
  you have a measured reason.
- For tight loops, profile *before* assuming the binding is the
  bottleneck. It usually isn't.
- For real-time control, drop to C++. The Python layer is for
  prototyping.
- Keep the binding layer thin. If you find yourself writing pure-Python
  helpers, put them in `python/tinyspatial/__init__.py`, not in the
  binding.

> ## Where this lives in the library
>
> | Concept                | File / line                              |
> | ---------------------- | ---------------------------------------- |
> | Binding entrypoint     | [`src/bindings/main.cpp`](../../src/bindings/main.cpp) |
> | Python package init    | [`python/tinyspatial/__init__.py`](../../python/tinyspatial/__init__.py) |
> | Parity tests           | [`python/tests/test_parity.py`](../../python/tests/test_parity.py) |
