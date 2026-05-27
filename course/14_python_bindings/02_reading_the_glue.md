# Reading the nanobind glue

Open [`src/bindings/main.cpp`](../../src/bindings/main.cpp) and follow along.
Every piece of the Python surface area comes from this one file.

## The module macro

```cpp
NB_MODULE(_tinyspatial, m) {
  m.doc() = "tinyspatial — rigid-body kinematics and dynamics in C++20.";
  // ... bindings ...
}
```

`NB_MODULE` is a macro that expands into the C entry point Python expects:
`PyInit__tinyspatial`. The first argument is the **module name**, and it
must match the shared library name. Our CMake invocation is

```cmake
nanobind_add_module(_tinyspatial STABLE_ABI src/bindings/main.cpp)
```

so the resulting file is `_tinyspatial.cpython-3xx-x86_64-linux-gnu.so`.
The leading underscore is a Python convention: 'this module is internal,
don't import it directly.' The user-facing `tinyspatial` package
(`python/tinyspatial/__init__.py`) re-exports symbols from
`_tinyspatial` after adding any pure-Python helpers.

## Binding a class

```cpp
nb::class_<SO3>(m, "SO3", "3-D rotation, stored as a unit quaternion (w >= 0).")
    .def(nb::init<>(), "Identity rotation.")
    .def(nb::init<const Eigen::Ref<const Matrix3>&>(), nb::arg("rotation_matrix"))
    .def_static("identity", &SO3::identity)
    .def("log", &SO3::log, "Logarithm: rotation -> rotation vector w.")
    // ...
```

Method-chained `.def(...)` calls add Python attributes:

- **`nb::init<Args...>()`** binds a constructor. You can overload by listing
  multiple `nb::init` lines.
- **`.def("name", &Type::method)`** binds a C++ method by pointer-to-member.
- **`.def_static`** binds a static method (`SO3.identity()` not `r.identity()`).
- **`.def_prop_ro`** binds a read-only property (no `()` to call).

The string after the function pointer is the Python docstring — `help(ts.SO3)`
will print it.

## Lambdas as trampolines

When the C++ signature doesn't map cleanly onto what we want in Python,
we use a lambda:

```cpp
m.def(
    "forward_kinematics",
    [](const Model& model, const VectorX& q) {
      Data d(model);
      forward_kinematics(model, d, q);
      std::vector<Matrix4> poses;
      poses.reserve(model.njoints());
      for (int i = 0; i < model.njoints(); ++i) {
        poses.push_back(d.pose_in_world[i].matrix());
      }
      return poses;
    },
    nb::arg("model"), nb::arg("q"));
```

Two things are happening:

1. The C++ `forward_kinematics(model, data, q)` fills a `Data` object as a
   side-effect. The Python user shouldn't have to manage that — they want
   `forward_kinematics(model, q)` to *return* the result. The lambda
   allocates `Data` internally, fills it, then extracts the poses.
2. The `Data` stores `SE3` objects, but Python users want 4×4 NumPy
   arrays. The lambda calls `.matrix()` on each `SE3` to produce them.

The cost: one extra `Data` allocation per call (small) and an extra
`vector<Matrix4>` allocation (also small). For users who need to avoid
both, the C++ API is right there.

## Operator overloads

```cpp
.def(nb::self * nb::self)
```

The `nb::self` token says 'left and right operand are both this class.'
`nb::self * nb::self` binds `__mul__` — Python `a * b` will dispatch to
the C++ `a.operator*(b)`. There's also `nb::self + nb::self`,
`nb::self == nb::self`, etc.

## Optional arguments

We expose `IkOptions` as a configurable struct:

```cpp
m.def(
    "solve_ik_dls",
    [](const Model& model, int link_id, const Matrix4& target,
       const VectorX& q_init, std::optional<DlsOptions> options) {
      Data d(model);
      const SE3 t = ...;  // build SE3 from 4x4 matrix
      return solve_ik_dls(model, d, link_id, t, q_init,
                          options.value_or(DlsOptions{}));
    },
    nb::arg("model"), nb::arg("link_id"), nb::arg("target_in_world"),
    nb::arg("q_init"), nb::arg("options") = nb::none());
```

`std::optional<DlsOptions>` paired with `nb::arg("options") = nb::none()`
means Python can call `solve_ik_dls(model, link, target, q_init)` —
options defaults to `None`, the lambda swaps in a default-constructed
`DlsOptions{}`.

This is more Pythonic than requiring the user to instantiate an empty
options object every time.

## Enums

```cpp
nb::enum_<JacobianFrame>(m, "JacobianFrame")
    .value("LOCAL", JacobianFrame::kLocal)
    .value("WORLD", JacobianFrame::kWorld)
    .value("LOCAL_WORLD_ALIGNED", JacobianFrame::kLocalWorldAligned)
    .export_values();
```

`.export_values()` exposes the enum *members* at module scope, so you
can write `ts.LOCAL` as well as `ts.JacobianFrame.LOCAL`. Pinocchio does
the same; we match the convention.

## What's *not* in this file

Things you might expect that we deliberately omit:

- **Python helper functions** — anything that doesn't need C++ goes in
  `python/tinyspatial/__init__.py` (or future submodules). Keep the C++
  binding layer thin.
- **String formatting beyond `__repr__`** — let Python's `numpy` and
  `repr` do their job.
- **Magic methods like `__getitem__` on `Model`** — they obscure what's
  happening. Explicit `.joint_id("name")` is better.
- **Pickling support** — none of our types should be persisted; the
  Model belongs to the URDF on disk.

Reading this glue is the best way to understand what `tinyspatial`
*offers* from Python. The next chapter shows what to *do* with it.
