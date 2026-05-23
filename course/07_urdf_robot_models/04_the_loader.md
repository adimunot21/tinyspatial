# The loader

The function signature is:

```cpp
Model build_model_from_urdf(std::string_view xml);
Model build_model_from_urdf_file(const std::string& path);
```

On a malformed URDF or unsupported feature it throws `UrdfParseError`, with
a message naming the offending element. On success it returns a fully built
`Model` you can hand to any algorithm. Let's look at how it works.

## Three passes

Internally the loader does the work in three explicit passes:

1. **Pass 1 — links.** Walk every `<link>` element, parse its `<inertial>`,
   store a `name → SpatialInertia` map. Reject duplicate names.
2. **Pass 2 — joints.** Walk every `<joint>` element, parse type, parent
   link, child link, origin, and axis. Build a `vector<JointInfo>`. Reject
   unknown joint types here, not later.
3. **Pass 3 — assembly.** Find the root link (the one that never appears as
   a child of any joint), then BFS outward, inserting joints into the
   `Model` in topological order with the right `parent` indices.

Each pass is a few dozen lines in
[`urdf_loader.cpp`](../../src/urdf/urdf_loader.cpp). The whole loader is
under 200 lines.

## Why three passes, not one

You could in principle build the Model as you walk the XML in document
order. But URDFs are *not* required to list links before the joints that
reference them, and joints are not required to appear in topological
order. A single pass would either fail on a forward reference or store
fix-ups to apply later. Three explicit passes are simpler and clearer.

## Error surface

The loader is the most user-input-facing code in the whole library. It
deliberately throws a *typed* `UrdfParseError` exception (rather than
`std::runtime_error`) so you can catch URDF problems specifically:

```cpp
try {
  Model m = build_model_from_urdf_file("robot.urdf");
  // use m
} catch (const UrdfParseError& e) {
  std::cerr << "URDF error: " << e.what() << "\n";
}
```

`UrdfParseError` is thrown for: malformed XML, missing required attributes
(`name`, `type`, `parent`, `child`), unknown joint types, dangling child
links, and cyclic / disconnected graphs.

## Fuzz testing

A `tests/unit/urdf/test_urdf_fuzz.cpp` runs 1000 random mutations of the
FR3 URDF and asserts the loader either succeeds or throws — *never crashes,
never has undefined behaviour.* The test runs under AddressSanitizer and
UndefinedBehaviorSanitizer in the debug build. This is the reason we are
careful about every pointer-from-XML — `tinyxml2` returns `nullptr` for
missing attributes, and every site that consumes one has to check.

## Where this lives in the library

| Concept | File · symbol |
| ------- | ------------- |
| Public API | [`urdf_loader.hpp`](../../include/tinyspatial/urdf/urdf_loader.hpp) · `build_model_from_urdf()`, `_file()` |
| Implementation | [`urdf_loader.cpp`](../../src/urdf/urdf_loader.cpp) — three passes |
| Error type | [`urdf_loader.hpp`](../../include/tinyspatial/urdf/urdf_loader.hpp) · `class UrdfParseError` |
| Fuzz test | [`test_urdf_fuzz.cpp`](../../tests/unit/urdf/test_urdf_fuzz.cpp) |

Next: [Exploring the three robots](05_exploring_the_robots.md).
