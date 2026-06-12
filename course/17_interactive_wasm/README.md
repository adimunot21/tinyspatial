# Chapter 17 — Interactive: the library in your browser

Everything so far has run on your machine: you built the C++, ran the
tests, maybe poked at the Python bindings. This chapter does something
different — it runs the **exact same library inside a web page**, with no
server doing the math. You move a slider, and a C++ `forward_kinematics`
call computes the new arm pose, in your browser, at 60 fps.

That's possible because of two choices made long before this chapter:

1. The library is **header-mostly with a tiny dependency surface** — no
   Boost, no ROS, no system libraries. A compiler that targets
   WebAssembly (Emscripten) can swallow the whole thing and emit a small
   `.wasm`.
2. The URDF loader can build a model from a **string**, not just a file.
   Browsers have no filesystem; you hand it the URDF text directly.

## What WebAssembly buys you

WebAssembly (WASM) is a portable binary instruction format that runs in
every modern browser at near-native speed. Emscripten is a toolchain
(`emcc`/`em++`, built on LLVM/Clang) that compiles C and C++ to it,
together with the glue — [embind](https://emscripten.org/docs/porting/connecting_cpp_and_javascript/embind.html)
— that exposes C++ classes and functions to JavaScript.

So the path is: the same `.cpp`/`.hpp` files → `emcc` → `tinyspatial.wasm`
plus a small `tinyspatial.js` loader → a `Robot` object you can call from
JavaScript. No reimplementation of the kinematics in JS; no rounding
differences; one source of truth.

## The binding

The web edge is one short file,
[`src/web/bindings_wasm.cpp`](../../src/web/bindings_wasm.cpp). It mirrors
the Python binding (`bindings/main.cpp`) but for JavaScript, and exposes a
single class:

```cpp
class Robot {
 public:
  explicit Robot(const std::string& urdf_xml)
      : model_(build_model_from_urdf(urdf_xml)), data_(model_) {}

  emscripten::val jointPositions(const emscripten::val& q_js) {
    // read q from the JS array, run the real forward_kinematics,
    // push every joint frame's world origin into a JS array, return it.
  }
  // …
};

EMSCRIPTEN_BINDINGS(tinyspatial) {
  emscripten::class_<Robot>("Robot")
      .constructor<std::string>()
      .function("jointPositions", &Robot::jointPositions);
}
```

`emscripten::val` is embind's bridge to a live JavaScript value — here a
plain `Array` of numbers in each direction. That's the whole interface
the demo needs.

## Calling it from JavaScript

The module is built `MODULARIZE`d, so you get a factory function:

```js
const Module = await createTinyspatial();      // loads the .wasm
const robot  = new Module.Robot(urdfString);   // construct from URDF text
const flat   = robot.jointPositions([0.5, 0.3]); // [x0,y0,z0, x1,y1,z1, …]
```

The [demo page](../../web/demo/) wires that to two range sliders and
redraws an SVG polyline from the returned positions every time you move
one. Open it and you are literally watching Featherstone's frame
propagation run in your browser.

## Build and run it yourself

```bash
# with the Emscripten SDK active (source emsdk_env.sh):
cmake --preset=wasm
cmake --build build/wasm -j           # -> build/wasm/tinyspatial.{js,wasm}

cp build/wasm/tinyspatial.js build/wasm/tinyspatial.wasm web/demo/
python3 -m http.server -d web/demo 8000
# open http://localhost:8000
```

If you don't have Emscripten installed, the `wasm` CI workflow builds the
module on every push and uploads `tinyspatial.js` + `tinyspatial.wasm` as
an artifact you can download.

## Why this matters for a course

A library you can *poke at live* — change a number, see the robot move —
teaches faster than a wall of API docs. The same WASM module that powers
this demo can be embedded directly in these chapter pages, so the
spatial-algebra and kinematics chapters can grow runnable widgets:
joint-space sliders, a manipulability ellipsoid that deforms as you near
a singularity, a drag-the-target IK toy. Same C++, no backend.

## Where this lives in the library

| Concept | File |
| ------- | ---- |
| WASM binding (embind) | [`src/web/bindings_wasm.cpp`](../../src/web/bindings_wasm.cpp) |
| Build target + `wasm` preset | [`CMakeLists.txt`](../../CMakeLists.txt), [`CMakePresets.json`](../../CMakePresets.json) |
| CI build of the module | [`.github/workflows/wasm.yml`](../../.github/workflows/wasm.yml) |
| Interactive demo | [`web/demo/`](../../web/demo/) |
| String URDF loader (no filesystem) | [`include/tinyspatial/urdf/urdf_loader.hpp`](../../include/tinyspatial/urdf/urdf_loader.hpp) |

## Further reading

- **Emscripten docs — embind**: connecting C++ and JavaScript. The whole
  `Robot` binding is one page of this manual.
- **MDN — WebAssembly concepts**: what WASM is and why it's fast.
- **`MODULARIZE` + `EXPORT_NAME`**: the Emscripten settings that turn the
  output into a clean `await createTinyspatial()` factory instead of a
  global.
