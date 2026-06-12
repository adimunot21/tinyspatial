# `web/` — tinyspatial in the browser

The library compiles to WebAssembly so the course can run real `tinyspatial`
client-side. This directory holds the WASM binding's *consumers*; the binding
itself is [`src/web/bindings_wasm.cpp`](../src/web/bindings_wasm.cpp).

## Build the module

You need the [Emscripten SDK](https://emscripten.org/docs/getting_started/downloads.html)
on your `PATH` (`source path/to/emsdk/emsdk_env.sh`, which sets `$EMSDK`). Then:

```bash
cmake --preset=wasm          # configures with the Emscripten toolchain
cmake --build build/wasm -j   # -> build/wasm/tinyspatial.js + tinyspatial.wasm
```

CI does exactly this in [`.github/workflows/wasm.yml`](../.github/workflows/wasm.yml)
and uploads the pair as a build artifact, so you can also just download it from
a green `wasm` run.

## Run the demo

Copy the two build outputs next to the demo and serve the folder over HTTP
(browsers won't instantiate a `.wasm` from a `file://` URL):

```bash
cp build/wasm/tinyspatial.js build/wasm/tinyspatial.wasm web/demo/
python3 -m http.server -d web/demo 8000
# open http://localhost:8000
```

[`web/demo/`](demo/) is a dependency-free page: `index.html` loads the module,
`app.js` builds a 2-link arm from a URDF string and redraws it from the joint
positions `forward_kinematics` returns as you move the sliders. No JavaScript
reimplementation of the kinematics — it's the compiled C++.

## The binding surface

`createTinyspatial()` (an ES module factory, `MODULARIZE`d) resolves to a module
exposing one class:

```js
const Module = await createTinyspatial();
const robot = new Module.Robot(urdfXmlString); // browser has no filesystem
robot.nq();                 // configuration dimension
robot.njoints();            // number of joint frames
robot.jointPositions(q);    // q: number[]  ->  flat [x0,y0,z0, x1,y1,z1, …]
```

Extending it (Jacobians, IK, RNEA) is a matter of adding `.function(...)` lines
to `EMSCRIPTEN_BINDINGS` in `src/web/bindings_wasm.cpp` — the algorithms are
already there.
