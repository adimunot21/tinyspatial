/// \file bindings_wasm.cpp
/// \brief Emscripten/embind binding that compiles the library to WebAssembly so
/// the course can run real tinyspatial in the browser.
///
/// This is the *web* edge of the library, the analogue of `bindings/main.cpp`
/// (nanobind) for Python. It is compiled only under Emscripten
/// (`cmake --preset=wasm`); the native build never sees it. The header-mostly,
/// tiny-dependency design is what makes the resulting `.wasm` small.
///
/// Exposes a single `Robot` class: construct it from a URDF *string* (the
/// browser has no filesystem), then call `jointPositions(q)` to get the world
/// position of every joint frame — enough to draw a live, interactive arm.
#include <emscripten/bind.h>
#include <emscripten/val.h>
#include <string>

#include "tinyspatial/algo/forward_kinematics.hpp"
#include "tinyspatial/model/model.hpp"
#include "tinyspatial/urdf/urdf_loader.hpp"

namespace {

using tinyspatial::Data;
using tinyspatial::Model;
using tinyspatial::VectorX;

/// A loaded robot, ready for repeated forward-kinematics queries from JS.
class Robot {
 public:
  explicit Robot(const std::string& urdf_xml)
      : model_(tinyspatial::build_model_from_urdf(urdf_xml)), data_(model_) {}

  [[nodiscard]] int nq() const { return model_.nq(); }
  [[nodiscard]] int nv() const { return model_.nv(); }
  [[nodiscard]] int njoints() const { return model_.njoints(); }

  /// Run forward kinematics for the configuration `q_js` (a JS array of length
  /// `nq()`) and return a flat JS array `[x0, y0, z0, x1, y1, z1, …]` of every
  /// joint frame's world-space origin.
  [[nodiscard]] emscripten::val jointPositions(const emscripten::val& q_js) {
    const int n = q_js["length"].as<int>();
    VectorX q(n);
    for (int i = 0; i < n; ++i) {
      q(i) = q_js[i].as<double>();
    }
    tinyspatial::forward_kinematics(model_, data_, q);

    emscripten::val out = emscripten::val::array();
    for (int i = 0; i < model_.njoints(); ++i) {
      const auto p = data_.pose_in_world[i].translation();
      out.call<void>("push", p.x());
      out.call<void>("push", p.y());
      out.call<void>("push", p.z());
    }
    return out;
  }

 private:
  Model model_;
  Data data_;
};

}  // namespace

EMSCRIPTEN_BINDINGS(tinyspatial) {
  emscripten::class_<Robot>("Robot")
      .constructor<std::string>()
      .function("nq", &Robot::nq)
      .function("nv", &Robot::nv)
      .function("njoints", &Robot::njoints)
      .function("jointPositions", &Robot::jointPositions);
}
