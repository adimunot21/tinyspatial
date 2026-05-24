/// \file main.cpp
/// \brief Minimal nanobind module: just enough surface area to run the
/// Pinocchio validation harness in `tests/validation/`. Full Python bindings
/// (the public Python package, examples, etc.) are Phase 8 work.
///
/// Exposed:
///   - `JacobianFrame` enum
///   - `Model` (read-only, with name / nq / nv / parent / names)
///   - `build_model_from_urdf_file(path) -> Model`
///   - `forward_kinematics(model, q) -> list of 4x4 numpy arrays`
///   - `compute_jacobian(model, q, link_id, frame) -> 6x nv numpy array`
#include <nanobind/eigen/dense.h>
#include <nanobind/nanobind.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/vector.h>
#include <vector>

#include "tinyspatial/algo/forward_kinematics.hpp"
#include "tinyspatial/algo/jacobian.hpp"
#include "tinyspatial/urdf/urdf_loader.hpp"

namespace nb = nanobind;
using namespace tinyspatial;

// NB_MODULE expands to macro-generated boilerplate (C-style arrays, casts,
// unnamed parameters) that the modernize/cppcoreguidelines checks flag. The
// macro is out of our hands; NOLINTBEGIN/END isn't affected by clang-format
// line-wrapping the way NOLINTNEXTLINE can be.
// NOLINTBEGIN(cppcoreguidelines-avoid-c-arrays, modernize-avoid-c-arrays,
// cppcoreguidelines-pro-bounds-array-to-pointer-decay, cppcoreguidelines-pro-type-cstyle-cast)
NB_MODULE(_tinyspatial, m) {  // NOLINT(readability-identifier-naming, readability-named-parameter)
  // NOLINTEND(cppcoreguidelines-avoid-c-arrays, modernize-avoid-c-arrays,
  // cppcoreguidelines-pro-bounds-array-to-pointer-decay, cppcoreguidelines-pro-type-cstyle-cast)
  m.doc() = "Validation-only Python bindings for tinyspatial (Phase 4).";

  nb::enum_<JacobianFrame>(m, "JacobianFrame")
      .value("LOCAL", JacobianFrame::kLocal)
      .value("WORLD", JacobianFrame::kWorld)
      .value("LOCAL_WORLD_ALIGNED", JacobianFrame::kLocalWorldAligned)
      .export_values();

  nb::class_<Model>(m, "Model")
      .def_prop_ro("name", [](const Model& self) -> const std::string& { return self.name; })
      .def_prop_ro("njoints", &Model::njoints)
      .def_prop_ro("nq", &Model::nq)
      .def_prop_ro("nv", &Model::nv)
      .def_prop_ro(
          "joint_names",
          [](const Model& self) -> const std::vector<std::string>& { return self.joint_names; })
      .def_prop_ro(
          "link_names",
          [](const Model& self) -> const std::vector<std::string>& { return self.link_names; })
      .def_prop_ro("parent",
                   [](const Model& self) -> const std::vector<int>& { return self.parent; })
      .def_prop_ro("idx_q", [](const Model& self) -> const std::vector<int>& { return self.idx_q; })
      .def_prop_ro("idx_v",
                   [](const Model& self) -> const std::vector<int>& { return self.idx_v; });

  m.def("build_model_from_urdf_file", &build_model_from_urdf_file, nb::arg("path"));

  m.def(
      "forward_kinematics",
      [](const Model& model, const VectorX& q) {
        Data d(model);
        forward_kinematics(model, d, q);
        std::vector<Matrix4> poses;
        poses.reserve(static_cast<size_t>(model.njoints()));
        for (int i = 0; i < model.njoints(); ++i) {
          poses.push_back(d.pose_in_world[i].matrix());
        }
        return poses;
      },
      nb::arg("model"), nb::arg("q"),
      "Returns a list of 4x4 homogeneous matrices, one per joint, expressing "
      "each joint frame's pose in the world.");

  m.def(
      "compute_jacobian",
      [](const Model& model, const VectorX& q, int link_id, JacobianFrame frame) {
        Data d(model);
        forward_kinematics(model, d, q);
        Matrix6X j(6, model.nv());
        compute_jacobian(model, d, link_id, j, frame);
        return j;
      },
      nb::arg("model"), nb::arg("q"), nb::arg("link_id"), nb::arg("frame") = JacobianFrame::kLocal,
      "6 x nv geometric Jacobian of `link_id`'s body frame in the chosen "
      "reference frame (LOCAL / WORLD / LOCAL_WORLD_ALIGNED).");
}
