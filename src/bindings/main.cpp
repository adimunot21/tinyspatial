/// \file main.cpp
/// \brief nanobind module exposing the public C++ API to Python.
///
/// Conventions (Python side mirrors C++ § 5–11):
///   - Spatial vectors are angular-first: `(ω; v)` for twists, `(τ; f)` for
///     wrenches. Always 6-vectors.
///   - SO3 / SE3 are lightweight value types; you can pass numpy 3×3 / 4×4
///     arrays in interchangeably via the explicit constructors.
///   - Functions that need `Data` allocate a fresh `Data` internally. Pass your
///     own through the C++ API if you want to reuse the scratchpad across calls.
#include <nanobind/eigen/dense.h>
#include <nanobind/nanobind.h>
#include <nanobind/operators.h>
#include <nanobind/stl/optional.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/tuple.h>
#include <nanobind/stl/vector.h>
#include <optional>
#include <string>
#include <tuple>
#include <vector>

#include "tinyspatial/algo/aba.hpp"
#include "tinyspatial/algo/crba.hpp"
#include "tinyspatial/algo/forward_kinematics.hpp"
#include "tinyspatial/algo/jacobian.hpp"
#include "tinyspatial/algo/rnea.hpp"
#include "tinyspatial/diff/fk_derivatives.hpp"
#include "tinyspatial/diff/rnea_derivatives.hpp"
#include "tinyspatial/ik/differentiable.hpp"
#include "tinyspatial/ik/dls.hpp"
#include "tinyspatial/ik/nullspace.hpp"
#include "tinyspatial/liegroup/se3.hpp"
#include "tinyspatial/liegroup/so3.hpp"
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
  m.doc() = "tinyspatial — rigid-body kinematics and dynamics in C++20.";

  // ---------------------------------------------------------------------------
  // Lie groups
  // ---------------------------------------------------------------------------

  nb::class_<SO3>(m, "SO3", "3-D rotation, stored as a unit quaternion (w >= 0).")
      .def(nb::init<>(), "Identity rotation.")
      .def(nb::init<const Eigen::Ref<const Matrix3>&>(), nb::arg("rotation_matrix"),
           "From a 3x3 rotation matrix (orthonormal, det ~ +1).")
      .def_static("identity", &SO3::identity)
      .def_static(
          "from_quaternion",
          [](Scalar w, Scalar x, Scalar y, Scalar z) { return SO3(Quaternion(w, x, y, z)); },
          nb::arg("w"), nb::arg("x"), nb::arg("y"), nb::arg("z"),
          "From a (w, x, y, z) Hamilton quaternion; normalised internally.")
      .def_static("exp", &SO3::exp, nb::arg("omega"),
                  "Exponential map: rotation vector (axis * angle) -> SO3.")
      .def("log", &SO3::log, "Logarithm: rotation -> rotation vector w, ||w|| in [0, pi].")
      .def(
          "matrix", [](const SO3& r) { return r.matrix(); }, "Return the 3x3 rotation matrix.")
      .def(
          "quaternion",
          [](const SO3& r) {
            const Quaternion& q = r.quaternion();
            Eigen::Matrix<Scalar, 4, 1> out;
            out << q.w(), q.x(), q.y(), q.z();
            return out;
          },
          "Return the underlying (w, x, y, z) quaternion as a 4-vector.")
      .def("inverse", &SO3::inverse)
      .def(
          "act", [](const SO3& r, const Vector3& p) { return r.act(p); }, nb::arg("point"),
          "Rotate a 3-D point.")
      .def(nb::self * nb::self)
      .def("__repr__", [](const SO3& r) {
        const Quaternion& q = r.quaternion();
        return std::string("SO3(w=") + std::to_string(q.w()) + ", x=" + std::to_string(q.x()) +
               ", y=" + std::to_string(q.y()) + ", z=" + std::to_string(q.z()) + ")";
      });

  nb::class_<SE3>(m, "SE3", "Rigid transform in 3-D, stored as (SO3, translation).")
      .def(nb::init<>(), "Identity transform.")
      .def(nb::init<const SO3&, const Eigen::Ref<const Vector3>&>(), nb::arg("rotation"),
           nb::arg("translation"))
      .def_static("identity", &SE3::identity)
      .def_static(
          "from_matrix",
          [](const Eigen::Ref<const Matrix4>& m44) {
            return SE3(SO3(Matrix3(m44.topLeftCorner<3, 3>())),
                       Vector3(m44.topRightCorner<3, 1>()));
          },
          nb::arg("matrix"), "From a 4x4 homogeneous matrix.")
      .def_static("exp", &SE3::exp, nb::arg("xi"),
                  "Exponential map: twist xi = (omega; v) -> SE3 transform.")
      .def("log", &SE3::log, "Logarithm: SE3 transform -> twist xi = (omega; v).")
      .def(
          "matrix", [](const SE3& t) { return t.matrix(); }, "Return the 4x4 homogeneous matrix.")
      .def("rotation", &SE3::rotation)
      .def("translation", [](const SE3& t) -> Vector3 { return t.translation(); })
      .def("inverse", &SE3::inverse)
      .def("adjoint", &SE3::adjoint,
           "6x6 adjoint (angular-first); maps a twist in this frame to the parent frame.")
      .def("adjoint_inverse", &SE3::adjoint_inverse)
      .def(
          "act", [](const SE3& t, const Vector3& p) { return t.act(p); }, nb::arg("point"))
      .def(nb::self * nb::self)
      .def("__repr__", [](const SE3& t) {
        const Vector3& p = t.translation();
        return std::string("SE3(t=[") + std::to_string(p.x()) + ", " + std::to_string(p.y()) +
               ", " + std::to_string(p.z()) + "])";
      });

  // ---------------------------------------------------------------------------
  // Kinematic tree
  // ---------------------------------------------------------------------------

  nb::enum_<JacobianFrame>(m, "JacobianFrame")
      .value("LOCAL", JacobianFrame::kLocal)
      .value("WORLD", JacobianFrame::kWorld)
      .value("LOCAL_WORLD_ALIGNED", JacobianFrame::kLocalWorldAligned)
      .export_values();

  nb::class_<Model>(m, "Model", "Robot model: topology + joint types + inertias.")
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
      .def_prop_ro("idx_v", [](const Model& self) -> const std::vector<int>& { return self.idx_v; })
      .def(
          "joint_id",
          [](const Model& self, const std::string& name) {
            for (int i = 0; i < self.njoints(); ++i) {
              if (self.joint_names[i] == name) {
                return i;
              }
            }
            throw nb::value_error(("unknown joint name: " + name).c_str());
          },
          nb::arg("name"), "Lookup joint index by name. Raises ValueError if not found.");

  nb::class_<Data>(m, "Data", "Per-configuration scratchpad sized to a Model.")
      .def(nb::init<const Model&>(), nb::arg("model"))
      .def(
          "pose_in_world",
          [](const Data& d, int i) -> Matrix4 { return d.pose_in_world.at(i).matrix(); },
          nb::arg("joint_id"), "4x4 world pose of joint frame `i`. Valid after forward_kinematics.")
      .def(
          "pose_in_parent",
          [](const Data& d, int i) -> Matrix4 { return d.pose_in_parent.at(i).matrix(); },
          nb::arg("joint_id"), "4x4 pose of joint `i` in its parent's frame.");

  m.def("build_model_from_urdf_file", &build_model_from_urdf_file, nb::arg("path"),
        "Load a URDF file from disk and build a Model.");

  // ---------------------------------------------------------------------------
  // Kinematics
  // ---------------------------------------------------------------------------

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
      "Return a list of 4x4 homogeneous matrices, one per joint, expressing each joint frame's "
      "pose in the world.");

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
      "6 x nv geometric Jacobian of `link_id`'s body frame in the chosen reference frame.");

  m.def(
      "compute_joint_jacobians",
      [](const Model& model, const VectorX& q) {
        Data d(model);
        std::vector<Matrix6X> jacobians;
        compute_joint_jacobians(model, d, q, jacobians);
        return jacobians;
      },
      nb::arg("model"), nb::arg("q"),
      "Per-joint body-frame Jacobians (one 6 x nv matrix per joint) computed in a single forward "
      "sweep.");

  // ---------------------------------------------------------------------------
  // Dynamics
  // ---------------------------------------------------------------------------

  m.def(
      "rnea",
      [](const Model& model, const VectorX& q, const VectorX& v, const VectorX& a,
         const Vector3& gravity) {
        Data d(model);
        VectorX tau(model.nv());
        rnea(model, d, q, v, a, tau, gravity);
        return tau;
      },
      nb::arg("model"), nb::arg("q"), nb::arg("v"), nb::arg("a"),
      nb::arg("gravity") = Vector3(0, 0, -9.81),
      "Inverse dynamics: joint torques tau required for (q, v, a) under `gravity` (world frame).");

  m.def(
      "crba",
      [](const Model& model, const VectorX& q) {
        Data d(model);
        MatrixX big_m(model.nv(), model.nv());
        crba(model, d, q, big_m);
        return big_m;
      },
      nb::arg("model"), nb::arg("q"),
      "Composite Rigid Body Algorithm: joint-space inertia matrix M(q) (nv x nv, SPD).");

  m.def(
      "aba",
      [](const Model& model, const VectorX& q, const VectorX& v, const VectorX& tau,
         const Vector3& gravity) {
        Data d(model);
        VectorX qdd(model.nv());
        aba(model, d, q, v, tau, qdd, gravity);
        return qdd;
      },
      nb::arg("model"), nb::arg("q"), nb::arg("v"), nb::arg("tau"),
      nb::arg("gravity") = Vector3(0, 0, -9.81),
      "Articulated-Body Algorithm: forward dynamics. Returns ddq for (q, v, tau).");

  m.def(
      "rnea_derivatives",
      [](const Model& model, const VectorX& q, const VectorX& v, const VectorX& a,
         const Vector3& gravity) {
        Data d(model);
        MatrixX dtau_dq(model.nv(), model.nv());
        MatrixX dtau_dv(model.nv(), model.nv());
        MatrixX dtau_da(model.nv(), model.nv());
        rnea_derivatives(model, d, q, v, a, dtau_dq, dtau_dv, dtau_da, gravity);
        return std::make_tuple(dtau_dq, dtau_dv, dtau_da);
      },
      nb::arg("model"), nb::arg("q"), nb::arg("v"), nb::arg("a"),
      nb::arg("gravity") = Vector3(0, 0, -9.81),
      "Analytical RNEA partials: (dtau/dq, dtau/dv, dtau/da), each nv x nv. dtau/da == M(q).");

  // ---------------------------------------------------------------------------
  // Inverse kinematics
  // ---------------------------------------------------------------------------

  nb::class_<DlsOptions>(m, "DlsOptions", "Tuning parameters for damped-least-squares IK.")
      .def(nb::init<>())
      .def_rw("max_iters", &DlsOptions::max_iters)
      .def_rw("tolerance", &DlsOptions::tolerance)
      .def_rw("damping", &DlsOptions::damping)
      .def_rw("step_size", &DlsOptions::step_size);

  nb::class_<NullspaceOptions>(m, "NullspaceOptions",
                               "Tuning parameters for task-priority (null-space) IK.")
      .def(nb::init<>())
      .def_rw("max_iters", &NullspaceOptions::max_iters)
      .def_rw("tolerance", &NullspaceOptions::tolerance)
      .def_rw("damping", &NullspaceOptions::damping)
      .def_rw("step_size", &NullspaceOptions::step_size)
      .def_rw("secondary_gain", &NullspaceOptions::secondary_gain);

  nb::class_<IkResult>(m, "IkResult", "Output of an IK solve.")
      .def_ro("q", &IkResult::q)
      .def_ro("error", &IkResult::error)
      .def_ro("iterations", &IkResult::iterations)
      .def_ro("converged", &IkResult::converged);

  m.def(
      "solve_ik_dls",
      [](const Model& model, int link_id, const Matrix4& target_in_world, const VectorX& q_init,
         std::optional<DlsOptions> options) {
        Data d(model);
        const SE3 target = SE3(SO3(Matrix3(target_in_world.topLeftCorner<3, 3>())),
                               Vector3(target_in_world.topRightCorner<3, 1>()));
        return solve_ik_dls(model, d, link_id, target, q_init, options.value_or(DlsOptions{}));
      },
      nb::arg("model"), nb::arg("link_id"), nb::arg("target_in_world"), nb::arg("q_init"),
      nb::arg("options") = nb::none(),
      "Damped-least-squares IK placing `link_id`'s body frame at `target_in_world` (4x4).");

  m.def(
      "solve_ik_nullspace",
      [](const Model& model, int link_id, const Matrix4& target_in_world, const VectorX& q_init,
         const VectorX& q_rest, std::optional<NullspaceOptions> options) {
        Data d(model);
        const SE3 target = SE3(SO3(Matrix3(target_in_world.topLeftCorner<3, 3>())),
                               Vector3(target_in_world.topRightCorner<3, 1>()));
        return solve_ik_nullspace(model, d, link_id, target, q_init, q_rest,
                                  options.value_or(NullspaceOptions{}));
      },
      nb::arg("model"), nb::arg("link_id"), nb::arg("target_in_world"), nb::arg("q_init"),
      nb::arg("q_rest"), nb::arg("options") = nb::none(),
      "Task-priority IK: primary Cartesian target + secondary 'stay near q_rest' posture task "
      "projected through the null-space.");

  m.def(
      "ik_implicit_derivative",
      [](const Model& model, int link_id, const VectorX& q_star, Scalar damping) {
        Data d(model);
        return ik_implicit_derivative(model, d, link_id, q_star, damping);
      },
      nb::arg("model"), nb::arg("link_id"), nb::arg("q_star"), nb::arg("damping") = 1e-2,
      "Analytical derivative dq*/dT* via the implicit function theorem at a converged IK "
      "solution. Returns an nv x 6 matrix (body-frame target perturbations).");
}
