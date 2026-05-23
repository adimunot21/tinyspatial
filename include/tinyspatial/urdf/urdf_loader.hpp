/// \file urdf_loader.hpp
/// \brief Build a `Model` from a URDF XML document.
///
/// We parse a *strict subset* of URDF — pure XML only. xacro substitution,
/// ROS-specific tags (`<transmission>`, `<gazebo>`, …), `<visual>`, and
/// `<collision>` blocks are deliberately ignored or rejected (CLAUDE.md §3,
/// PROJECT_PLAN §3 Phase 3 risk). Anything required for kinematics and
/// dynamics is supported: links with `<inertial>`, joints of type `revolute`,
/// `continuous`, `prismatic`, `fixed`, and `floating`, with `<origin>` and
/// `<axis>`.
#ifndef TINYSPATIAL_URDF_URDF_LOADER_HPP
#define TINYSPATIAL_URDF_URDF_LOADER_HPP

#include <stdexcept>
#include <string>
#include <string_view>

#include "tinyspatial/model/model.hpp"

namespace tinyspatial {

/// Thrown by the URDF loader when the input is not a parseable subset.
/// The `what()` message names the offending element when possible.
class UrdfParseError : public std::runtime_error {
 public:
  using std::runtime_error::runtime_error;
};

/// Build a `Model` from a URDF XML string.
///
/// \throws UrdfParseError on malformed XML, unknown joint type, missing
///         required attributes, or cyclic / disconnected link graphs. We
///         deliberately surface errors instead of guessing; URDF parsing is
///         user-input handling, not a programmer-error precondition.
[[nodiscard]] Model build_model_from_urdf(std::string_view xml);

/// Convenience: read a URDF from a file on disk and build the model.
[[nodiscard]] Model build_model_from_urdf_file(const std::string& path);

}  // namespace tinyspatial

#endif  // TINYSPATIAL_URDF_URDF_LOADER_HPP
