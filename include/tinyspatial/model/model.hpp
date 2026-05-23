/// \file model.hpp
/// \brief `Model`: a robot's kinematic tree, and `Data`: the per-configuration
/// scratchpad consumed by algorithms.
///
/// The Model holds everything constant about a robot: the joint topology,
/// joint frame placements in their parents, and per-link spatial inertias.
/// Algorithms (forward kinematics, RNEA, …) read the Model and write into a
/// Data sized to match it. The split mirrors Pinocchio so the validation suite
/// can compare structures directly.
///
/// Joints are listed in topological order: `parent[i] < i` for every `i`, with
/// root joints having `parent[i] == -1`. We do not include a "universe" joint;
/// the world frame is the implicit parent of root joints.
#ifndef TINYSPATIAL_MODEL_MODEL_HPP
#define TINYSPATIAL_MODEL_MODEL_HPP

#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include "tinyspatial/core/types.hpp"
#include "tinyspatial/liegroup/se3.hpp"
#include "tinyspatial/model/joint.hpp"
#include "tinyspatial/spatial/force.hpp"
#include "tinyspatial/spatial/inertia.hpp"
#include "tinyspatial/spatial/motion.hpp"

namespace tinyspatial {

/// A kinematic tree. Construct it incrementally with `add_joint()`; algorithms
/// then read it as a constant lookup table.
class Model {
 public:
  std::string name;

  std::vector<std::string> joint_names;
  /// Name of the body (link) attached on the *child* side of each joint.
  std::vector<std::string> link_names;
  std::vector<Joint> joints;
  /// Parent joint index, or `-1` for a root joint (parent is the world frame).
  std::vector<int> parent;
  /// Placement of joint i's frame in its parent's body frame (when the joint
  /// is at `q = 0`). The full child-in-parent transform at a given `q` is
  /// `placement[i] * joint_transform(joints[i], q_slice(i))`.
  std::vector<SE3> placement;
  /// Spatial inertia of the link attached to each joint, expressed in that
  /// link's own body frame.
  std::vector<SpatialInertia> inertia;
  /// First index into the flat `q` vector for each joint.
  std::vector<int> idx_q;
  /// First index into the flat `v` (and `a`, `f`) vector for each joint.
  std::vector<int> idx_v;

  [[nodiscard]] int njoints() const { return static_cast<int>(joints.size()); }
  [[nodiscard]] int nq() const { return nq_; }
  [[nodiscard]] int nv() const { return nv_; }

  /// Add a joint to the tree. Returns the new joint's index. `parent_idx`
  /// must already exist (`-1` for a root joint). The new joint is appended
  /// at index `njoints() - 1`.
  int add_joint(std::string joint_name, std::string link_name, int parent_idx, Joint joint,
                const SE3& joint_placement, const SpatialInertia& link_inertia) {
    const int idx = njoints();
    const int this_nq = tinyspatial::nq(joint);
    const int this_nv = tinyspatial::nv(joint);
    joint_names.push_back(std::move(joint_name));
    link_names.push_back(std::move(link_name));
    joints.push_back(std::move(joint));
    parent.push_back(parent_idx);
    placement.push_back(joint_placement);
    inertia.push_back(link_inertia);
    idx_q.push_back(nq_);
    idx_v.push_back(nv_);
    nq_ += this_nq;
    nv_ += this_nv;
    return idx;
  }

  /// Linear lookup by joint name; returns `-1` if not found.
  [[nodiscard]] int find_joint(std::string_view target) const {
    for (int i = 0; i < njoints(); ++i) {
      if (joint_names[i] == target) {
        return i;
      }
    }
    return -1;
  }

 private:
  int nq_ = 0;
  int nv_ = 0;
};

/// Per-configuration scratchpad. Sized to match a Model at construction time;
/// later filled in by algorithms (forward kinematics in Phase 4, RNEA in
/// Phase 5, …).
class Data {
 public:
  /// Build buffers sized to `model.njoints()`.
  explicit Data(const Model& model)
      : pose_in_parent(model.njoints(), SE3::identity()),
        pose_in_world(model.njoints(), SE3::identity()),
        v(model.njoints()),
        a(model.njoints()),
        f(model.njoints()) {}

  /// Joint i's frame expressed in joint `parent[i]`'s frame
  /// (= placement * joint_transform). Filled by forward kinematics.
  /// Pinocchio name: `liMi`.
  std::vector<SE3> pose_in_parent;
  /// Joint i's frame expressed in the world frame. Filled by forward kinematics.
  /// Pinocchio name: `oMi`.
  std::vector<SE3> pose_in_world;
  /// Spatial velocity of body i in its own frame.
  std::vector<Motion> v;
  /// Spatial acceleration of body i in its own frame.
  std::vector<Motion> a;
  /// External / joint force on body i in its own frame.
  std::vector<Force> f;
};

}  // namespace tinyspatial

#endif  // TINYSPATIAL_MODEL_MODEL_HPP
