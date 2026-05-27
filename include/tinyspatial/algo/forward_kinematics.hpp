/// \file forward_kinematics.hpp
/// \brief Compute every joint frame's pose given a configuration `q`.
///
/// One outbound sweep over the kinematic tree. For each joint `i`,
///
///   pose_in_parent[i]  =  Model::placement[i] · joint_transform(joints[i], q_slice)
///   pose_in_world[i]   =  parent[i] == -1
///                           ? pose_in_parent[i]
///                           : pose_in_world[parent[i]] · pose_in_parent[i]
///
/// The Model is topologically ordered (`parent[i] < i`), so a single `for`
/// from 0 to njoints() suffices — by the time we hit joint `i`, its parent's
/// world pose is already filled in.
#ifndef TINYSPATIAL_ALGO_FORWARD_KINEMATICS_HPP
#define TINYSPATIAL_ALGO_FORWARD_KINEMATICS_HPP

#include "tinyspatial/core/types.hpp"
#include "tinyspatial/model/joint.hpp"
#include "tinyspatial/model/model.hpp"

namespace tinyspatial {

/// Fill `data.pose_in_parent` and `data.pose_in_world` for every joint.
/// \pre `q.size() == model.nq()`. `data` is sized to `model.njoints()`.
inline void forward_kinematics(const Model& model, Data& data, const Eigen::Ref<const VectorX>& q) {
  for (int i = 0; i < model.njoints(); ++i) {
    const Joint& j = model.joints[i];
    const int joint_nq = nq(j);
    // `q.segment(...)` is a zero-copy Block view; `Ref<const VectorX>` binds
    // to it directly. Using `const VectorX q_slice = ...` here would heap-
    // allocate per joint per FK call, which is the dominant cost on small
    // arms.
    const SE3 local = model.placement[i] * joint_transform(j, q.segment(model.idx_q[i], joint_nq));
    data.pose_in_parent[i] = local;
    const int parent_idx = model.parent[i];
    data.pose_in_world[i] = (parent_idx == -1) ? local : data.pose_in_world[parent_idx] * local;
  }
}

}  // namespace tinyspatial

#endif  // TINYSPATIAL_ALGO_FORWARD_KINEMATICS_HPP
