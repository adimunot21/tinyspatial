# Walking the tree in code

The whole forward-kinematics function is one short loop. Here it is, lightly
re-formatted from `forward_kinematics.hpp`:

```cpp
inline void forward_kinematics(const Model& model, Data& data,
                               const Eigen::Ref<const VectorX>& q) {
  for (int i = 0; i < model.njoints(); ++i) {
    const Joint& j = model.joints[i];
    const int joint_nq = nq(j);
    const VectorX q_slice = q.segment(model.idx_q[i], joint_nq);
    const SE3 local = model.placement[i] * joint_transform(j, q_slice);
    data.pose_in_parent[i] = local;
    const int parent_idx = model.parent[i];
    data.pose_in_world[i] =
        (parent_idx == -1) ? local : data.pose_in_world[parent_idx] * local;
  }
}
```

Eleven lines. Read each one against chapter 06–07.

## Why each piece is the way it is

- **`const Eigen::Ref<const VectorX>& q`** — Eigen's reference type. Accepts
  `VectorX`, a slice of one, or a `Map` of a raw buffer, without copying.
- **`q_slice = q.segment(idx_q[i], joint_nq)`** — pulls out just this joint's
  configuration coordinates. For a revolute joint that's 1 number; for a
  floating joint, 7.
- **`placement[i] * joint_transform(j, q_slice)`** — composition order
  matters: the placement comes first, then the joint moves *from* that
  placement. This is "go to the joint frame, then rotate/translate."
- **`pose_in_world[parent[i]] * local`** — chains the cumulative world pose.
  Note the multiply order: parent-on-left, local-on-right, because
  `SE(3) * SE(3)` composes "apply right first, then left" (the chapter 03
  convention).
- **`parent_idx == -1` is the root case.** Root joints have no parent's pose
  to multiply with; their `pose_in_world == local`.

## Cost

A single pass over `njoints` joints. Each iteration is a constant amount of
work (a few SE(3) multiplies, one `q.segment()` view, one `joint_transform`
call). So FK is `O(N)` where `N` is the number of joints. No allocations in
the loop — every output is written into pre-sized `Data` buffers.

On a Franka FR3 this loop takes around a microsecond on a modern x86 core in
Release. The dynamics algorithms in chapter 10 will run this same loop as
their *first step* hundreds of times per second.

## A reused convention you may have missed

`pose_in_parent[i]` stores `placement[i] * joint_transform(...)` — the
*combined* parent-frame contribution, not just the URDF placement. We don't
store the placement and joint pieces separately, because every downstream
algorithm wants the combined one.

If you ever need just the URDF placement, it's still in `Model::placement[i]`
(constant). If you ever need just the joint motion, recompute it with
`joint_transform(joints[i], q.segment(idx_q[i], nq(joints[i])))`. The Data
holds the *useful product*.

## Where this lives in the library

| Concept | File · symbol |
| ------- | ------------- |
| The FK function | [`forward_kinematics.hpp`](../../include/tinyspatial/algo/forward_kinematics.hpp) |
| Slicing `q` per joint | `q.segment(model.idx_q[i], nq(j))` |
| Output buffers | [`model.hpp`](../../include/tinyspatial/model/model.hpp) · `Data::pose_in_parent`, `Data::pose_in_world` |
| Tests | [`test_forward_kinematics.cpp`](../../tests/unit/algo/test_forward_kinematics.cpp) |

Next: [Validating against Pinocchio](04_validation.md).
