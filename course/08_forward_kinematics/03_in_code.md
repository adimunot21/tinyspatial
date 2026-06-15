# Walking the tree in code

The whole forward-kinematics function is one short loop. Here it is verbatim from
[`forward_kinematics.hpp`](../../include/tinyspatial/algo/forward_kinematics.hpp):

```cpp
template <typename S>
void forward_kinematics(const ModelT<S>& model, DataT<S>& data,
                        const Eigen::Ref<const typename Types<S>::VectorX>& q) {
  for (int i = 0; i < model.njoints(); ++i) {
    const JointT<S>& j = model.joints[i];
    const int joint_nq = nq(j);
    // `q.segment(...)` is a zero-copy Block view; `Ref<const VectorX>` binds
    // to it directly. Using `const VectorX q_slice = ...` here would heap-
    // allocate per joint per FK call, which is the dominant cost on small
    // arms.
    const SE3T<S> local =
        model.placement[i] * joint_transform(j, q.segment(model.idx_q[i], joint_nq));
    data.pose_in_parent[i] = local;
    const int parent_idx = model.parent[i];
    data.pose_in_world[i] = (parent_idx == -1) ? local : data.pose_in_world[parent_idx] * local;
  }
}
```

The function is templated on the scalar `S`. Instantiated on `double` (the alias
`forward_kinematics(const Model&, Data&, q)`) it is ordinary FK; instantiated on
the autodiff scalar `Jet` it returns exact derivatives (Chapter 13). Read the body
through the `double` lens: `ModelT<S>` is `Model`, `SE3T<S>` is `SE3`.

## Why each piece is the way it is

- **`const Eigen::Ref<const ...VectorX>& q`** — Eigen's reference type. Accepts a
  `VectorX`, a slice of one, or a `Map` of a raw buffer, without copying.
- **`q.segment(model.idx_q[i], joint_nq)`** — slices out this joint's
  configuration coordinates (1 for a revolute joint, 7 for a floating one) and
  passes the slice *directly* into `joint_transform`. The earlier version of this
  code bound the slice to a named `const VectorX q_slice`, which heap-allocates
  once per joint per call; the segment view eliminates that allocation, the single
  largest FK cost on small arms.
- **`placement[i] * joint_transform(j, ...)`** — composition order matters: the
  fixed placement applies first, then the joint moves *from* that placement.
- **`pose_in_world[parent[i]] * local`** — chains the cumulative world pose,
  parent-on-left, local-on-right, because `SE(3) * SE(3)` applies the right factor
  first (the Chapter 03 convention).
- **`parent_idx == -1` is the root case.** A root joint has no parent pose to
  compose with, so its world pose equals `local`.

## Cost

A single pass over `njoints` joints, each iteration a constant amount of work (a
few SE(3) products, one `q.segment()` view, one `joint_transform`). FK is therefore
$O(N)$ in the number of joints, with no allocation in the loop — every output is
written into pre-sized `Data` buffers, and the zero-copy slice above is what keeps
that promise.

On a Franka FR3 this loop takes around a microsecond on a modern x86 core in
Release. The dynamics algorithms in chapter 10 will run this same loop as
their *first step* hundreds of times per second.

## An easily missed convention

`pose_in_parent[i]` stores `placement[i] * joint_transform(...)` — the
*combined* parent-frame contribution, not just the URDF placement. We don't
store the placement and joint pieces separately, because every downstream
algorithm wants the combined one.

The bare URDF placement remains available in `Model::placement[i]` (constant).
The bare joint motion can be recomputed with
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
