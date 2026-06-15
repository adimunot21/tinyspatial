# Links and joints, in code

A **link** is a rigid body — a piece of metal that doesn't bend. A **joint**
is the constrained motion between two adjacent links. Every algorithm in the
library is a recipe for walking links and joints and doing a small piece of
arithmetic at each.

## What a link stores

Surprisingly little. From a *kinematics* point of view, the link is just an
anchor point — the joint frames are what carry pose. From a *dynamics* point
of view, the link has a mass distribution: a mass, a centre of mass, and an
inertia tensor. That's exactly the `SpatialInertia` from chapter 05.

In `tinyspatial`, the link information is stored alongside its parent joint
in the `Model`:

- `Model::link_names[i]` — the URDF name of the body attached on the *child*
  side of joint `i`.
- `Model::inertia[i]` — the spatial inertia of that body, expressed in its
  own body frame.

There's no separate "Link" class. Every link in a tree is associated with
exactly one inbound joint (the joint that connects it to its parent), so the
link data lives in the same parallel arrays as the joint data.

> **Why no Link class?** Every link is uniquely identified by its inbound joint
> index, and algorithms iterate over joints — never over "free" links. A Link
> wrapper would buy nothing and force two indirections.

## What a joint stores

The joint stores its *geometric data* — the parameters that determine how
it constrains motion — and nothing else:

- **Type**: revolute (1 DOF rotation), prismatic (1 DOF translation), fixed
  (0 DOF), or floating (6 DOF). One of four small structs (chapter 03).
- **Axis** (for revolute and prismatic): a unit 3-vector in the joint's own
  frame. The chosen DOF moves about (revolute) or along (prismatic) this
  axis.

In addition, the `Model` stores a **placement** per joint — an SE(3) that
positions the joint frame in its parent link's frame. This is the joint's
"resting pose", read straight out of the URDF `<origin>` tag.

The full child-frame-in-parent transform at a given configuration `q` is

```
T_parent_to_child  =  placement  *  joint_transform(joint, q_slice)
```

where `q_slice` is the joint's piece of the global configuration vector.
`joint_transform` is the variant-dispatched function in `joint.hpp`:
revolute returns `exp(axis * q)`, prismatic returns a pure translation,
fixed returns the identity, floating reads 7 numbers (translation +
quaternion) and builds an SE(3).

## Why separating placement from joint_transform matters

The placement is *fixed* in the URDF and the joint variable is *what changes*.
Separating them allows the library to:

- store the placement once and not recompute it,
- swap the joint type without re-deriving the placement,
- reason about kinematics at `q = 0` (placements only) and at general `q`
  (placements plus per-joint transforms) using the same machinery.

Forward kinematics (chapter 08) is essentially "compose `placement *
joint_transform` along the chain." The split is what makes that one line.

## Where this lives in the library

| Concept | File · symbol |
| ------- | ------------- |
| Joint variant types | [`joint.hpp`](../../include/tinyspatial/model/joint.hpp) · `JointRevolute`, `JointPrismatic`, `JointFixed`, `JointFloating` |
| The joint-transform free function | [`joint.hpp`](../../include/tinyspatial/model/joint.hpp) · `joint_transform()` |
| Joint placements + inertias arrays | [`model.hpp`](../../include/tinyspatial/model/model.hpp) · `Model::placement`, `Model::inertia` |

Next: [The tree](02_the_tree.md).
