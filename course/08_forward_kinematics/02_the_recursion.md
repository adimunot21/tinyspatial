# The recursion

Forward kinematics is fundamentally one idea applied recursively to every
joint in the tree:

> **Child's world pose = parent's world pose · child's pose in parent.**

Read it as: to know where the child is in the world, start at the parent (a
problem you already solved), then walk over by whatever the joint between
them is currently doing.

## Two pieces per joint

For joint `i`, the *child's pose in parent's frame* has two parts:

1. **The static placement** `Model::placement[i]`. This is the joint's
   resting position relative to its parent — it never depends on `q`. It's
   what the `<origin>` of the URDF joint tag gave us. If you froze every
   motor, this is what the kinematics would look like.

2. **The joint's motion** `joint_transform(joints[i], q_slice(i))`. A
   revolute joint at angle 1.5 rad becomes a rotation; a prismatic joint at
   displacement 0.2 m becomes a translation; a fixed joint becomes the
   identity. This is `q`-dependent, hence the slice of the global `q`.

The two compose:

$$
T_{\text{parent} \to \text{child}}(q) \;=\; \mathrm{placement}_i \;\cdot\; T_{\text{joint}_i}(q)
$$

and *that* is what gets multiplied into the parent's world pose to give the
child's world pose. In the code we store it as `Data::pose_in_parent[i]`.

## Why "topological order" matters

When you compute joint `i`, you assume joint `parent[i]`'s world pose is
already known. That's only true if joints are listed in an order where every
joint's parent comes before it. The `Model` invariant from chapter 06 —
`parent[i] < i` — guarantees this. So the algorithm becomes:

```
for i in 0 .. njoints - 1:
  local         = placement[i] · joint_transform(joints[i], q_slice)
  pose_in_parent[i] = local
  pose_in_world[i]  = (parent[i] == -1) ? local : pose_in_world[parent[i]] · local
```

A single forward pass. No recursion stack, no fixpoint loop. Topological
order turned what could be a graph-walking algorithm into a `for` loop.

## A worked walk

`simple_arm` at `q = (π/2, 0)`:

| i | joint              | placement   | joint_transform | local                   | parent | world                                |
| - | ------------------ | ----------- | --------------- | ----------------------- | ------ | ------------------------------------ |
| 0 | joint1 (revolute z)| identity    | `R_z(π/2)`      | `R_z(π/2)`              | −1     | `R_z(π/2)`                           |
| 1 | joint2 (revolute z)| `T(+x,1)`   | identity        | `T(+x,1)`               | 0      | `R_z(π/2) · T(+x,1)` = `T(+y,1)`     |

That `T(+y, 1)` is the test
`ForwardKinematics.HandComputedSimpleArm` — joint 2 lands one metre along
+y in the world after joint 1 rotates 90°. The product of two `SE(3)`s
rotated and translated the second link's frame exactly there.

## Where this lives in the library

| Concept | File · symbol |
| ------- | ------------- |
| Topological invariant | [`model.hpp`](../../include/tinyspatial/model/model.hpp) · `Model::parent` |
| Placement (URDF origin) | `Model::placement[i]` |
| Joint motion | [`joint.hpp`](../../include/tinyspatial/model/joint.hpp) · `joint_transform()` |
| The recursion in code | [`forward_kinematics.hpp`](../../include/tinyspatial/algo/forward_kinematics.hpp) |

Next: [Walking the tree in code](03_in_code.md).
