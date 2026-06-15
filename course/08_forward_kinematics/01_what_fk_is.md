# What forward kinematics is

**Forward kinematics** (FK) is the function

$$
\mathbf{x} \;=\; \mathrm{FK}(q),
$$

that maps a configuration vector $q \in \mathbb{R}^{nq}$ to the world pose of
every link. The "x" is plural — FK fills in $n$ poses, one per joint frame, in
a single call.

## What FK is *not*

FK is easily confused with two other things it is not:

- **It's not** *inverse* kinematics. IK is the much harder problem
  "given a target pose, find a `q` that produces it" — there might be zero,
  one, or infinitely many solutions, and it requires iterative algorithms.
  FK has *exactly one* answer and is one matrix product per joint.
- **It's not** the *Jacobian*. The Jacobian is the *derivative* of FK with
  respect to `q`, covered in chapter 09. FK alone is just a snapshot.

FK is purely geometry. No mass, no forces, no time. Just composition of rigid
transforms along a tree.

## Why FK is the foundation of everything else

Every other algorithm in `tinyspatial` builds on FK:

- The **Jacobian** is computed from the link poses that FK fills in
  (chapter 09).
- **Inverse dynamics** (RNEA, chapter 10) needs link velocities and
  accelerations, which start with the link poses and propagate using FK's
  outputs.
- **Inverse kinematics** (chapter 12) is iterative FK + a Jacobian step:
  evaluate FK, compute the error, descend via the Jacobian, repeat.

So while FK looks trivial, getting it right (and fast) sets the ceiling on
everything downstream. The Pinocchio cross-check in chapter 04 shows agreement
with the reference library to about machine precision — the foundation the next
four chapters are built on.

## The one-paragraph summary

For each joint `i` in topological order, the algorithm computes

```
local[i]  =  Model::placement[i] · joint_transform(joints[i], q-slice)
world[i]  =  parent == -1  ?  local[i]
                            :  world[parent[i]] · local[i]
```

That's it. The next chapter unpacks each piece.

## Where this lives in the library

| Concept | File · symbol |
| ------- | ------------- |
| FK function | [`forward_kinematics.hpp`](../../include/tinyspatial/algo/forward_kinematics.hpp) · `forward_kinematics()` |
| Output buffers | [`model.hpp`](../../include/tinyspatial/model/model.hpp) · `Data::pose_in_parent`, `Data::pose_in_world` |

Next: [The recursion](02_the_recursion.md).
