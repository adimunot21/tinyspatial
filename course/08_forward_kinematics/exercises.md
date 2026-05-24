# Exercises — Chapter 08

These exercises walk you through FK by hand and at the C++ level.

## 1. Two joints, by hand

For `simple_arm` at `q = (0, π/2)`:

(a) Write `joint_transform(joints[0], 0)` and `joint_transform(joints[1], π/2)`
as SE(3)s.
(b) Compute `pose_in_world[0]` and `pose_in_world[1]` by hand using the
recursion.
(c) Predict the world-frame translation of `pose_in_world[1]`.

<details><summary>Hint</summary>
joint1 doesn't move (`q=0` → identity rotation). joint2 rotates about z by π/2
at its placement of (+1, 0, 0). The link 2 frame is at the link 1 frame, then
translated +1 along link 1's x, then rotated. Position should still be (1, 0, 0)
because the rotation about z at that point keeps the origin fixed in xy.
</details>

## 2. Why does FK only need one pass?

Topological order means `parent[i] < i`. Why does that turn FK from a graph
traversal into a single `for` loop? Could you have done it with a tree-of-
pointers data structure? What would that cost you?

## 3. Add a third joint to simple_arm

Modify `simple_arm.urdf` to add a third revolute joint at the end of link 2.
Load it from C++ and verify:

(a) `model.njoints()` is 3.
(b) `parent[2]` is 1.
(c) `pose_in_world[2]` at `q = 0` lies at (2, 0, 0).

(See `tests/unit/algo/test_forward_kinematics.cpp` for the test pattern.)

## 4. FK as a self-check of the URDF

For each fixture URDF in `data/robots/`, run FK at `q = 0` and print the
world position of the end-effector. Compare the values against the
"cumulative reach" you'd expect from the link lengths in the URDF. Use this
to sanity-check that your synthetic URDFs match your intent.

(For `so_arm101`, the cumulative reach at `q = 0` is 0.462 m, as exercise 4 of
chapter 07 already established.)

## 5. The role of pose_in_parent

For one configuration `q` on `franka_fr3`, verify by hand that
`pose_in_world[i] == pose_in_world[parent[i]] * pose_in_parent[i]` for at
least three joints. Then explain what would go wrong if `Data` only stored
`pose_in_world` (no `pose_in_parent`).

(Hint: many downstream algorithms — RNEA, ABA, the Jacobian — propagate
*relative* quantities along the chain. They need the local-to-parent
transforms ready, not just the cumulative ones.)

---

Solutions are not committed. Use the parity table as your reference.
