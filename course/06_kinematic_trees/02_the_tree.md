# The tree

A 6-DOF arm is a serial chain — base → link1 → link2 → … → end-effector — but
not every robot is. A humanoid forks at the torso into a head, two arms, and
two legs. A two-finger gripper is two parallel chains off a common palm. The
right abstraction is a **tree**: each node (link/joint) has exactly one
parent, but a parent can have any number of children.

## How we store the tree: parallel arrays

The textbook way to store a tree is with pointers — each node holds a
pointer-to-parent and a list-of-children. The library does not do that.
Instead, `Model` stores everything in **parallel arrays indexed by joint id**:

```cpp
class Model {
  std::vector<Joint>           joints;     // joints[i] is the i-th joint
  std::vector<int>             parent;     // parent[i] = index of i's parent
                                            //          (or -1 if i is a root)
  std::vector<SE3>             placement;  // placement[i] in parent[i]'s frame
  std::vector<SpatialInertia>  inertia;    // body i's spatial inertia
  // … and so on.
};
```

Three reasons to prefer arrays-of-ints over pointers:

1. **Cache locality.** The dynamics inner loop touches one field per joint
   along the whole chain; contiguous storage is twenty times faster than
   chasing pointers.
2. **Trivial serialization.** No graph-with-cycles puzzle when a Model is saved
   to a file.
3. **Algorithms become for-loops.** `for (int i = 1; i < n; ++i)` over a
   topologically-ordered array is the cleanest possible expression of "do
   something to every joint in outbound order."

The cost is one indirection: "joint i's parent's data" is `x[parent[i]]`. That
is a small price.

## Topological order

The library requires `parent[i] < i` for every joint `i`. Walking joints in
index order is therefore automatically a **breadth-first / depth-first**
outbound traversal: by the time joint `i` is processed, joint `parent[i]` is
already done. Going inbound (from leaves toward root) is the same loop run
backwards.

This invariant is established at construction time. `add_joint()` returns the
new joint's index and refuses any `parent_idx >= njoints()`, so the order
cannot be violated by accident. The URDF loader produces topologically ordered
joints by walking the link graph BFS-from-root.

## Roots and the world frame

Most arms have one root joint (the joint connecting the base to the world).
A floating-base humanoid has *one* root joint of type `JointFloating`.
A multi-armed system on a fixed base might have several root joints,
all with `parent == -1`. The library does not include a separate "universe"
joint; the world frame is the implicit parent of every root.

## Walking the tree

A canonical outbound traversal:

```cpp
for (int i = 0; i < model.njoints(); ++i) {
  const int parent_idx = model.parent[i];
  // …compute joint i's quantities given parent_idx's quantities…
}
```

That is the whole pattern. Pinocchio's algorithms are largely this loop with
one or two extra lines per iteration. The same shape (forward, then backward)
is the bones of RNEA, ABA, CRBA in chapters 10–11.

## Where this lives in the library

| Concept | File · symbol |
| ------- | ------------- |
| Joint topology arrays | [`model.hpp`](../../include/tinyspatial/model/model.hpp) · `Model::joints`, `parent`, `placement` |
| Topological-order builder | [`model.hpp`](../../include/tinyspatial/model/model.hpp) · `Model::add_joint()` |
| Tree test | [`test_model.cpp`](../../tests/unit/model/test_model.cpp) · `Model.ShapeAndIndexing` |

Next: [The joint variants](03_joint_variants.md).
