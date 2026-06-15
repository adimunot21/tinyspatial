# Computing it: the screw-axis walk

The algorithm is a short walk from the target link `L` *up* to the root,
filling in one column per joint along the way. The LOCAL-frame
version, lightly re-formatted from `jacobian.hpp`:

```cpp
j_out.setZero();
const SE3 pose_world_L_inv = data.pose_in_world[link_id].inverse();
for (int i = link_id; i != -1; i = model.parent[i]) {
  const SE3 T_link_i = pose_world_L_inv * data.pose_in_world[i];
  const Matrix6 ad_link_from_i = T_link_i.adjoint();
  const int col_start = model.idx_v[i];
  // (per joint variant): place the screw axis into J's columns.
}
// then optionally multiply by an outer adjoint to convert to WORLD / LWA.
```

Three things are happening.

## 1. Only ancestors contribute

`for (int i = link_id; i != -1; i = model.parent[i])` walks from L up to
the root through `Model::parent[]`. Every joint we visit is an *ancestor*
of L; moving it moves L. Joints *not* on this path — siblings or descendants
of L — don't contribute to L's velocity, so their Jacobian columns stay
zero. (`JacobianShape.IsZeroOnNonAncestorJoints` tests this on the
simple_arm.)

## 2. Each ancestor places its screw axis

For each ancestor `i`, we need its screw axis (in `i`'s own frame) expressed
in `L`'s frame. That's a single adjoint:

$$
S_{\text{in } L} \;=\; \mathrm{Ad}_{T_{L, i}} \cdot S_{\text{in } i}.
$$

The chain-of-transforms tells us $T_{L, i} = T_{L, \text{world}} \cdot
T_{\text{world}, i}$ — and that's a one-line FK lookup.

The screw axis itself depends on the joint type, dispatched with `std::visit`:

| Joint type | Screw axis(es) in i's frame |
| ---------- | --------------------------- |
| `JointRevolute` | `(axis, 0, 0, 0)` (axis in angular block) |
| `JointPrismatic` | `(0, 0, 0, axis)` (axis in linear block) |
| `JointFloating` | the 6 unit twists — 6 columns, identity in i's frame |
| `JointFixed` | nothing |

## 3. One frame conversion at the end

After the walk fills `j_out` in the LOCAL frame, the function applies one
6×6 multiplication to convert to WORLD or LOCAL_WORLD_ALIGNED. Per-column
conversion would do the same work `n_v` times; the one-shot multiply is
strictly cheaper.

## Cost analysis

The walk visits up to `depth` joints, where `depth ≤ njoints`. Each step
does:

- one SE(3) multiplication + inverse: ~30 flops,
- one 6×6 adjoint formation + 6×k matrix-vector multiply: ~30k flops where k = nv contribution.

So the total is `O(depth · nv_per_joint)`, which for serial arms is
`O(N^2)` — the well-known cost of the naive Jacobian. Pinocchio is cleverer
(it caches per-joint partial transforms during FK and reuses them); we'll
revisit this in Phase 9. For Phase 4 the naive form is plenty fast.

## A trap to avoid

A tempting shortcut writes the LWA Jacobian by computing the LOCAL
Jacobian and then "rotating the angular and linear parts independently."
That is *only* valid because the LWA frame shares the link's origin. For
the WORLD Jacobian, the off-diagonal `[t]× R` block of the adjoint couples
angular into linear: computing the WORLD form column-by-column or
rotating naively silently drops the coupling, and the linear rows
come out wrong by exactly the velocity-of-the-link's-rotation term. The
parity test catches this immediately, but the construction is better
avoided in the first place.

## Where this lives in the library

| Concept | File · symbol |
| ------- | ------------- |
| Algorithm | [`jacobian.hpp`](../../include/tinyspatial/algo/jacobian.hpp) · `compute_jacobian()` |
| Per-joint screw axis dispatch | `std::visit` over `Joint` |
| Walk-up loop | `for (int i = link_id; i != -1; i = model.parent[i])` |
| Final frame conversion | `switch (frame)` at the bottom |
| Validating each frame | [`test_jacobian.cpp`](../../tests/unit/algo/test_jacobian.cpp), [`docs/PINOCCHIO_PARITY.md`](../../docs/PINOCCHIO_PARITY.md) |

Next: [Singularities](05_singularities.md).
