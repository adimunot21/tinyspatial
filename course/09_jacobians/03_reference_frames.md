# Reference frames

A twist is only meaningful relative to a frame. "The end-effector moves at
1 m/s along x" *in whose x?* So the Jacobian — which produces twists — comes
in three flavours, one per reference frame. `compute_jacobian` takes the
frame as a parameter; the choice matters downstream.

## The three flavours

Pinocchio names them this way, and this library follows exactly:

| Frame | What its rows mean |
| ----- | ------------------ |
| `LOCAL` | Angular velocity & linear velocity expressed in **the link's own body frame**. |
| `WORLD` | Twist of the link expressed in the **world frame**. |
| `LOCAL_WORLD_ALIGNED` ("LWA") | Translation at the link's origin but rotation axes aligned with the world. |

The LWA frame is the most useful one for *humans* and most IK / control
code. The local one is the most natural for *dynamics* (Featherstone's
algorithms speak local-frame, chapter 10). World is occasionally useful
for visualisation but less often the right object than LWA.

## How they relate

`LOCAL` is the primitive. The other two come from `LOCAL` by left-multiplying
with an adjoint-like matrix (chapter 04):

- **`WORLD = Ad_{T_{world,link}} · LOCAL`** — full SE(3) adjoint of the link's
  world pose.
- **`LOCAL_WORLD_ALIGNED = diag(R, R) · LOCAL`** — just the rotation block, no
  translation coupling, where `R` is the link's world rotation.

The library does exactly this: compute the LOCAL Jacobian by walking up the
chain, then apply the final multiplication based on `frame`.

```cpp
switch (frame) {
  case JacobianFrame::kLocal:              /* keep as is */                  break;
  case JacobianFrame::kWorld:               j_out = T_world.adjoint() * j_out;  break;
  case JacobianFrame::kLocalWorldAligned: { /* multiply by diag(R, R)     */  ... }
}
```

The arithmetic-rule version is `LWA = Ad of (R, 0) · LOCAL` — the LWA frame
shares its origin with the link, so the translation part of the adjoint
vanishes.

## Picking the right frame

A small decision table:

| Question | Frame |
| -------- | ----- |
| "Where would the end-effector go if I pushed each joint at unit velocity, in the gripper's own coords?" | `LOCAL` |
| "What's the velocity of the gripper as I see it from my robot operator console?" | `LOCAL_WORLD_ALIGNED` |
| "Composing this with another Plücker that is in the world frame." | `WORLD` |
| "Writing IK code." | `LOCAL_WORLD_ALIGNED` (almost always) |

The IK chapter (12) covers why the LWA frame plays best with most
control conventions.

## Order matters

A subtle one: the `compute_jacobian` function fills the local form first,
then applies the frame transform *once* at the end. It does not transform
column-by-column. That's `O(N)` work to walk the chain plus a single 6×6
matrix multiply, instead of an $O(N)$ Plücker per column. Small but real
performance win.

## Where this lives in the library

| Concept | File · symbol |
| ------- | ------------- |
| The enum | [`jacobian.hpp`](../../include/tinyspatial/algo/jacobian.hpp) · `enum class JacobianFrame` |
| Frame conversion in code | `compute_jacobian()` final `switch` |
| LWA relation test | [`test_jacobian.cpp`](../../tests/unit/algo/test_jacobian.cpp) · `LocalWorldAlignedMatchesRotatedLocal` |
| Tested against Pinocchio | [`docs/PINOCCHIO_PARITY.md`](../../docs/PINOCCHIO_PARITY.md) — three columns |

Next: [Computing it: the screw-axis walk](04_in_code.md).
