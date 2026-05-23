# How Featherstone thinks

You now have every primitive Featherstone's dynamics algorithms use. This last
chapter is a mental model — the *language* in which the next five chapters are
going to talk.

## A robot is a chain of frames

Imagine the arm laid out as a tree of frames, one per link, each connected to
its parent by a joint. At each frame lives:

- a **spatial inertia** $I$ — the local body's mass distribution, expressed at
  the body-frame origin;
- a **twist** $v$ — the body's spatial velocity in the body frame;
- an **acceleration** $a$ — its spatial acceleration in the body frame;
- a **wrench** $f$ — net force on the body, in the body frame.

Featherstone's insight: do *all* the algebra **per link, in that link's body
frame.** Don't try to express everything in the world frame. Instead, when you
need to move a quantity between adjacent frames — parent ↔ child — apply a
Plücker. When you need to relate a velocity to a momentum, multiply by the
local inertia. The algorithms are then a small, fixed pattern.

## The recurring move: "outward velocities, inward forces"

Three Featherstone algorithms — RNEA (inverse dynamics), ABA (forward dynamics),
CRBA (mass matrix) — all share a structure:

1. **Outward sweep (base → leaves).** Use joint velocities to push spatial
   *velocity* outward along the chain. Each child's velocity is the
   *Plücker-transported* parent velocity plus the joint's velocity contribution.
2. **Compute per-link kinematics.** Local accelerations, momenta, bias forces.
   This is where $I \cdot v$ and the Lie-bracket cross products show up.
3. **Inward sweep (leaves → base).** Use spatial *forces* the other direction.
   The wrench on a parent is the *force-Plücker-transported* wrench on its
   child plus whatever the parent itself contributes. Joint torques fall out as
   projections.

Outward = motion-flavoured = `motion_plucker`. Inward = force-flavoured =
`force_plucker`. The two sweeps are duals of each other, which is exactly the
duality `Motion`/`Force` make explicit in code.

## Why this matters now

You won't see RNEA / ABA / CRBA until chapters 10–11. But the building blocks
are *in your hands*:

```cpp
Motion v_parent  = ... ;
Motion v_child   = T_child_to_parent.inverse() * v_parent
                 + joint_axis * joint_velocity;     // outward velocity
Force  f_child   = inertia_child * a_child
                 + cross(v_child, inertia_child * v_child);  // bias
Force  f_at_parent = T_child_to_parent * f_child;  // inward force (dual Plücker)
```

Read those three lines slowly. They contain the entire Featherstone vocabulary:
Plücker transports for motion (outward) and force (inward), inertia mapping
velocity to momentum (`Force = SpatialInertia * Motion`), and the velocity-
product correction (`cross(Motion, Force) -> Force`). Phase 2 has given you the
words; phases 4–5 will form the sentences.

## Check your understanding

1. A child link's velocity in its own frame is given the parent's velocity in
   the parent's frame and the joint between them. Which Plücker transports the
   parent's velocity into the child's frame?
2. The wrench on a parent from a child needs to be re-expressed in the parent
   frame. Which Plücker?
3. Why is `cross(Motion, Force)` defined to return a `Force`, while
   `cross(Motion, Motion)` returns a `Motion`?

(Answers: the motion Plücker $\mathrm{Ad}_{T_{\text{parent} \to \text{child}}}$;
the force Plücker $\mathrm{Ad}_T^{-\top}$ of the child→parent transform; because
the velocity-product term arises from differentiating a *momentum* over time,
and momentum is a force-like quantity — so the result must be a wrench.)

## Where this lives in the library

This chapter is synthesis; every primitive named above lives in
[`include/tinyspatial/spatial/`](../../include/tinyspatial/spatial). The
algorithms that *use* them arrive in chapters 08–11.

Next: the [exercises](exercises.md), then on to
[chapter 06 — Kinematic trees](../06_kinematic_trees/README.md).
