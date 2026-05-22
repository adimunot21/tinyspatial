# Why robots need this

That was a lot of abstraction. Let's cash it in. Here is where each Lie-group
tool shows up in the rest of the library — a map of debts this chapter pays off.

## Forward kinematics (chapters 06, 08)

Each joint places one link relative to its parent by a rigid transform
$T \in SE(3)$. Finding where the hand is = composing the chain
$T_1 T_2 \cdots T_n$ — pure $SE(3)$ **composition**. A revolute joint at angle
$q$ contributes $\exp(q\,\hat{s})$ for its screw axis $\hat s$ — that's the
**exponential map** doing the joint's job.

## Jacobians and velocity (chapter 09)

"How fast does the hand move when the joints move?" maps joint velocities to a
spatial velocity (a twist) of the end-effector. Assembling that map column by
column uses the **adjoint** to carry each joint's contribution into a common
frame. Singularities — poses where the arm loses a direction of motion — are
where this map drops rank.

## Dynamics (chapters 10–11)

Featherstone's RNEA/ABA/CRBA push velocities and accelerations *outward* along
the chain and forces *inward*. Outward velocity propagation is **adjoint**
transport; the velocity-product (Coriolis) terms are **spatial cross products**;
inward force transport is the **force cross product** and the adjoint transpose.
Almost every line is a tool from this chapter.

## Inverse kinematics (chapters 12–13)

"What joint angles put the hand *there*?" is an optimisation *on the manifold*.
You measure the error as a `log` (the twist from where you are to where you want
to be), convert it to a joint correction with a **Jacobian** (and its inverse),
step, and repeat. Differentiable IK then differentiates *through* that whole
loop — which needs the **Jacobians of exp/log** we built here.

## The thread

Notice the pattern across all of these: **lift to the tangent space, do linear
algebra, map back with `exp`/`log`, and shuttle between frames with the
adjoint.** That sentence is the entire engineering value of Lie groups for
robotics. Everything downstream is a specialisation of it.

If chapters 01–05 felt like machinery without a purpose, this is the purpose. You
now have the complete vocabulary that the kinematics and dynamics chapters will
speak in.

## Check your understanding

1. Which Lie-group operation computes where a robot's hand is, given joint
   angles?
2. You measure the error between the current and desired end-effector pose as a
   6-vector. Which map produced it, and what space does it live in?
3. Why does propagating a velocity from a child link to its parent involve the
   adjoint rather than just a rotation?

(Answers: composition of $SE(3)$ transforms; the `log`, living in the tangent
space $\mathfrak{se}(3)$; because a velocity's linear part depends on the origin
it's measured about, and the translation between frames mixes angular velocity
into linear — the $[t]_\times R$ block of the adjoint.)

## Where this lives in the library

This chapter is a synthesis; the relevant code is everything in
[`liegroup/`](../../include/tinyspatial/liegroup) and
[`spatial/cross.hpp`](../../include/tinyspatial/spatial/cross.hpp). The chapters
named above will link to the algorithms that consume it.

Next: the [exercises](exercises.md), then onward to
[chapter 05 — Spatial algebra](../05_spatial_algebra/README.md).
