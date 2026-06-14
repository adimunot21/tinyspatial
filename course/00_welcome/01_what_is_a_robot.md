# The robot model

"Robot" is a broad word. This course uses a precise, narrow meaning: a robot is
a **kinematic chain** — a sequence of rigid bodies connected by joints, anchored
to a fixed base.

## Links and joints

The model has two primitives:

- **Links** — rigid bodies. They neither bend nor stretch. (Physical metal flexes
  slightly; that deflection is outside the rigid-body model and is ignored.) Each
  link carries a mass, a centre of mass, and a rotational inertia.
- **Joints** — the constrained connections between links. The two that dominate
  robot arms are the **revolute** joint, which rotates about a fixed axis (an
  elbow), and the **prismatic** joint, which translates along a fixed axis (a
  drawer slide). The library also supports `fixed` and `floating` joints.

A chain of links connected by joints, with one end fixed to the **base**, has a
free end — the **end-effector** — where a gripper or tool is mounted.

```
   base                                     end-effector
    ▇═══════[J1]═══════[J2]═══════[J3]═══════▶ (gripper)
            link 1     link 2     link 3
```

## The three questions

Almost every quantity of interest about such a chain falls into three problems.
The course develops each in turn.

### 1. Forward kinematics — configuration to pose

Given the joint values (the **configuration** $q$), where is each link, and in
particular where is the end-effector, and in what orientation? This is pure
geometry: no forces, no time. It is the subject of Chapter 08.

### 2. The Jacobian — joint rates to spatial velocity

Given the configuration and the joint velocities $\dot q$, what is the resulting
spatial velocity of the end-effector? The linear map from $\dot q$ to that
velocity is the **Jacobian** $J(q)$ — the bridge between joint space and task
space, and the object that exposes **singularities** (configurations where the
arm loses the ability to move in some direction). Chapter 09.

### 3. Dynamics — forces and motion

Introduce mass, inertia, and gravity. **Inverse dynamics** computes the joint
torques required to produce a desired acceleration (the RNEA algorithm,
Chapter 10); **forward dynamics** computes the acceleration produced by given
torques (ABA, Chapter 11). These are the computational core of the library.

## Why a library is warranted

A single joint can be handled by hand. A 6- or 7-joint arm cannot: each link's
transform composes with every transform before it, and the bookkeeping grows
quickly. Compounding the difficulty, the field admits several equally valid
**conventions** — distinct ways to parameterise a rotation or compose a transform
— and silently mixing them produces wrong answers that remain numerically
plausible. The library fixes one set of conventions, applies them mechanically,
and validates the result against an independent reference. Those convention
choices are stated explicitly wherever they matter.

## "Spatial"

The term **spatial vector** has a precise meaning here: a 6-vector that bundles a
rotational part (3 components) with a linear part (3 components). Velocity, force,
and acceleration are each represented this way. This packing is what makes the
dynamics equations compact, and it gives the library its name. Chapter 05
develops the spatial algebra in full.

Next: [Build and test the library](02_setup_your_machine.md)

---

### Where this lives in the library

The data structures for links and joints are introduced in Chapter 06; this
chapter establishes only the model. For reference:

| Concept | Where it lives |
| ------- | -------------- |
| Joint types (revolute, prismatic, fixed, floating) | [`model/joint.hpp`](../../include/tinyspatial/model/joint.hpp) · `JointRevoluteT`, `JointPrismaticT` |
| The robot as a tree of links | [`model/model.hpp`](../../include/tinyspatial/model/model.hpp) · `ModelT` |
