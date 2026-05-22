# What is a robot? (for our purposes)

The word "robot" covers everything from a Roomba to a Mars rover to a chatbot.
We mean something much more specific, so let's pin it down.

## A robot is a chain of rigid bodies connected by joints

Hold your arm out. Your upper arm is a fairly rigid segment. Your forearm is
another. They meet at the elbow — a **joint** — which lets the forearm rotate
relative to the upper arm. Your shoulder is another joint; your wrist, another.

A robot arm is exactly this idea, made of metal:

- **Links** — the rigid segments. They don't bend or stretch. (Real metal flexes
  a tiny bit; we ignore that. That's the "rigid" in *rigid-body dynamics*.)
- **Joints** — the connections that allow controlled relative motion. The most
  common is the **revolute** joint, which rotates about a single axis, like your
  elbow. A **prismatic** joint slides in a straight line, like a drawer.

Chain a few links together with joints and anchor one end to the ground (the
**base**), and the free end is usually where you bolt a gripper or a tool — we
call that the **end-effector**.

```
   base                                     end-effector
    ▇═══════[J1]═══════[J2]═══════[J3]═══════▶ (gripper)
            link 1     link 2     link 3
```

## The three questions this library answers

Once you have such a chain, almost everything you want to ask falls into three
buckets. The whole course is, in a sense, just these three questions asked with
increasing sophistication.

### 1. "Given the joint angles, where is everything?" — *forward kinematics*

If the elbow is bent 30° and the shoulder is rotated 45°, where exactly is the
hand in space, and which way is it pointing? This is **forward kinematics**, and
it's pure geometry — no forces, no time, just "if the joints are *here*, the
hand is *there*."

### 2. "How do joint speeds relate to hand speed?" — *the Jacobian*

Spin the shoulder motor at 1 radian per second. How fast, and in which
direction, does the hand move? The answer depends on the current pose, and the
object that captures it is called the **Jacobian**. It's the bridge between
"joint space" (motor angles) and "task space" (where the hand is). Jacobians are
also how a robot senses it's near a **singularity** — a pose where it suddenly
loses the ability to move in some direction.

### 3. "What forces produce what motion?" — *dynamics*

Now bring in mass and gravity. To hold the arm still against gravity, the motors
must apply specific torques. To *accelerate* the arm along a path, they must
apply more. Computing the torques for a desired motion is **inverse dynamics**
(the RNEA algorithm); computing the motion that results from given torques is
**forward dynamics** (the ABA algorithm). These are the heart of the library.

## Why is this hard enough to need a library?

Two links and one joint, you could do by hand. But a real arm has six or seven
joints, each rotation composes with all the ones before it, and the bookkeeping
explodes. Worse, there are many *conventions* — different valid ways to write
down a rotation or stack up a transform — and mixing them silently gives wrong
answers that look plausible. A good library picks one set of conventions, applies
them mechanically, and is tested to death. That's what we're building.

## A note on "spatial"

You'll see the word *spatial* everywhere here. In robotics it has a precise
meaning: a **spatial vector** is a 6-dimensional vector that bundles together a
rotational part (3 numbers) and a linear part (3 numbers). Velocity, force,
acceleration — all become 6-vectors. It turns out that packing them this way
makes the dynamics equations beautifully compact. That's where the *spatial* in
`tinyspatial` comes from, and Chapter 05 is devoted to it.

## Check your understanding

1. Name the two physical ingredients every robot arm in this course is made of.
2. Which of the three questions involves *no* forces or masses?
3. Your elbow is a ______ joint; a drawer slide is a ______ joint.

(Answers: links and joints; forward kinematics; revolute, prismatic.)

Next: [Set up your machine](02_setup_your_machine.md)

---

### Where this lives in the library

Nothing to point at yet — the data structures for links and joints arrive in
Chapter 06. For now, just hold the mental model: **links + joints = a robot**.

| Concept | Where it lives (arrives in a later phase) |
| ------- | ----------------------------------------- |
| Joint types (revolute, prismatic, …) | `include/tinyspatial/model/joint.hpp` *(Phase 3)* |
| The robot as a tree of links | `include/tinyspatial/model/model.hpp` *(Phase 3)* |
