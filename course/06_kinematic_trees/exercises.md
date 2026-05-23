# Exercises — Chapter 06

These wire your head into the array-of-joints data layout. Most are short C++
snippets; you can run them by adapting
[`src/examples/se3_basics.cpp`](../../src/examples/se3_basics.cpp) or by writing
a quick test under `tests/`.

## 1. Predict the indices

Build (on paper) a 3-joint serial arm: a floating base joint, then two
revolute joints in a chain.

(a) What does `Model::nq()` return? `Model::nv()`?
(b) What are `idx_q` and `idx_v` for each joint?
(c) What's `parent[2]`?

<details><summary>Hint</summary>
Floating contributes 7 to nq and 6 to nv. The next two joints contribute
1 each to both. The chain is serial, so parent[1] = 0 and parent[2] = 1.
</details>

## 2. Read a real model

Adapt the `simple_arm` test to instead load `ur5e.urdf`. Print:

- the number of joints,
- the parent of every joint,
- the placement of joint 2 (`m.placement[2].translation()`).

(The fixture URDF lives at `data/robots/ur5e.urdf`. Use
`build_model_from_urdf_file()` from `tinyspatial::urdf`.)

## 3. Compose by hand

For `simple_arm` at `q = (0, 0)`, work out the pose of `link2`'s frame in the
world by chaining `placement[0] * joint_transform(joints[0], 0)` and so on.
Confirm the translation lands at `(1, 0, 0)`.

## 4. Joint variants

Without running the code, predict which joint type comes out for each URDF
fragment. (Then check by loading.)

```xml
<joint name="j" type="continuous"> ... </joint>
<joint name="j" type="revolute">   ... </joint>
<joint name="j" type="fixed">      ... </joint>
<joint name="j" type="floating">   ... </joint>
```

<details><summary>Answer</summary>
continuous → `JointRevolute` (we collapse continuous to revolute).
revolute → `JointRevolute`.
fixed → `JointFixed`.
floating → `JointFloating`.
</details>

## 5. Fixed joints contribute 0 DOF

In `so_arm101.urdf`, the gripper is attached via a `<joint type="fixed">`.
Why does `Model::nq()` come back as 5 and not 6? Look at
`tests/unit/urdf/test_urdf_loader.cpp::SoArm101HasMixedJoints` and explain in
one sentence.

---

Solutions are not committed here yet — verify against the library and the
tests named in chapters 06–07.
