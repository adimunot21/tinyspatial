# Exercises — Chapter 07

These are hands-on with the URDF loader. Each one is a short C++ snippet
you can adapt from [`se3_basics.cpp`](../../src/examples/se3_basics.cpp) or
the loader tests.

## 1. Inspect a URDF by hand

Open [`data/robots/simple_arm.urdf`](../../data/robots/simple_arm.urdf). Without
running anything, predict:

(a) How many `<link>` blocks?
(b) How many `<joint>` blocks?
(c) What does the loader return for `Model::njoints()` and `Model::nq()`?
(d) Which link is the root (never appears as a `<child>`)?

## 2. Add a joint

Modify `simple_arm.urdf` to add a third revolute link and joint. What's the
new `nq()`? What's `parent[2]`?

(Test it by writing a tiny program that loads it and prints these.)

## 3. Trigger every kind of error

Without looking at the loader source, predict whether each of these will
throw `UrdfParseError`. Then verify by running them through
`build_model_from_urdf()`.

(a) A URDF where `<joint>` has no `type` attribute.
(b) A URDF where a `<joint>`'s `<child>` references a link that isn't
defined.
(c) A URDF with a `<joint type="screw">`.
(d) A URDF where two `<joint>` elements share the same `name`.
(e) A `<robot>` element with no `<link>` and no `<joint>`.

<details><summary>Hints</summary>
(a), (b), (c) throw — required attributes, dangling child, unsupported type.
(d) currently does **not** throw — joint names aren't checked for uniqueness
(only link names are). Worth a small follow-up issue if you're so inclined.
(e) does *not* throw — a robot with one link and no joints is valid.
</details>

## 4. Read a real spec

Pick UR5e. The placement of `shoulder_lift_joint` (joint index 1) has rpy
`(0, π/2, 0)`. Compute by hand what `m.placement[1].rotation().matrix()`
should be, then verify against
[`test_urdf_placements.cpp`](../../tests/unit/urdf/test_urdf_placements.cpp)::
`Ur5eShoulderAndUpperArmPlacements`.

## 5. Make the inertial element rotate the tensor

Add a non-identity `rpy` to a link's `<inertial><origin>` in a small URDF.
Verify the loader applies $\bar I_{\text{link}} = R\,\bar
I_{\text{inertial}}\,R^\top$ correctly by hand-computing a 2-D rotation of
a known diagonal inertia and comparing with `Model::inertia[i].inertia_com()`.

## 6. Fuzz it yourself

Write a one-off program that reads `ur5e.urdf`, corrupts a single random
byte, and tries to load it. Run it 100 times — what fraction succeed? How
many raise `UrdfParseError`? Why don't any crash?

---

Solutions are not committed here. The fuzz test under
[`tests/unit/urdf/test_urdf_fuzz.cpp`](../../tests/unit/urdf/test_urdf_fuzz.cpp)
is the reference behaviour you should expect.
