# The URDF tags we care about

URDF defines dozens of tags. The kinematic and dynamic subset we need is
about ten, and you can fit it on one page.

## `<robot>` — the root

```xml
<robot name="my_robot">
  ...
</robot>
```

Wraps everything. `name` becomes `Model::name`. A URDF must have exactly one
`<robot>` root, or the loader throws.

## `<link>` — a rigid body

```xml
<link name="link_42">
  <inertial>
    <mass value="3.5"/>
    <origin xyz="0 0 0.1" rpy="0 0 0"/>
    <inertia ixx="0.01" iyy="0.01" izz="0.01" ixy="0" ixz="0" iyz="0"/>
  </inertial>
  <visual>      ...</visual>    <!-- ignored -->
  <collision>   ...</collision> <!-- ignored -->
</link>
```

- `name` (required) — the link's URDF name. Becomes `Model::link_names[i]`
  for the joint whose child is this link.
- `<inertial>` (optional) — the mass distribution. Chapter 03 walks through
  how this becomes a `SpatialInertia`. Without it, the link is treated as
  massless.
- `<visual>` and `<collision>` — meshes for rendering / collision checking.
  We're not a renderer, so we ignore them.

## `<joint>` — a constraint

```xml
<joint name="elbow" type="revolute">
  <parent link="upper_arm"/>
  <child link="forearm"/>
  <origin xyz="0 0 0.4" rpy="0 1.5707 0"/>
  <axis xyz="0 0 1"/>
  <limit lower="-3.14" upper="3.14" effort="10" velocity="1"/> <!-- ignored -->
</joint>
```

The mandatory bits:

- `name` (required) — joint's URDF name.
- `type` (required) — one of `revolute`, `continuous`, `prismatic`, `fixed`,
  `floating`. The loader rejects anything else.
- `<parent link="...">` and `<child link="...">` — define which links the
  joint connects.
- `<origin>` (optional, defaults to identity) — where the joint frame sits
  in the parent link's frame.
- `<axis>` (optional for `revolute`/`prismatic`; defaults to `(1, 0, 0)`) —
  the unit axis of motion in the joint frame.

`<limit>`, `<dynamics>`, `<safety_controller>`, `<mimic>` — silently ignored
(joint limits and frictions are concerns of trajectory planners and
simulators, not of the kinematics/dynamics core).

## `<origin>` — a pose

`<origin>` shows up inside both `<inertial>` and `<joint>` (and the
ignored `<visual>` / `<collision>`). It's always a 6-number rigid transform:

```xml
<origin xyz="1.0 2.0 3.0" rpy="0.1 0.2 0.3"/>
```

- `xyz` is the translation, in metres, in the parent's frame.
- `rpy` is the rotation, in radians, as **roll-pitch-yaw**: first roll
  about *x*, then pitch about the (rotated) *y*, then yaw about the (twice-
  rotated) *z*. The composed rotation matrix is `R = Rz(yaw) · Ry(pitch) ·
  Rx(roll)`. Defaults to identity.

This is the rare place URDF differs subtly from the way many robotics
texts write rotations. Get it wrong and your placements come out
transposed; get it right and the rest is mechanical.

## What we deliberately don't support

These tags appear in real URDFs but we treat them as errors or skip them
outright:

| Tag / feature | What we do | Why |
| ------------- | ---------- | --- |
| xacro substitution | **Reject** | A macro language, not URDF. Run xacro yourself. |
| `<mimic>` joints | **Reject** | Composition, not a primitive. |
| `<planar>` / `<screw>` joints | **Reject** | Not currently in scope. |
| `<transmission>`, `<gazebo>` | **Ignore** | ROS-specific extensions. |
| `<visual>`, `<collision>` | **Ignore** | We're not a renderer. |
| `<limit>`, `<dynamics>` | **Ignore** | Planning/sim concerns. |

If your URDF needs any of the rejected features, pre-process it (or convert
to plain URDF) before loading. The contract is in
[`urdf_loader.hpp`](../../include/tinyspatial/urdf/urdf_loader.hpp).

Next: [The inertial element](03_inertias.md).
