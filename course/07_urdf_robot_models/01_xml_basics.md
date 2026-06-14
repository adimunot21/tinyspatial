# XML, in five minutes

XML is text describing a *tree of named boxes with labels*. Every box
("element") has a name, optional attributes (key-value pairs), and either some
text or more boxes nested inside it. The syntax is:

```xml
<box-name attribute="value" another="value2">
  ... contents (more boxes or plain text) ...
</box-name>
```

Or, if the box has no contents, the shorthand:

```xml
<box-name attribute="value"/>
```

(That trailing slash is "self-closing.")

## A worked example

Here is a tiny robot in URDF:

```xml
<?xml version="1.0"?>
<robot name="my_robot">
  <link name="base">
    <inertial>
      <mass value="2.0"/>
      <origin xyz="0 0 0.05" rpy="0 0 0"/>
    </inertial>
  </link>
  <link name="arm"/>
  <joint name="elbow" type="revolute">
    <parent link="base"/>
    <child link="arm"/>
    <origin xyz="0 0 0.1"/>
    <axis xyz="0 0 1"/>
  </joint>
</robot>
```

Read top to bottom:

- `<robot name="my_robot">` — the outermost box, named "my_robot."
- Inside it, two `<link>` boxes (one with an `<inertial>` sub-box) and one
  `<joint>` box.
- Each box's attributes (`name="..."`) and sub-boxes (`<mass value="2.0"/>`)
  describe its data.
- Closing tags (`</robot>`) end each box.

## What we extract

The URDF parser walks the tree once, ignores most details, and pulls out
exactly:

- The `<robot>` name (becomes `Model::name`).
- Each `<link>`'s name + optional `<inertial>` (becomes a `SpatialInertia`).
- Each `<joint>`'s name, type, parent, child, `<origin>`, and `<axis>`.

Everything else — `<visual>` blocks, `<collision>` blocks, ROS-specific tags
like `<transmission>` — is silently skipped, because we are a kinematics and
dynamics library, not a renderer.

## A note on XML's encoding quirks

URDF files are UTF-8 text. Strings are quoted with double quotes (`"..."`).
Numbers inside attributes are space-separated:

```xml
<origin xyz="1.5 0 -2"/>
```

is a 3-vector `(1.5, 0, -2)`. Boolean-looking values are strings, not types:
`<mass value="2.0"/>` is the string `"2.0"`, which the parser converts to a
`double`.

That is essentially all you need to read URDF. The next chapter goes through
each tag we support.

## Where this lives in the library

The XML *parsing* is done by the tinyxml2 library we vendor under
`third_party/tinyxml2/`. We use its tree-walking API (`FirstChildElement`,
`NextSiblingElement`, `Attribute`) and nothing more.

| Concept | File |
| ------- | ---- |
| XML parser library | [`third_party/tinyxml2/`](../../third_party/tinyxml2) |
| Where we call into it | [`src/urdf/urdf_loader.cpp`](../../src/urdf/urdf_loader.cpp) |

Next: [The URDF tags we care about](02_urdf_tags.md).
