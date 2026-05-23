# The inertial element

`<inertial>` is where URDF tells us the *mass distribution* of a link.
There's some convention work happening here that's easy to get wrong, so
let's unpack it carefully.

## The block

```xml
<inertial>
  <mass value="3.5"/>
  <origin xyz="0 0 0.1" rpy="0 0 0"/>
  <inertia ixx="0.02" iyy="0.02" izz="0.005"
           ixy="0"    ixz="0"    iyz="0"/>
</inertial>
```

Three pieces of data:

- **`<mass value>`** — the scalar mass in kilograms.
- **`<origin>`** — the pose of the *centre of mass* in the link's frame. The
  translation is the COM position; the rotation, if non-identity, rotates the
  inertia-tensor axes (see below).
- **`<inertia>`** — the $3\times3$ rotational inertia tensor *about the COM*,
  expressed in the inertial-element's frame (i.e. the frame defined by
  `<origin rpy>`).

Six independent numbers because the tensor is symmetric:

$$
\bar I = \begin{bmatrix} \mathrm{ixx} & \mathrm{ixy} & \mathrm{ixz} \\
                          \mathrm{ixy} & \mathrm{iyy} & \mathrm{iyz} \\
                          \mathrm{ixz} & \mathrm{iyz} & \mathrm{izz}
\end{bmatrix}.
$$

## What the loader produces

Our `SpatialInertia` (chapter 05) is *(mass, COM, inertia about COM in the
link frame)*. URDF gives us *(mass, COM, inertia about COM in the inertial
element's frame)*. The inertial element's frame can be rotated relative to
the link frame via `<origin rpy>`. So the parser does:

1. Read mass, COM (translation of `<origin>`), and $\bar I_{\text{inertial}}$.
2. Compute $R$ = rotation matrix from the `rpy` of `<origin>`.
3. Rotate the tensor into the link frame:
   $\bar I_{\text{link}} = R\,\bar I_{\text{inertial}}\,R^\top$.
4. Construct `SpatialInertia(mass, COM, inertia_link_frame)`.

Most URDFs leave `<origin rpy>` at zero, in which case the rotate-step is a
no-op. The few that use a non-trivial inertial origin do so because the
principal axes of inertia aren't axis-aligned with the link frame, and the
rotation expresses that.

## Quick checks worth doing

If you find yourself debugging an inertia issue, ask:

1. **Is the tensor symmetric?** It must be. The off-diagonal `ixy`, `ixz`,
   `iyz` appear in both halves.
2. **Is it positive (semi-)definite?** A real rigid body has positive
   diagonal entries (`ixx`, `iyy`, `izz`) and the eigenvalues are all
   non-negative. The library's `Matrix6IsSymmetric` and
   `KineticEnergyFormula` tests would expose negative-PSD inertias.
3. **Is the COM where you expect?** Print `model.inertia[i].com()` and
   compare to your CAD.

## Where this lives in the library

| Concept | File · symbol |
| ------- | ------------- |
| URDF inertial parser | [`src/urdf/urdf_loader.cpp`](../../src/urdf/urdf_loader.cpp) · `parse_inertial()` |
| Spatial inertia destination | [`spatial/inertia.hpp`](../../include/tinyspatial/spatial/inertia.hpp) · `SpatialInertia` |
| Round-trip test | [`test_urdf_loader.cpp`](../../tests/unit/urdf/test_urdf_loader.cpp) · `SimpleArmRoundTrip` |

Next: [The loader](04_the_loader.md).
