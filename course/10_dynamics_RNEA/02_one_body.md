# Newton–Euler on one body

Before we walk a tree, let's do RNEA on a single rigid body floating in space.
This is the whole algorithm — passes, gravity trick, dual-adjoint transports
— concentrated into the smallest possible system.

## The setup

One rigid body with spatial inertia $I$ (in its own body frame). You know
its spatial velocity $v$ (a twist; chapter 05) and you would like it to
accelerate at $a$ (also in its body frame). Gravity in the world is $g$
(a 3-vector). What spatial wrench $f$ must act on the body?

## Newton–Euler in spatial form

Newton's second law for a rigid body, written in body-frame spatial
quantities (Featherstone 2008 eq. 2.78), is

$$
f \;=\; I\,a \;+\; v \times^{*} (I\,v) \;-\; f_{\text{gravity}},
$$

where:

- $I a$ is the "rate of change of momentum due to acceleration" term —
  the spatial analogue of $m a$.
- $v \times^{*} (I v)$ is the Coriolis-like bias: the body's momentum
  changes direction because the body is spinning, and that requires a force
  even at $a = 0$. The $\times^{*}$ is the force-side spatial cross-product
  (chapter 05). For a body with $v = (\omega; v_{\text{linear}})$, this is
  exactly the gyroscopic torque $\omega \times I_{\text{com}} \omega$ plus
  some linear-momentum coupling.
- $f_{\text{gravity}}$ is the (body-frame) gravitational force; for a body
  with mass $m$ and COM at $c$ in body frame, it's
  $f_{\text{gravity}} = (c \times m R^\top g, \; m R^\top g)$
  where $R$ is the rotation from body to world.

That last line is annoying. Every body has its own $R$, its own COM, so
$f_{\text{gravity}}$ is a different expression per body. We can do better.

## The gravity trick

Here is one of the prettiest pieces of computational dynamics. Move
gravity from a force on each body to a *fictitious acceleration of the
base*.

Imagine the whole world is in a falling elevator that accelerates upward
at exactly $-g$ — *i.e.* the elevator's floor pushes up at $-g$ on
everything inside, so a free-floating object accelerates at $+g$. To an
observer inside the elevator, gravity has disappeared. We do the same
thing computationally: pretend the base accelerates at $-g$ in world
coordinates, and write the bodies' accelerations relative to that.

Then $f_{\text{gravity}}$ vanishes from each body's Newton–Euler equation,
and the propagated $a$ already includes the gravitational contribution.
Concretely, the recursion will start the root with

$$
a_0 \;=\; X_0\,(-g \text{ as a spatial acceleration in world})
$$

and every body's $a$ that follows inherits this. Output torques come out
including the gravity bias automatically.

In our code:

```cpp
const Motion neg_gravity_world(Vector3::Zero(), -gravity);
// ...
data.a[i] = (i_from_parent * neg_gravity_world) + a_j + cross(data.v[i], v_j);
```

There is no `if gravity != 0` branch and no per-body gravity force.

## Single-body RNEA

For a single body, "RNEA" collapses to one step:

```
v = v_joint                                    # joint contributes velocity
a = X_base * (-g_world) + a_joint              # gravity trick + joint accel
f = I·a + v ×* (I·v)                           # spatial Newton–Euler
τ = Sᵀ · f                                     # project onto joint subspace
```

That is exactly what RNEA's outer recursion does N times, once per body,
with the kinematic propagation handled by the same `data.v[i]`,
`data.a[i]` machinery you've already met from forward kinematics.

## Why the dual adjoint?

In the tree version, we transport forces from child to parent. That
transport uses the *force* Plücker $X^{*}_{T} = \mathrm{Ad}_{T}^{-\top}$, not
the motion adjoint. Twists transform with $X$; wrenches transform with
$X^{*}$. The library makes this a type error — `Motion * Force` doesn't
compile — so we get the duality right by construction. Chapter 05 has the
why; this chapter just relies on it.

## Sanity check (mental)

If $v = a = 0$ and $g = 0$: $f = 0$. ✓ — no motion, no force.

If $v = a = 0$ and $g \neq 0$: $f = -X_0 g$ (a body-frame gravity force).
For a body with mass 1 kg at origin and gravity in $-z$:
$f$ has linear part $(0, 0, 9.81)$ N pushing *up* in the body's frame —
exactly the force the base must exert to hold the body against
gravity. ✓

> ## Where this lives in the library
>
> | Concept | File / line |
> | ------- | ----------- |
> | `I·a + v ×* (I·v)` | [`rnea.hpp:120-121`](../../include/tinyspatial/algo/rnea.hpp#L120-L121) |
> | Gravity trick | [`rnea.hpp:96, 110`](../../include/tinyspatial/algo/rnea.hpp#L96-L110) |
> | `Sᵀ·f` projection | [`rnea.hpp:63-81`](../../include/tinyspatial/algo/rnea.hpp#L63-L81) |
