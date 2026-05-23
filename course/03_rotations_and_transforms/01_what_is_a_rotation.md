# What is a rotation?

A rotation is a way of re-orienting things in space *without* stretching,
squashing, or moving them off a fixed point. Spin a globe: every city stays the
same distance from the centre and the same distance from every other city. Only
the *orientation* changed.

Mathematically, a rotation in 3-D is a transformation of vectors that

- preserves lengths (‖Rv‖ = ‖v‖ for every vector v), and
- preserves orientation (it doesn't turn a right hand into a left hand — no
  mirror flips).

That second condition is what separates a rotation from a reflection. Both keep
lengths; only rotations keep "handedness."

## Active vs passive: the eternal confusion

Here is the single most common source of sign bugs in robotics, so we pin it
down now.

- **Active (alibi):** the rotation moves the *object*. The coordinate frame
  stays put; the arrow you're rotating swings to a new direction.
- **Passive (alias):** the object stays put; you rotate the *coordinate frame*
  you're describing it in. The arrow is unchanged, but its coordinates change.

These two give *inverse* answers. Rotating an object by +30° (active) produces
the same new coordinates as leaving it alone and rotating your frame by −30°
(passive).

**tinyspatial uses the active convention.** When you call `SO3::exp(ω)` and then
`.act(v)`, you are physically rotating the vector `v`. We say so explicitly so
you never have to guess. (Pinocchio agrees here.)

## Why orientation is genuinely hard

Position is easy: three numbers (x, y, z), add them like you'd expect. You might
hope orientation is the same — "just store three angles." It isn't, for two
reasons that motivate the whole next few chapters:

1. **Rotations don't commute.** Rotate a book 90° about the vertical axis, then
   90° about the axis pointing right; now do the same two rotations in the
   opposite order. The book ends up in a *different* orientation. Order matters,
   which is exactly the behaviour of matrix multiplication (also non-commuting)
   — a strong hint about how to represent rotations.
2. **The space of rotations is curved.** There's no way to smoothly label every
   orientation with three numbers without something going wrong somewhere (a
   "gimbal lock" singularity, or a discontinuity). We'll make this precise in
   chapter 04 when we call the set of rotations a *manifold*.

## Try it in your head

Hold your right hand flat, fingers forward, palm down. Rotate +90° about the
vertical (thumb now points left). Then rotate +90° about the forward axis.
Note the final palm direction. Reset, and do the two rotations in the other
order. Different result? That's non-commutativity, felt directly.

## Where this lives in the library

The active convention is realised by `SO3::act`, which rotates a point:

| Concept | File · lines |
| ------- | ------------ |
| `act(p)` rotates a vector (active convention) | [`so3.hpp`](../../include/tinyspatial/liegroup/so3.hpp) · `act` |
| Composition order (non-commuting `*`) | [`so3.hpp`](../../include/tinyspatial/liegroup/so3.hpp) · `operator*` |

Next: [Rotation matrices](02_rotation_matrices.md).
