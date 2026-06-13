# The inverse problem

You have a robot at configuration $q$. You can call `forward_kinematics`
and it tells you where every link of the robot is. Now reverse the
direction:

> *"Move the hand to **here**. What joint angles do I need?"*

That's inverse kinematics. Let's make the statement precise.

## The formal problem

Pick a link $L$ on the robot. Pick a target pose $T^* \in \mathrm{SE}(3)$
expressed in the world frame. Find a configuration $q^*$ such that

$$
\mathrm{FK}_L(q^*) \;=\; T^*.
$$

The map $\mathrm{FK}_L : \mathbb{R}^{n_q} \to \mathrm{SE}(3)$ is in
general:

- **Nonlinear.** Composition of joint transforms is multiplicative, not
  additive.
- **Many-to-one** for $n_v \geq 6$. For a 7-DoF arm, infinitely many
  $q$ map to the same $T^*$.
- **Onto a proper subset.** Some $T^*$ aren't reachable by any $q$ —
  outside the workspace, the wrong orientation at this distance, etc.

So "the" solution might be one of many, or might not exist at all. Real
IK code has to be honest about this.

## Why we don't just solve it algebraically

For a few special robots — *e.g.* the classical 6-DoF arm with a
spherical wrist (Pieper conditions) — you can write down a closed-form
solution by hand. It's pretty: a sequence of `atan2`s and square roots.

But the moment the geometry deviates from those special structures
(7-DoF, offset wrists, parallel axes), closed-form goes away. There's a
[whole research area](https://en.wikipedia.org/wiki/Inverse_kinematics)
about polynomial-system-solving methods (the IKFast school, originally
by Rosen Diankov), but those are heavyweight code generators, not
runtime libraries.

What every general-purpose robotics library does instead: **iterate**.

## The iterative idea

Start from some initial guess $q_0$. At each step:

1. Compute the current pose $T_k = \mathrm{FK}_L(q_k)$.
2. Compute the *error* between $T_k$ and $T^*$.
3. Update $q_{k+1} = q_k + \Delta q_k$ where $\Delta q_k$ is chosen to
   reduce the error.

Repeat until the error is small enough. Each step is cheap (one FK call,
one Jacobian, a small linear solve); we just do many of them.

This is Newton's method on a manifold. The next sub-chapter (02) tells
you how to define the "error" and the "step" precisely — and the answer
involves $\log_{\mathrm{SE}(3)}$, which you met in chapter 04.

## Writing the error on SE(3)

The hand pose is a rigid transform, not a vector. You can't just subtract
$T_k$ from $T^*$ — that operation isn't defined on a Lie group. Instead,
the standard definition of the *body-frame* pose error is:

$$
e = \log_{\mathrm{SE}(3)}\!\bigl(T_k^{-1} \cdot T^*\bigr) \;\in\; \mathbb{R}^6.
$$

In words: ask "what twist would I have to apply to body $L$ in *its
own frame* over unit time to take it from where it is to where it
should be?" The answer is the Lie tangent — a 6-vector in
angular-first ordering.

Two properties make this the right definition:

- **It's zero when $T_k = T^*$.** Obvious but important.
- **It's locally linear in the configuration error.** If $q_k$ is close
  to $q^*$, then $e \approx J_L(q_k) \cdot (q^* - q_k)$, where
  $J_L$ is the *local-frame* Jacobian (chapter 09). That's what lets
  Newton's method work.

In code:

```cpp
const SE3 current = data.pose_in_world[link_id];
const SE3 err_se3 = current.inverse() * target_in_world;
const Vector6 e = err_se3.log();
```

That `Vector6 e` is the input to every iteration of every IK solver in
this library.

## What happens when the error is big

The local linearisation isn't perfect. For large $e$ (say, the hand is
30 cm from the target), the Newton step might *overshoot* — pushing the
hand past the target, possibly into a worse configuration. There are
three classic tricks for handling this, in roughly increasing
sophistication:

1. **Step-size scaling.** Multiply $\Delta q$ by some $\alpha \in (0, 1]$.
2. **Damping.** Use the *damped* pseudoinverse instead of the exact one,
   which produces shorter, safer steps when the Jacobian is poorly
   conditioned. (This is what DLS does. Sub-chapter 03.)
3. **Line search.** Compute multiple candidate $\Delta q$ magnitudes and
   pick the one that minimises the error along that direction.

The library uses (1) and (2). (3) is occasionally worth adding if you
have a hard convergence problem; it's not in `dls.hpp` because it would
double the per-iteration FK count.

> ## Where this lives in the library
>
> | Concept | File · symbol |
> | ------- | ------------- |
> | The Lie-tangent error | [`dls.hpp`](../../include/tinyspatial/ik/dls.hpp) · `solve_ik_dls` (the `log()` residual) |
> | The iterative loop | `dls.hpp` · `solve_ik_dls` |
