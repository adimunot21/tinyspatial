# Forward vs inverse dynamics

The same physical equation,

$$
\tau \;=\; M(q)\,\ddot q \;+\; h(q, \dot q), \qquad h := C\dot q + g,
$$

gets used in two opposite directions depending on the goal.

## Inverse dynamics: τ given $(q, \dot q, \ddot q)$

Given a motion plan — a sequence of $(q, \dot q, \ddot q)$ over time — what
torques would the motors need to produce to make it happen?

This is what **RNEA** computes (chapter 10). It is used in:

- **Computed-torque control**: given a desired trajectory, the inner
  feedforward is "the torque RNEA reports as needed to follow this
  trajectory." PD feedback then corrects what's left.
- **Trajectory feasibility checking**: given a planned motion, is any torque
  exceeded? (Robots have torque limits per joint.)
- **Energy / power analysis**: $\dot q^\top \tau$ is mechanical power, which
  often needs to be bounded.

Why "inverse"? Because we're inverting Newton's second law: given the
*motion*, find the *force*.

## Forward dynamics: $\ddot q$ given $(q, \dot q, \tau)$

Given torques applied right now (and the current state), what does the robot
do next?

This is what every *simulator* solves at every step. It is used in:

- **Physics simulation**: integrate $\ddot q$ to get $\dot q$ to get $q$,
  step by step.
- **MPC / planning**: the inner forward-simulation that an MPC needs to
  evaluate a candidate action sequence.
- **RL environments**: same as above, with PyTorch around the outside.

Why "forward"? Because we're applying Newton's second law in the natural
direction: given the *force*, find the *acceleration*.

## Two ways to solve forward dynamics

Both are $O(n)$ for chain robots and used in production code somewhere.

### The long way: CRBA + RNEA

$$
\ddot q \;=\; M(q)^{-1}\,(\tau - h(q, \dot q)),
$$

where:
- $M$ comes from **CRBA** (this chapter, section 02);
- $h$ comes from **RNEA** at $\ddot q = 0$ (chapter 10) — it's just the
  inverse-dynamics torque needed to keep $\ddot q = 0$.

Cost: CRBA is $O(n^2)$, RNEA is $O(n)$, the linear solve is $O(n^3)$ in
general but $O(n)$ for chain robots (because $M$ inherits the chain's
sparsity pattern). Conceptually clean; computationally fine for $n < 20$.

### The short way: ABA

**ABA** does forward dynamics in one $O(n)$ sweep, without ever forming
$M$. It is faster, more compact, and uses the *articulated-body inertia*,
introduced by Featherstone. That is section 03.

## When $M$ is actually required

Even with ABA available, $M$ in its dense form is sometimes needed
directly. Examples:

- **Operational-space control**: the projection $\Lambda = (J M^{-1}
  J^\top)^{-1}$ is the apparent inertia at the end-effector, and requires
  $M$.
- **Inertial-shaping controllers**: many adaptive-control proofs require
  $\dot M - 2C$ being skew-symmetric, which is provable structurally but
  also testable numerically with $M$ in hand.
- **Energy estimation**: total kinetic energy is $\tfrac{1}{2} \dot q^\top M
  \dot q$, an instantaneous scalar that requires $M$.

CRBA serves those cases.

## A handy property

For any $\dot q$ and any $\tau$:

$$
\mathrm{ABA}(q, \dot q, \mathrm{RNEA}(q, \dot q, a)) \;=\; a.
$$

I.e. ABA is the inverse of RNEA in $\ddot q$ space. The tests prove this:
`test_aba.cpp:InverseOfRnea` runs 5 random samples on each of 4 fixtures
and checks the round-trip to $10^{-9}$. Forward and inverse dynamics that
fail this test disagree, and *one of them* is wrong.

The library applies the same cross-check directly: `test_aba.cpp:ConsistentWithCrbaAndRnea`
checks $\ddot q_{\mathrm{ABA}} = M^{-1}(\tau - h)$ where $M$ comes from
CRBA and $h$ from RNEA — three independent algorithms agreeing to $10^{-9}$
on every fixture is strong triangulation. A sign flip in any one of them
makes the test fail loudly.

> ## Where this lives in the library
>
> | Concept | File / line |
> | ------- | ----------- |
> | RNEA → bias term | [`rnea.hpp`](../../include/tinyspatial/algo/rnea.hpp) |
> | CRBA → mass matrix | [`crba.hpp`](../../include/tinyspatial/algo/crba.hpp) |
> | ABA → forward dynamics | [`aba.hpp`](../../include/tinyspatial/algo/aba.hpp) |
> | The round-trip test | [`test_aba.cpp:InverseOfRnea`](../../tests/unit/algo/test_aba.cpp) |
