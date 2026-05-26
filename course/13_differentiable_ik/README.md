# Chapter 13 — Differentiable inverse kinematics

You've used the IK solver as a function: pose in, configuration out.

$$
q^* \;=\; \mathrm{IK}(T^*, q_{\mathrm{init}}).
$$

What if you wanted to use it as a **building block in a bigger
gradient-based pipeline**? Imagine:

- A planner that outputs target poses, learned by gradient descent on a
  task loss. The loss depends on $q^*$ (e.g., torque, joint limits,
  reachability); the gradient has to flow through IK.
- An MPC inner loop that uses IK to convert Cartesian setpoints into
  joint setpoints; warm-starting and sensitivity matter for stability.
- A neural network that predicts $q^*$ from raw observations, trained
  by also feeding $q^*$ through the FK map. The IK Jacobian appears in
  the loss gradient.

For any of this, you need $\partial q^* / \partial T^*$: how the IK
solution moves when the target moves. That's what "differentiable IK"
provides.

This chapter is short on purpose — it builds on chapter 12. If you
haven't read that yet, do.

## The trick

You don't actually have to differentiate the IK algorithm itself. You
can use the **implicit function theorem** to read off the derivative
from the *fixed-point condition* alone — and the result is just the
damped pseudoinverse you've already met.

That makes "differentiable IK" essentially free: one extra Jacobian
call, one 6 × 6 linear solve, done.

## The sub-chapters

1. [Why differentiable IK matters](01_why_differentiable_ik.md) — three
   use cases that motivate the rest.
2. [The implicit function theorem](02_implicit_function_theorem.md) —
   the math, with concrete examples before we apply it to IK.
3. [The IK case](03_the_ik_case.md) — derivation of
   `∂q*/∂T* = J⁺` from the IK fixed-point condition.
4. [In code](04_in_code.md) — `differentiable.hpp` walked through, plus
   a worked numpy example via the Python binding.

Then: [exercises](exercises.md).

## Prerequisites

Chapter 12 (inverse kinematics) and chapter 09 (Jacobians), at least.
Chapter 04 (Lie groups, especially `exp`/`log` on $\mathrm{SE}(3)$)
is also load-bearing.

## Where this lives in the library

| Concept | File |
| ------- | ---- |
| Differentiable IK | [`include/tinyspatial/ik/differentiable.hpp`](../../include/tinyspatial/ik/differentiable.hpp) |
| Unit tests | [`tests/unit/ik/test_differentiable.cpp`](../../tests/unit/ik/test_differentiable.cpp) |

## Further reading

- **Mariano Phielipp & Pieter Abbeel**, "Differentiable Inverse
  Kinematics for Multi-Step Motion Planning" — uses exactly the IFT
  trick we use here, in an RL context.
- **PyTorch `torch.autograd.Function`** — read the docs on custom
  backward passes. The IK derivative we compute here is exactly what
  you'd put in `backward()` to make IK a torch-differentiable function.
- **The Adjoint State Method** (Pontryagin, 1960s) — the dynamic
  analogue. Same idea: derivatives at the fixed point without
  unrolling.
