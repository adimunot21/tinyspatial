# Exercises — Chapter 09

These mix pen-and-paper with C++ probes.

## 1. Predict the Jacobian column

For `simple_arm` at `q = (0, 0)`, predict (in the **LOCAL** frame at link 1):

(a) Column 0 of $J$ (the contribution of joint 0).
(b) Column 1 of $J$ (the contribution of joint 1).

Then verify in C++:

```cpp
const Model m = build_model_from_urdf_file("data/robots/simple_arm.urdf");
Data d(m);
forward_kinematics(m, d, VectorX::Zero(m.nq()));
Matrix6X J(6, m.nv());
compute_jacobian(m, d, /*link_id=*/1, J, JacobianFrame::kLocal);
std::cout << J << "\n";
```

<details><summary>Hint</summary>
joint 0 axis is z. From link 1's frame at q=0 (placed +x by 1 from joint 0,
no rotation), the screw axis of joint 0 has angular = (0, 0, 1) and linear =
(t × ω) at link 1's origin. Here t = (-1, 0, 0) (link 1 is +x from joint 0,
so joint 0 is at -1 from link 1's perspective)… work it out and compare.
</details>

## 2. Frame conversion by hand

For a `simple_arm` configuration where the link 1 frame happens to have
`R_link_world = R_z(π/2)`, write out by hand:

(a) The 6×6 `diag(R, R)` that converts LOCAL → LOCAL_WORLD_ALIGNED.
(b) The 6×6 full adjoint that converts LOCAL → WORLD (translation of link 1
is (0, 1, 0), say).

Apply each to your LOCAL Jacobian column 0 from exercise 1 and check by
calling `compute_jacobian` with the other two frames.

## 3. Drop a rank

Take `ur5e` at `q = (0, 0, 0, 0, 0, 0)`. Compute J in `LOCAL_WORLD_ALIGNED`
and print its singular values via Eigen's `JacobiSVD`. Then sweep
`q[3]` from −π to π in small steps and plot $\sigma_{\min}$. Identify the
configuration(s) where it dips to (near) zero — that's the wrist singularity
of the UR5e in action.

## 4. Why Pinocchio's rows are different

Pinocchio's Jacobian rows are `(v_x, v_y, v_z, ω_x, ω_y, ω_z)`; ours are
`(ω, v)`. Show that the 6×6 permutation matrix `P = [[0, I3], [I3, 0]]`
converts one to the other:

(a) Prove it on paper: if `J_pin = P · J_ts`, then `J_pin · q_dot` is what
order? And which order is `J_ts · q_dot`?
(b) Implement `J_pin = P · J_ts` (or vice versa) and verify against
`tests/validation/test_kinematics.py`, which does this conversion.

## 5. A wrong "obvious" implementation

Imagine you compute the WORLD Jacobian column-by-column by *just rotating
each LOCAL column with `R_link_world`*. Show that this is wrong, and write
down which term you've omitted. (Hint: the $[t]_\times R$ block of the
adjoint.) Then run a test: load `ur5e` at some random `q`, compute
LOCAL J, naively rotate to "world", compare with `compute_jacobian(..., kWorld)`,
and observe the linear rows are off.

## 6. Jacobian transpose for force

If a wrench `f` is applied at the link's frame, the joint torques that
produce that wrench are `τ = J_local^T · f`. Why is it $J^T$ and not $J^{-1}$?
(Hint: duality from chapter 05 — power $f \cdot v$ is invariant under frame
change.) Verify numerically on `simple_arm` for any `q` and any small `f`.

---

Solutions aren't committed. The C++ tests and the parity table are the
references.
