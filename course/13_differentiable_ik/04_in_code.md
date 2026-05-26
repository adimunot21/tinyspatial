# In code

The whole header is ~30 lines of substance. Let's walk through it.

## The function

```cpp
[[nodiscard]] inline MatrixX ik_implicit_derivative(const Model& model, Data& data, int link_id,
                                                    const Eigen::Ref<const VectorX>& q_star,
                                                    Scalar damping = 1e-2) {
  forward_kinematics(model, data, q_star);

  Matrix6X j(6, model.nv());
  compute_jacobian(model, data, link_id, j, JacobianFrame::kLocal);

  const Matrix6 jjt_damped = j * j.transpose() + damping * damping * Matrix6::Identity();
  const Matrix6 inv_jjt = jjt_damped.ldlt().solve(Matrix6::Identity());

  MatrixX dq_dtarget(model.nv(), 6);
  dq_dtarget.noalias() = j.transpose() * inv_jjt;
  return dq_dtarget;
}
```

That's the whole thing. Line by line:

1. **`forward_kinematics(model, data, q_star)`** — fills the per-joint
   transforms needed by the Jacobian.

2. **`compute_jacobian(..., JacobianFrame::kLocal)`** — body-local
   Jacobian $J_L$ at $q^*$. The frame must match the convention used
   by the IK error (body-local in our case).

3. **`Matrix6 jjt_damped = J·J^T + λ²I`** — the damped Gram matrix. The
   `damping * damping` is the $\lambda^2$ from the formula; we ask the
   user for $\lambda$ (not $\lambda^2$) to match the DLS solver's
   convention.

4. **`jjt_damped.ldlt().solve(I_6)`** — invert via LDLT factorisation.
   $J J^T + \lambda^2 I$ is symmetric positive-definite, so LDLT
   handles it cleanly. Cost is $O(1)$ — it's a 6×6 matrix.

5. **`dq_dtarget = J^T · (J J^T + λ²I)⁻¹`** — the damped right-pseudoinverse,
   stored as an $n_v \times 6$ matrix.

Notice we never compute $J^+$ explicitly as a single algorithm step.
Instead we solve a small linear system. This is the standard
"don't form pseudoinverses" pattern from numerical linear algebra.

## What's NOT in the function

A few things you might expect:

- **No convergence check.** The function trusts the caller to pass in a
  converged $q^*$. If you pass in a non-fixed-point, you get the
  *would-be* derivative at that configuration — still a well-defined
  matrix, but its meaning as $\partial q^* / \partial T^*$ depends on
  the IK actually having converged.

- **No SVD-based projector.** A more robust version would use SVD
  truncation to handle exact singularities. The library's damped form
  is good enough for the fixture robots and matches the solver. Future
  work might add an SVD variant.

- **No frame conversion option.** The output is body-local; if you
  need world-frame, post-multiply by `T*.adjoint_inverse()`. We could
  add a `frame` enum like `compute_jacobian` does, but the conversion
  is a one-liner the caller can do.

## A worked numpy example

Once we ship the Python bindings (Phase 8), this becomes natural in
PyTorch. For now you'd write it manually:

```python
import tinyspatial as ts
import numpy as np

# Build model, solve IK to get q_star.
model = ts.build_model_from_urdf_file("data/robots/franka_fr3.urdf")
target = ...  # your SE(3) target as 4x4
q_init = np.zeros(model.nq)
result = ts.solve_ik_dls(model, end_effector_id, target, q_init,
                         damping=1e-5, tolerance=1e-12)
assert result.converged

# Get the implicit derivative.
dq_dtarget = ts.ik_implicit_derivative(model, end_effector_id, result.q,
                                       damping=1e-5)
print(dq_dtarget.shape)  # (7, 6) for Franka

# Now: gradient of some loss L(q*) w.r.t. the target perturbation:
# ∇_(δξ) L = (∂q*/∂(δξ))^T · ∇_q L
def loss_grad_q_star(q_star):
    """Toy task loss: 'middle of joint range' penalty."""
    return q_star  # dL/dq for L = 0.5 * ||q||^2

grad_L_q = loss_grad_q_star(result.q)
grad_L_target_twist = dq_dtarget.T @ grad_L_q  # 6-vector
print("Target perturbation that maximally improves the loss:", grad_L_target_twist)
```

(The Python binding for `ik_implicit_derivative` is Phase 8 work; not
yet exposed.)

## What you'd do for PyTorch

If you wanted to wrap this for PyTorch with `torch.autograd.Function`:

```python
class IkFunction(torch.autograd.Function):
    @staticmethod
    def forward(ctx, target):
        # Run the C++ DLS solver.
        target_np = target.detach().numpy()
        q_star = solve_ik(target_np)
        ctx.target_np = target_np
        ctx.q_star = q_star
        return torch.from_numpy(q_star)

    @staticmethod
    def backward(ctx, grad_output):
        # grad_output has shape (nv,); we want shape of target (probably 6).
        dq_dtarget = ik_implicit_derivative(model, link_id, ctx.q_star)
        # Chain rule: ∇_target L = (∂q*/∂target)^T · ∇_q* L
        grad_target = dq_dtarget.T @ grad_output.numpy()
        return torch.from_numpy(grad_target)
```

About 15 lines. The forward pass calls the C++ solver; the backward
pass calls the implicit derivative. That's the whole "differentiable
IK" interface, done correctly, without unrolling the solver.

## A cautionary tale

If you're tempted to autodiff through the iteration:

```python
# BAD: 200 iterations of damped Newton, unrolled.
q = q_init
for _ in range(200):
    T_k = fk(q)
    e = log(T_k.inverse() @ target)
    J = jacobian(q)
    dq = J.T @ torch.linalg.solve(J @ J.T + lam**2 * I, e)
    q = q + dq
return q
```

The gradient through this:

- Stores 200 intermediate tensors → 200× memory.
- Is **non-zero** even when the iteration is past convergence: small
  numerical wobble in the final iterations propagates back as a noisy
  gradient. The implicit derivative is *the* clean answer in that
  regime.
- Sometimes diverges entirely if the iteration hit a near-singular
  Jacobian on the way.

Always prefer the implicit method.

> ## Where this lives in the library
>
> | Concept | File / line |
> | ------- | ----------- |
> | The whole function | [`differentiable.hpp:52-66`](../../include/tinyspatial/ik/differentiable.hpp#L52-L66) |
> | Tests | [`test_differentiable.cpp`](../../tests/unit/ik/test_differentiable.cpp) |
