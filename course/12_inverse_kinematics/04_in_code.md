# In code: `solve_ik_dls`

`include/tinyspatial/ik/dls.hpp` is small — ~50 lines of substance. This
sub-chapter walks through every line.

## The signature

```cpp
[[nodiscard]] inline IkResult solve_ik_dls(const Model& model, Data& data, int link_id,
                                           const SE3& target_in_world,
                                           const Eigen::Ref<const VectorX>& q_init,
                                           const DlsOptions& options = {});
```

Inputs:

- `model` / `data` — kinematic tree + scratchpad (chapter 06).
- `link_id` — which link's body frame we're placing.
- `target_in_world` — where we want it.
- `q_init` — starting guess. Usually the robot's current configuration.
- `options` — `max_iters`, `tolerance`, `damping`, `step_size`.

Output: `IkResult { q, error, iterations, converged }`. Note we
*always* return a result; non-convergence is signalled via the
`converged` flag, not via an exception. That's because IK failure is
common enough that exceptions would be the wrong API.

## The options struct

```cpp
struct DlsOptions {
  int max_iters = 200;
  Scalar tolerance = 1e-6;
  Scalar damping = 1e-2;
  Scalar step_size = 1.0;
};
```

Sensible defaults for a 6/7-DoF arm working in metres and radians. If
you find yourself tuning these:

- `damping` smaller (e.g. `1e-3`) → tighter convergence, more risk near
  singularities.
- `damping` larger (e.g. `1e-1`) → safer near singularities, slower
  convergence elsewhere.
- `step_size` smaller (e.g. `0.5`) → less oscillation when far from
  target, but slower.
- `max_iters` larger → costs more compute, gives slow seeds a chance.

## The iteration loop

```cpp
for (int iter = 0; iter < options.max_iters; ++iter) {
  forward_kinematics(model, data, result.q);
  const SE3 current = data.pose_in_world[link_id];
  const SE3 err_se3 = current.inverse() * target_in_world;
  result.error = err_se3.log();

  if (result.error.cwiseAbs().maxCoeff() <= options.tolerance) {
    result.iterations = iter;
    result.converged = true;
    return result;
  }

  compute_jacobian(model, data, link_id, j, JacobianFrame::kLocal);

  const Matrix6 jjt_damped = j * j.transpose() + lambda_sq_i6;
  const Vector6 alpha = jjt_damped.ldlt().solve(result.error);
  const VectorX dq = j.transpose() * alpha;

  result.q.noalias() += options.step_size * dq;
}
```

Things worth noticing:

1. **Forward kinematics first.** Computes both `pose_in_parent` and
   `pose_in_world` for every joint. The Jacobian call uses
   `pose_in_world` internally, so we don't need to re-do FK after the
   step until next iteration.

2. **Body-frame error.** `current.inverse() * target_in_world` is the
   pose error expressed in the link's own frame. `.log()` is the
   $\mathrm{SE}(3)$ Lie logarithm, returning a 6-vector twist.

3. **Convergence check before the Jacobian.** Saves one Jacobian call
   per success. Cheap-first.

4. **`Matrix6 lambda_sq_i6` precomputed.** Pulled out of the loop because
   it doesn't change.

5. **`ldlt().solve(...)`.** $J J^\top + \lambda^2 I$ is symmetric
   positive definite, so LDLT is the right factorisation. Cholesky
   (LLT) would also work and be slightly faster; LDLT handles the
   borderline case where $J$ is near-singular more gracefully.

6. **`.noalias()`.** Eigen idiom for "trust me, the LHS and RHS don't
   alias; skip the safety copy." `+=` with `noalias` is fine because
   the LHS appears on neither side of the multiplication.

7. **No early exit on stalled progress.** If the step doesn't reduce
   the error, we just keep going. A more sophisticated solver might
   detect stagnation and *e.g.* increase damping or re-seed; we don't
   do that — the caller is expected to handle non-convergence by
   restarting from a fresh seed.

## A worked call

```cpp
#include "tinyspatial/ik/dls.hpp"
#include "tinyspatial/urdf/urdf_loader.hpp"

const auto m = tinyspatial::build_model_from_urdf_file("data/robots/franka_fr3.urdf");
tinyspatial::Data d(m);
const int end_effector = m.njoints() - 1;

// Target: 10 cm above and 50 cm forward of where the EE currently is at q=0.
tinyspatial::forward_kinematics(m, d, tinyspatial::VectorX::Zero(m.nq()));
tinyspatial::SE3 target = d.pose_in_world[end_effector];
target.translation()(0) += 0.5;
target.translation()(2) += 0.1;

const auto q_init = tinyspatial::VectorX::Zero(m.nq());
const auto result = tinyspatial::solve_ik_dls(m, d, end_effector, target, q_init);

if (result.converged) {
  std::cout << "Converged in " << result.iterations << " iterations.\n";
  // Use result.q
} else {
  std::cout << "IK failed; final error: " << result.error.cwiseAbs().maxCoeff() << '\n';
  // Maybe retry with a different seed
}
```

That's it. Five-line setup, one-call IK.

> ## Where this lives in the library
>
> | Concept | File / line |
> | ------- | ----------- |
> | `DlsOptions` | [`dls.hpp:39-49`](../../include/tinyspatial/ik/dls.hpp#L39-L49) |
> | `IkResult` | [`dls.hpp:52-62`](../../include/tinyspatial/ik/dls.hpp#L52-L62) |
> | Iteration loop | [`dls.hpp:70-94`](../../include/tinyspatial/ik/dls.hpp#L70-L94) |
