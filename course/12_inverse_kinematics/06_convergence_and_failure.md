# When IK fails

Inverse kinematics is a non-convex problem. Production-grade IK code
fails fairly often — single-digit percent of the time on a 6-DoF arm
from random seeds, sometimes more. Knowing how to diagnose and respond
to failure is part of using IK responsibly.

This sub-chapter is a field guide.

## The three classes of failure

### 1. Unreachable target

The target pose is outside the robot's workspace. No configuration
reaches it. Symptoms:

- The error magnitude levels off at some non-zero value and stops
  decreasing.
- The configuration $q$ drifts but the hand stays put.
- Setting `max_iters` higher doesn't help.

**Diagnosis:** Add the reachable-workspace check upstream. The reachable
workspace of an arm is the set of $\{T \mid \exists q : \mathrm{FK}(q) = T\}$;
for typical arms it's a roughly-spherical region with the base at the
center and radius equal to the sum of link lengths.

**Fix:** Don't ask IK to do the impossible. Project the target onto the
workspace, or pick a different one.

### 2. Local-minimum trap

The target *is* reachable, but the iteration converges to a $q$ that
isn't a solution — typically a local minimum of $\|e\|^2$. Symptoms:

- The error decreases monotonically but plateaus above the tolerance.
- The error magnitude is small (say, $10^{-3}$) but not below
  `tolerance` ($10^{-6}$).
- A *different* random seed sometimes succeeds for the same target.

**Diagnosis:** Compare the final $q$ to the seed; if they're close, the
iteration didn't move much from a bad initial guess.

**Fix:** Random restart. Generate a new seed and retry. For our
fixtures, 1–3 restarts brings the success rate from ~85% to ~99%.
Some libraries (Pinocchio's `inverse_kinematics`, `ikpy`) wrap this
automatically; tinyspatial's [`solve_ik_dls`](../../include/tinyspatial/ik/dls.hpp)
leaves it to the caller, intentionally — sometimes the caller wants
to fail fast.

### 3. Singularity-induced oscillation

The iteration is *near* a singularity. The damping helps, but if the
step still has a component along a fast-changing direction, the
iteration can oscillate without converging. Symptoms:

- The error oscillates between two values across consecutive
  iterations.
- The step magnitude doesn't decrease.
- Sometimes the configuration goes to extreme values (joints near
  their limits).

**Diagnosis:** Print $\|J \delta q - e\|$ each iteration. If the
linearisation is bad (residual is large), you're near a singularity.

**Fix options:**

- **Increase `damping`.** From `1e-2` to `5e-2` or `1e-1`. Trades off
  accuracy for stability.
- **Decrease `step_size`.** From `1.0` to `0.5`. Shorter, safer steps.
- **Wampler's adaptive damping** (sub-chapter 03). Damps more when error
  is large.

## What "converged" really means

The library reports `converged = true` when

$$
\|e\|_\infty \le \text{tolerance}
$$

where $e$ is the body-frame Lie-tangent error. This combines
*translation* (the linear part, in metres) and *rotation* (the angular
part, in radians, the magnitude of the rotation axis × angle vector).
At a tolerance of $10^{-6}$ that's about a micrometer of position error
or a microradian of rotation error.

This is **very tight**. For most applications you can comfortably set
`tolerance = 1e-4` and converge faster.

If you need translation and rotation tolerances separately, run the
solve once and then check `result.error.head<3>()` (angular) and
`result.error.tail<3>()` (linear) against your own thresholds.

## Random-restart wrapper sketch

Here's a robust IK call you'd write around the library's solver:

```cpp
tinyspatial::IkResult solve_ik_with_restarts(
    const tinyspatial::Model& m, tinyspatial::Data& d, int link_id,
    const tinyspatial::SE3& target, const tinyspatial::VectorX& q_init,
    int max_restarts = 5) {
  std::mt19937 gen{std::random_device{}()};
  std::uniform_real_distribution<double> uni(-3.0, 3.0);  // wide

  auto result = solve_ik_dls(m, d, link_id, target, q_init);
  if (result.converged) return result;

  for (int r = 0; r < max_restarts; ++r) {
    tinyspatial::VectorX q_seed(m.nq());
    for (int i = 0; i < m.nq(); ++i) q_seed(i) = uni(gen);
    result = solve_ik_dls(m, d, link_id, target, q_seed);
    if (result.converged) return result;
  }
  return result;  // last attempt's result (probably with converged=false)
}
```

About 30 lines. The library doesn't ship this, on purpose — wrapping
strategies depend on the application (does the caller want random
seeds? Should the seed be near the previous solution? Should we time-box
the search?). Easy to write, hard to make canonical.

## Debugging an IK failure

When `solve_ik_dls` returns `converged = false`, here's a recipe:

1. **Look at `result.error`.** Is it dominated by the angular or linear
   part? If angular, your rotation target may be far from reachable
   geometry; if linear, your translation target may be too far.

2. **Run again with `max_iters = 1000` and print iteration-by-iteration
   error magnitudes.** If they're monotonically decreasing but slowly,
   you're fine — just need more iterations. If they plateau, you're in
   a local minimum.

3. **Print the singular values of `J` at the final `q`.** In numpy after
   exporting:
   ```python
   sigmas = np.linalg.svd(J, compute_uv=False)
   print(sigmas[-1] / sigmas[0])  # condition number-ish
   ```
   If the ratio is `< 1e-6`, you're near a singularity.

4. **Try a different `q_init`.** The simplest non-trivial change. Often
   the *only* fix needed.

5. **Bump `damping`.** If the issue is singularity-induced instability,
   `5e-2` is the next thing to try.

## What the library does *not* do

- **No constraint handling.** Joint limits, self-collision, environment
  collision — all silently ignored. The solution $q^*$ may push joints
  past their physical limits.
- **No multi-target / pose-and-orientation prioritisation.** You get
  one full $\mathrm{SE}(3)$ pose target.
- **No global solver.** No genetic algorithms, no IKFast-style
  closed-form generation, no neural-network IK. Just iterative DLS.

These are real limitations. For applications that need them, Pinocchio's
`task-space-inverse-dynamics` extension or full motion-planning
libraries (MoveIt, Drake) are the next step up.

## TL;DR

- Use DLS as your default.
- Wrap it with random restarts for production use.
- If it oscillates near a configuration, bump `damping`.
- If it stops short of tolerance, try a different seed or relax `tolerance`.
- If it's structurally unreachable, IK can't help you — check your geometry.

> ## Where this lives in the library
>
> | Concept | File · symbol |
> | ------- | ------------- |
> | The convergence check | [`dls.hpp`](../../include/tinyspatial/ik/dls.hpp) · `solve_ik_dls` |
> | `IkResult.converged` flag | `dls.hpp` · `IkResult::converged` |
