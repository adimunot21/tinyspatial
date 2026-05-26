# Using `tinyspatial` from Python

Three example notebooks live in [`python/examples/`](../../python/examples/).
Each one is self-contained — open it in Jupyter and run cell-by-cell.
This chapter is the executive summary.

## Setup

```bash
# From the repo root:
cmake --preset=validation
cmake --build build/validation -j   # builds the _tinyspatial extension
# Then either install in editable mode, or just add python/ to PYTHONPATH:
export PYTHONPATH=$(pwd)/python
python -c "import tinyspatial as ts; print(ts.SO3.identity())"
```

If you see `SO3(w=1.0, x=0.0, y=0.0, z=0.0)`, you're good.

## Notebook 1 — Forward kinematics on UR5e

[`01_fk_tour_ur5e.ipynb`](../../python/examples/01_fk_tour_ur5e.ipynb)

The point of this notebook is to *see* the kinematic tree. You load a URDF,
inspect the parent array, the joint names, the link names; you compute FK
at a known configuration and verify the wrist position; you sample 1000
random configurations and plot the resulting workspace cloud in 3D.

The pattern is:

```python
import tinyspatial as ts
import numpy as np

model = ts.build_model_from_urdf_file('data/robots/ur5e.urdf')
q = np.array([0.0, -np.pi/2, np.pi/2, -np.pi/2, -np.pi/2, 0.0])
poses = ts.forward_kinematics(model, q)
wrist_T = poses[-1]                    # 4x4 numpy array
wrist_xyz = wrist_T[:3, 3]             # position
wrist_R = wrist_T[:3, :3]              # rotation matrix
```

The Jacobian access follows:

```python
J = ts.compute_jacobian(model, q, link_id=model.njoints - 1, frame=ts.LOCAL)
# J is 6 x nv. Angular-first: rows 0..2 are omega, rows 3..5 are v.
```

## Notebook 2 — RNEA vs Pinocchio

[`02_rnea_vs_pinocchio_franka.ipynb`](../../python/examples/02_rnea_vs_pinocchio_franka.ipynb)

This is the visual sanity check that backs the 1e-10 parity claim. We
build the *same* Franka model in both libraries from the *same* URDF;
sample 1000 random $(q, v, a)$; call RNEA in both; histogram the
residual.

Why use Python for this if there's a C++ test? Because *seeing* a
histogram of 1000 points clustering at 1e-13 is more convincing than
'`ctest` reported pass'. For a portfolio piece, this is the kind of
plot that goes in the README.

The crucial line:

```python
import pinocchio as pin
import tinyspatial as ts

# Same URDF, two libraries, same gravity.
pin_model = pin.buildModelFromUrdf(urdf)
ts_model = ts.build_model_from_urdf_file(urdf)
g = np.array([0., 0., -9.81])
pin_model.gravity.linear = g

tau_pin = pin.rnea(pin_model, pin_data, q, v, a)
tau_ts  = ts.rnea(ts_model, q, v, a, g)
residual = np.max(np.abs(tau_pin - tau_ts))   # expect ~ 1e-13
```

## Notebook 3 — IK with null-space elbow control

[`03_ik_nullspace_elbow_franka.ipynb`](../../python/examples/03_ik_nullspace_elbow_franka.ipynb)

The most useful notebook for application engineers. You define a
Cartesian target for the wrist, solve plain DLS to get *some*
configuration, then solve the *same* target with a secondary 'stay near
the elbow-up rest posture' objective. Side-by-side bar chart shows the
joint angles you get from each.

```python
opts = ts.DlsOptions()
opts.damping = 1e-3
opts.tolerance = 1e-9

res_dls = ts.solve_ik_dls(model, ee_link, target_pose, q_init, opts)

ns_opts = ts.NullspaceOptions()
ns_opts.damping = 1e-3
ns_opts.secondary_gain = 0.5

res_ns = ts.solve_ik_nullspace(model, ee_link, target_pose, q_init,
                                q_rest, ns_opts)

# Both hit the target; only res_ns is biased toward q_rest.
```

The bonus cell calls `ik_implicit_derivative` for the analytical
$\partial q^*/\partial T^*$ — the gateway to chaining IK into PyTorch
or JAX (see chapter 13).

## What if you want the Data object?

Most Python callers don't. The lambdas allocate `Data` internally and
throw it away. If you're calling the same algorithm in a tight loop and
the allocation is showing up in your profile, drop down to the C++
layer — that's the bridge nanobind makes cheap. The Python API is for
*the iteration loop*, not the production hot path.

That said, you *can* construct a `Data` from Python:

```python
data = ts.Data(model)
# ... but there's currently no FK function that takes (model, data, q) ...
```

A future revision (Phase 9) will expose the data-taking variants for
users who profile and find the allocation matters.
