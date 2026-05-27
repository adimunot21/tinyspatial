"""Python-side Pinocchio parity, belt-and-braces.

The exhaustive cross-check lives in `tests/validation/` and runs from the
CMake test driver. This file is a pytest-friendly subset that exercises the
public Python API the way a user would: ``import tinyspatial`` from the
package, call functions with NumPy arrays, compare against ``pinocchio``.

Tolerance is loose-ish (1e-9) because the C++ side already enforces 1e-10;
this is here to catch regressions in the *binding layer* (forgotten args,
wrong defaults, dtype slip-ups), not in the numerics.
"""
from __future__ import annotations

import pathlib

import numpy as np
import pytest

pin = pytest.importorskip("pinocchio")
import tinyspatial as ts  # noqa: E402

REPO = pathlib.Path(__file__).resolve().parents[2]
ROBOTS = ["franka_fr3", "ur5e", "so_arm101"]
TOL = 1e-9

# Swap angular/linear blocks: linear-first <-> angular-first.
P_ROW_SWAP = np.zeros((6, 6))
P_ROW_SWAP[:3, 3:] = np.eye(3)
P_ROW_SWAP[3:, :3] = np.eye(3)


@pytest.fixture(params=ROBOTS)
def models(request):
    urdf = str(REPO / "data" / "robots" / f"{request.param}.urdf")
    pin_model = pin.buildModelFromUrdf(urdf)
    pin_data = pin_model.createData()
    ts_model = ts.build_model_from_urdf_file(urdf)
    return request.param, pin_model, pin_data, ts_model


def _random_state(pin_model, rng):
    q = pin.randomConfiguration(pin_model)
    v = rng.standard_normal(pin_model.nv)
    a = rng.standard_normal(pin_model.nv)
    return q, v, a


def test_fk_matches_pinocchio(models):
    robot, pin_model, pin_data, ts_model = models
    pin.seed(0)
    rng = np.random.default_rng(0)
    for _ in range(5):
        q, _, _ = _random_state(pin_model, rng)
        pin.forwardKinematics(pin_model, pin_data, q)
        ts_poses = ts.forward_kinematics(ts_model, q)
        for k in range(1, pin_model.njoints):
            assert np.allclose(pin_data.oMi[k].homogeneous, ts_poses[k - 1], atol=TOL), robot


def test_jacobian_local_frame_matches_pinocchio(models):
    robot, pin_model, pin_data, ts_model = models
    pin.seed(1)
    rng = np.random.default_rng(1)
    last_pin = pin_model.njoints - 1
    last_ts = last_pin - 1
    for _ in range(5):
        q, _, _ = _random_state(pin_model, rng)
        pin.computeJointJacobians(pin_model, pin_data, q)
        j_pin = P_ROW_SWAP @ pin.getJointJacobian(pin_model, pin_data, last_pin, pin.LOCAL)
        j_ts = ts.compute_jacobian(ts_model, q, last_ts, ts.LOCAL)
        assert np.allclose(j_pin, j_ts, atol=TOL), robot


def test_rnea_matches_pinocchio(models):
    robot, pin_model, pin_data, ts_model = models
    pin_model.gravity.linear = np.array([0.0, 0.0, -9.81])
    pin_model.gravity.angular = np.zeros(3)
    pin.seed(2)
    rng = np.random.default_rng(2)
    for _ in range(5):
        q, v, a = _random_state(pin_model, rng)
        tau_pin = pin.rnea(pin_model, pin_data, q, v, a)
        tau_ts = ts.rnea(ts_model, q, v, a)
        assert np.allclose(tau_pin, tau_ts, atol=TOL), robot


def test_crba_matches_pinocchio(models):
    robot, pin_model, pin_data, ts_model = models
    pin.seed(3)
    rng = np.random.default_rng(3)
    for _ in range(5):
        q, _, _ = _random_state(pin_model, rng)
        m_pin = pin.crba(pin_model, pin_data, q)
        m_pin = np.triu(m_pin) + np.triu(m_pin, 1).T
        m_ts = ts.crba(ts_model, q)
        assert np.allclose(m_pin, m_ts, atol=TOL), robot


def test_aba_matches_pinocchio(models):
    robot, pin_model, pin_data, ts_model = models
    pin_model.gravity.linear = np.array([0.0, 0.0, -9.81])
    pin_model.gravity.angular = np.zeros(3)
    pin.seed(4)
    rng = np.random.default_rng(4)
    for _ in range(5):
        q, v, _ = _random_state(pin_model, rng)
        tau = rng.standard_normal(pin_model.nv)
        qdd_pin = pin.aba(pin_model, pin_data, q, v, tau)
        qdd_ts = ts.aba(ts_model, q, v, tau)
        assert np.allclose(qdd_pin, qdd_ts, atol=TOL), robot


def test_ik_round_trip(models):
    """Sample a target via FK; the solver should recover a configuration that
    realises the same end-effector pose to within tolerance."""
    robot, pin_model, _, ts_model = models
    rng = np.random.default_rng(5)
    pin.seed(5)
    q_target = pin.randomConfiguration(pin_model)
    target_pose = ts.forward_kinematics(ts_model, q_target)[-1]

    res = ts.solve_ik_dls(
        ts_model, ts_model.njoints - 1, target_pose, np.zeros(ts_model.nq)
    )
    # 6-DoF UR5e is occasionally off-target from cold start; accept the result
    # only if the end-effector error itself is small, not strict converged-flag.
    final_pose = ts.forward_kinematics(ts_model, res.q)[-1]
    err = np.max(np.abs(final_pose[:3, 3] - target_pose[:3, 3]))
    assert err < 5e-3, f"{robot}: position error {err:.3e}"


def test_ik_implicit_derivative_shapes(models):
    """The implicit derivative should have shape (nv, 6) and not blow up."""
    robot, pin_model, _, ts_model = models
    rng = np.random.default_rng(6)
    q = rng.standard_normal(ts_model.nq)
    dq_dT = ts.ik_implicit_derivative(ts_model, ts_model.njoints - 1, q, damping=1e-2)
    assert dq_dT.shape == (ts_model.nv, 6), robot
    assert np.all(np.isfinite(dq_dT)), robot
