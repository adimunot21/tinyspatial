"""Pinocchio cross-validation for forward kinematics and the geometric Jacobian.

For each fixture URDF, sample N=1000 random configurations from a fixed seed,
compute the per-joint world-frame poses (FK) and the last-joint Jacobian in
three reference frames in both libraries, and assert agreement to 1e-10.

Conventions worth flagging:

- **Joint indexing.** Pinocchio includes a "universe" joint at index 0 and lists
  movable joints at indices 1..N. tinyspatial lists every joint (including
  fixed ones) from 0. For our fixtures the movable joints come first, so
  pin index `k` (k >= 1) maps to our index `k - 1`.

- **Jacobian row order.** Pinocchio's rows are linear-first (v then ω);
  tinyspatial's are angular-first (ω then v). We apply the 6×6 permutation
  `[[0, I3], [I3, 0]]` to Pinocchio's Jacobian before comparing.

Run via `python tests/validation/test_kinematics.py` after building the
binding with `cmake --preset=validation && cmake --build build/validation`.
Generates / updates `docs/PINOCCHIO_PARITY.md`.
"""

from __future__ import annotations

import os
import pathlib
import sys
from dataclasses import dataclass

import numpy as np
import pinocchio as pin

REPO = pathlib.Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / "python"))
import tinyspatial as ts  # noqa: E402  (after sys.path manipulation)

NUM_SAMPLES = 1000
TOLERANCE = 1e-10
SEED = 0xC0FFEE

ROBOTS = ["simple_arm", "franka_fr3", "ur5e", "so_arm101"]

# Swap the 3-row blocks: turns linear-first into angular-first (or back).
P_ROW_SWAP = np.zeros((6, 6))
P_ROW_SWAP[:3, 3:] = np.eye(3)
P_ROW_SWAP[3:, :3] = np.eye(3)

FRAMES = [
    ("LOCAL", pin.LOCAL, ts.LOCAL),
    ("WORLD", pin.WORLD, ts.WORLD),
    ("LOCAL_WORLD_ALIGNED", pin.LOCAL_WORLD_ALIGNED, ts.LOCAL_WORLD_ALIGNED),
]


@dataclass
class Result:
    fk_max_diff: float = 0.0
    j_max_diff: dict[str, float] = None
    rnea_max_diff: float = 0.0
    crba_max_diff: float = 0.0
    aba_max_diff: float = 0.0
    dtau_dq_max_diff: float = 0.0
    dtau_dv_max_diff: float = 0.0
    dtau_da_max_diff: float = 0.0

    def __post_init__(self):
        if self.j_max_diff is None:
            self.j_max_diff = {name: 0.0 for name, _, _ in FRAMES}


def validate(robot: str) -> Result:
    urdf = str(REPO / "data" / "robots" / f"{robot}.urdf")
    pin_model = pin.buildModelFromUrdf(urdf)
    pin_data = pin_model.createData()
    ts_model = ts.build_model_from_urdf_file(urdf)

    # Sanity: nq / nv must match between the two libraries.
    assert pin_model.nq == ts_model.nq, f"{robot}: nq mismatch"
    assert pin_model.nv == ts_model.nv, f"{robot}: nv mismatch"
    n_movable = pin_model.njoints - 1  # excludes the universe joint at idx 0

    # Deterministic random configurations come from pinocchio (it handles
    # quaternion normalisation for floating joints; none of our fixtures have
    # them, but the same harness will scale to humanoid URDFs later).
    pin.seed(SEED)

    result = Result()
    rng = np.random.default_rng(SEED)
    for _ in range(NUM_SAMPLES):
        q = pin.randomConfiguration(pin_model)
        v = rng.standard_normal(pin_model.nv)
        a = rng.standard_normal(pin_model.nv)

        pin.forwardKinematics(pin_model, pin_data, q)
        pin.computeJointJacobians(pin_model, pin_data, q)
        ts_poses = ts.forward_kinematics(ts_model, q)

        # FK per movable joint.
        for k in range(1, pin_model.njoints):
            pin_pose = pin_data.oMi[k].homogeneous
            ts_pose = ts_poses[k - 1]
            diff = float(np.max(np.abs(pin_pose - ts_pose)))
            if diff > result.fk_max_diff:
                result.fk_max_diff = diff

        # Jacobian at the last movable joint, all three frames.
        last_pin = pin_model.njoints - 1
        last_ts = last_pin - 1
        for name, pin_frame, ts_frame in FRAMES:
            j_pin_linear_first = pin.getJointJacobian(pin_model, pin_data, last_pin, pin_frame)
            j_pin = P_ROW_SWAP @ j_pin_linear_first  # angular-first
            j_ts = ts.compute_jacobian(ts_model, q, last_ts, ts_frame)
            diff = float(np.max(np.abs(j_pin - j_ts)))
            if diff > result.j_max_diff[name]:
                result.j_max_diff[name] = diff

        # RNEA inverse dynamics. Pinocchio's gravity is a Motion stored on the
        # model; we override with the same vector for cross-check determinism.
        gravity = np.array([0.0, 0.0, -9.81])
        # Pinocchio's model.gravity defaults to that already; set explicitly
        # in case it was mutated.
        pin_model.gravity.linear = gravity
        pin_model.gravity.angular = np.zeros(3)
        tau_pin = pin.rnea(pin_model, pin_data, q, v, a)
        tau_ts = ts.rnea(ts_model, q, v, a, gravity)
        diff = float(np.max(np.abs(tau_pin - tau_ts)))
        if diff > result.rnea_max_diff:
            result.rnea_max_diff = diff

        # CRBA mass matrix. Pinocchio fills only the upper triangle of M;
        # symmetrise before comparison.
        m_pin = pin.crba(pin_model, pin_data, q)
        m_pin = np.triu(m_pin) + np.triu(m_pin, 1).T
        m_ts = ts.crba(ts_model, q)
        diff = float(np.max(np.abs(m_pin - m_ts)))
        if diff > result.crba_max_diff:
            result.crba_max_diff = diff

        # ABA forward dynamics. Use a fresh random τ so it isn't trivially
        # consistent with RNEA above.
        tau_in = rng.standard_normal(pin_model.nv)
        qdd_pin = pin.aba(pin_model, pin_data, q, v, tau_in)
        qdd_ts = ts.aba(ts_model, q, v, tau_in, gravity)
        diff = float(np.max(np.abs(qdd_pin - qdd_ts)))
        if diff > result.aba_max_diff:
            result.aba_max_diff = diff

        # Analytical RNEA derivatives. Pinocchio fills data.dtau_dq, data.dtau_dv;
        # ∂τ/∂a is the mass matrix M (only upper triangle filled, like crba).
        pin.computeRNEADerivatives(pin_model, pin_data, q, v, a)
        dtau_dq_pin = np.array(pin_data.dtau_dq)
        dtau_dv_pin = np.array(pin_data.dtau_dv)
        dtau_da_pin = np.array(pin_data.M)
        dtau_da_pin = np.triu(dtau_da_pin) + np.triu(dtau_da_pin, 1).T

        dtau_dq_ts, dtau_dv_ts, dtau_da_ts = ts.rnea_derivatives(ts_model, q, v, a, gravity)

        d_q = float(np.max(np.abs(dtau_dq_pin - dtau_dq_ts)))
        d_v = float(np.max(np.abs(dtau_dv_pin - dtau_dv_ts)))
        d_a = float(np.max(np.abs(dtau_da_pin - dtau_da_ts)))
        if d_q > result.dtau_dq_max_diff:
            result.dtau_dq_max_diff = d_q
        if d_v > result.dtau_dv_max_diff:
            result.dtau_dv_max_diff = d_v
        if d_a > result.dtau_da_max_diff:
            result.dtau_da_max_diff = d_a
    return result


def write_parity_table(results: dict[str, Result]) -> str:
    rows = []
    rows.append("# Pinocchio parity — `tinyspatial`")
    rows.append("")
    rows.append(
        "Numeric agreement with **Pinocchio 3.9.0** on each fixture URDF, "
        f"over **{NUM_SAMPLES}** random configurations (fixed seed `0x{SEED:X}`)."
    )
    rows.append(
        "Tolerance is **1e-10** absolute (CLAUDE.md §11). Pinocchio Jacobians are "
        "permuted to angular-first row order before comparison; this is a documented "
        "convention difference, not a bug (CLAUDE.md §5)."
    )
    rows.append("")
    rows.append("## Forward kinematics + Jacobians (Phase 4)")
    rows.append("")
    rows.append(
        "| Robot | FK | J (LOCAL) | J (WORLD) | J (LWA) |"
    )
    rows.append(
        "| ----- | -- | --------- | --------- | ------- |"
    )
    for robot, r in results.items():
        rows.append(
            f"| `{robot}` | `{r.fk_max_diff:.2e}` | "
            f"`{r.j_max_diff['LOCAL']:.2e}` | `{r.j_max_diff['WORLD']:.2e}` | "
            f"`{r.j_max_diff['LOCAL_WORLD_ALIGNED']:.2e}` |"
        )
    rows.append("")
    rows.append("## Dynamics (Phase 5)")
    rows.append("")
    rows.append("| Robot | RNEA (inverse) | CRBA (mass matrix) | ABA (forward) |")
    rows.append("| ----- | -------------- | ------------------ | ------------- |")
    for robot, r in results.items():
        rows.append(
            f"| `{robot}` | `{r.rnea_max_diff:.2e}` | "
            f"`{r.crba_max_diff:.2e}` | `{r.aba_max_diff:.2e}` |"
        )
    rows.append("")
    rows.append("## Analytical derivatives (Phase 6)")
    rows.append("")
    rows.append("| Robot | ∂τ/∂q | ∂τ/∂v | ∂τ/∂a (= M) |")
    rows.append("| ----- | ----- | ----- | ----------- |")
    for robot, r in results.items():
        rows.append(
            f"| `{robot}` | `{r.dtau_dq_max_diff:.2e}` | "
            f"`{r.dtau_dv_max_diff:.2e}` | `{r.dtau_da_max_diff:.2e}` |"
        )
    rows.append("")
    rows.append(
        "_Regenerate with `cmake --preset=validation && cmake --build build/validation && "
        ".venv/bin/python tests/validation/test_kinematics.py`._"
    )
    rows.append("")
    return "\n".join(rows)


def main() -> int:
    results = {}
    any_failed = False
    for robot in ROBOTS:
        print(f"=== {robot} ===")
        r = validate(robot)
        print(f"  FK max diff: {r.fk_max_diff:.3e}")
        for name in r.j_max_diff:
            print(f"  J ({name}) max diff: {r.j_max_diff[name]:.3e}")
        print(f"  RNEA max diff: {r.rnea_max_diff:.3e}")
        print(f"  CRBA max diff: {r.crba_max_diff:.3e}")
        print(f"  ABA  max diff: {r.aba_max_diff:.3e}")
        print(f"  ∂τ/∂q max diff: {r.dtau_dq_max_diff:.3e}")
        print(f"  ∂τ/∂v max diff: {r.dtau_dv_max_diff:.3e}")
        print(f"  ∂τ/∂a max diff: {r.dtau_da_max_diff:.3e}")
        if (
            r.fk_max_diff > TOLERANCE
            or any(v > TOLERANCE for v in r.j_max_diff.values())
            or r.rnea_max_diff > TOLERANCE
            or r.crba_max_diff > TOLERANCE
            or r.aba_max_diff > TOLERANCE
            or r.dtau_dq_max_diff > TOLERANCE
            or r.dtau_dv_max_diff > TOLERANCE
            or r.dtau_da_max_diff > TOLERANCE
        ):
            any_failed = True
            print(f"  ** EXCEEDS {TOLERANCE:.0e} **")
        results[robot] = r

    out_path = REPO / "docs" / "PINOCCHIO_PARITY.md"
    out_path.write_text(write_parity_table(results))
    print(f"\nWrote {out_path.relative_to(REPO)}")

    return 1 if any_failed else 0


if __name__ == "__main__":
    sys.exit(main())
