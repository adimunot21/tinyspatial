"""Python-side timing of tinyspatial vs Pinocchio on the fixture URDFs.

Why Python? A C++ benchmark would need to link Pinocchio's C++ surface — which
pulls in Boost (forbidden by CLAUDE.md s8) and forces an ABI dance with the
cmeel-shipped libstdc++. From Python both libraries are already installed and
the *ratio* between them is the same up to a fixed per-call overhead. The
absolute timings include the binding-layer cost both ways, which is what
real users see.

Output: a Markdown table that gets pasted into ``docs/BENCHMARKS.md``.

Run from the repo root:
    python python/tools/benchmark_vs_pinocchio.py
"""
from __future__ import annotations

import pathlib
import sys
import time
from dataclasses import dataclass, field

import numpy as np
import pinocchio as pin

REPO = pathlib.Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / "python"))
import tinyspatial as ts  # noqa: E402

ROBOTS = ["simple_arm", "franka_fr3", "ur5e", "so_arm101"]
N_WARMUP = 50
N_TIMED = 2000
P_ROW_SWAP = np.zeros((6, 6))
P_ROW_SWAP[:3, 3:] = np.eye(3)
P_ROW_SWAP[3:, :3] = np.eye(3)


@dataclass
class Timing:
    ts_ns: float = 0.0
    pin_ns: float = 0.0

    @property
    def ratio(self) -> float:
        return self.ts_ns / self.pin_ns if self.pin_ns > 0 else float("inf")


def _time(callable_, n: int) -> float:
    """Return median ns per call over ``n`` repetitions."""
    t0 = time.perf_counter_ns()
    for _ in range(n):
        callable_()
    elapsed = time.perf_counter_ns() - t0
    return elapsed / n


def bench_one(robot: str) -> dict[str, Timing]:
    urdf = str(REPO / "data" / "robots" / f"{robot}.urdf")
    pin_model = pin.buildModelFromUrdf(urdf)
    pin_data = pin_model.createData()
    pin_model.gravity.linear = np.array([0.0, 0.0, -9.81])
    pin_model.gravity.angular = np.zeros(3)
    ts_model = ts.build_model_from_urdf_file(urdf)

    rng = np.random.default_rng(0)
    pin.seed(0)
    q = pin.randomConfiguration(pin_model)
    v = rng.standard_normal(pin_model.nv)
    a = rng.standard_normal(pin_model.nv)
    tau = rng.standard_normal(pin_model.nv)
    g = np.array([0.0, 0.0, -9.81])
    last_pin = pin_model.njoints - 1
    last_ts = last_pin - 1

    # Warm up to settle the binding layer.
    for _ in range(N_WARMUP):
        ts.forward_kinematics(ts_model, q)
        pin.forwardKinematics(pin_model, pin_data, q)

    results: dict[str, Timing] = {}

    results["FK"] = Timing(
        ts_ns=_time(lambda: ts.forward_kinematics(ts_model, q), N_TIMED),
        pin_ns=_time(lambda: pin.forwardKinematics(pin_model, pin_data, q), N_TIMED),
    )

    results["Jacobian"] = Timing(
        ts_ns=_time(
            lambda: ts.compute_jacobian(ts_model, q, last_ts, ts.LOCAL), N_TIMED
        ),
        pin_ns=_time(
            lambda: pin.getJointJacobian(pin_model, pin_data, last_pin, pin.LOCAL),
            N_TIMED,
        ),
    )

    results["RNEA"] = Timing(
        ts_ns=_time(lambda: ts.rnea(ts_model, q, v, a, g), N_TIMED),
        pin_ns=_time(lambda: pin.rnea(pin_model, pin_data, q, v, a), N_TIMED),
    )

    results["CRBA"] = Timing(
        ts_ns=_time(lambda: ts.crba(ts_model, q), N_TIMED),
        pin_ns=_time(lambda: pin.crba(pin_model, pin_data, q), N_TIMED),
    )

    results["ABA"] = Timing(
        ts_ns=_time(lambda: ts.aba(ts_model, q, v, tau, g), N_TIMED),
        pin_ns=_time(lambda: pin.aba(pin_model, pin_data, q, v, tau), N_TIMED),
    )

    results["RNEA derivatives"] = Timing(
        ts_ns=_time(lambda: ts.rnea_derivatives(ts_model, q, v, a, g), N_TIMED),
        pin_ns=_time(
            lambda: pin.computeRNEADerivatives(pin_model, pin_data, q, v, a), N_TIMED
        ),
    )

    return results


def format_table(all_results: dict[str, dict[str, Timing]]) -> str:
    rows = [
        "| Algorithm | Robot | tinyspatial (ns) | Pinocchio (ns) | ratio (ts/pin) |",
        "| --------- | ----- | ----------------:| --------------:| --------------:|",
    ]
    algorithms = ["FK", "Jacobian", "RNEA", "CRBA", "ABA", "RNEA derivatives"]
    for algo in algorithms:
        for robot, results in all_results.items():
            t = results[algo]
            rows.append(
                f"| {algo} | `{robot}` | {t.ts_ns:8.0f} | {t.pin_ns:8.0f} | **{t.ratio:5.2f}x** |"
            )
    return "\n".join(rows)


def main() -> None:
    all_results = {}
    for robot in ROBOTS:
        print(f"benchmarking {robot}...", flush=True)
        all_results[robot] = bench_one(robot)
    print()
    print(format_table(all_results))


if __name__ == "__main__":
    main()
