# Pinocchio parity — `tinyspatial`

Numeric agreement with **Pinocchio 3.9.0** on each fixture URDF, over **1000** random configurations (fixed seed `0xC0FFEE`).
Tolerance is **1e-10** absolute (CLAUDE.md §11). Pinocchio Jacobians are permuted to angular-first row order before comparison; this is a documented convention difference, not a bug (CLAUDE.md §5).

## Phase 4 — Forward kinematics + Jacobians

| Robot | FK max-abs diff | J max-abs diff (LOCAL) | J max-abs diff (WORLD) | J max-abs diff (LWA) |
| ----- | --------------- | ---------------------- | ---------------------- | -------------------- |
| `simple_arm` | `7.77e-16` | `9.99e-16` | `1.44e-15` | `1.55e-15` |
| `franka_fr3` | `1.22e-15` | `1.21e-15` | `1.33e-15` | `1.33e-15` |
| `ur5e` | `8.88e-16` | `1.22e-15` | `1.33e-15` | `1.33e-15` |
| `so_arm101` | `8.88e-16` | `8.88e-16` | `1.33e-15` | `1.33e-15` |

_Regenerate with `cmake --preset=validation && cmake --build build/validation && .venv/bin/python tests/validation/test_kinematics.py`._
