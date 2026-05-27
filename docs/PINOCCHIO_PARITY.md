# Pinocchio parity — `tinyspatial`

Numeric agreement with **Pinocchio 3.9.0** on each fixture URDF, over **1000** random configurations (fixed seed `0xC0FFEE`).
Tolerance is **1e-10** absolute (CLAUDE.md §11). Pinocchio Jacobians are permuted to angular-first row order before comparison; this is a documented convention difference, not a bug (CLAUDE.md §5).

## Forward kinematics + Jacobians (Phase 4)

| Robot | FK | J (LOCAL) | J (WORLD) | J (LWA) |
| ----- | -- | --------- | --------- | ------- |
| `simple_arm` | `7.77e-16` | `9.99e-16` | `1.44e-15` | `1.55e-15` |
| `franka_fr3` | `1.22e-15` | `1.21e-15` | `1.33e-15` | `1.33e-15` |
| `ur5e` | `8.88e-16` | `1.22e-15` | `1.33e-15` | `1.33e-15` |
| `so_arm101` | `8.88e-16` | `8.88e-16` | `1.33e-15` | `1.33e-15` |

## Dynamics (Phase 5)

| Robot | RNEA (inverse) | CRBA (mass matrix) | ABA (forward) |
| ----- | -------------- | ------------------ | ------------- |
| `simple_arm` | `3.55e-15` | `1.78e-15` | `7.11e-15` |
| `franka_fr3` | `7.11e-14` | `5.33e-15` | `9.38e-13` |
| `ur5e` | `4.97e-14` | `3.11e-15` | `4.55e-13` |
| `so_arm101` | `7.77e-16` | `2.43e-17` | `5.46e-12` |

## Analytical derivatives (Phase 6)

| Robot | ∂τ/∂q | ∂τ/∂v | ∂τ/∂a (= M) |
| ----- | ----- | ----- | ----------- |
| `simple_arm` | `5.33e-15` | `3.55e-15` | `1.78e-15` |
| `franka_fr3` | `1.28e-13` | `1.42e-14` | `6.22e-15` |
| `ur5e` | `4.97e-14` | `8.88e-15` | `3.55e-15` |
| `so_arm101` | `8.33e-16` | `4.16e-17` | `2.43e-17` |

_Regenerate with `cmake --preset=validation && cmake --build build/validation && .venv/bin/python tests/validation/test_kinematics.py`._
