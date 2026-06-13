# Glossary

Terms and notation used throughout the course, with the chapter that introduces
each and the corresponding library symbol where one exists.

| Term | Definition | Notation | Chapter | Library symbol |
| ---- | ---------- | -------- | ------- | -------------- |
| **SO(3)** | The group of 3-D rotations. A rotation has determinant $+1$ and preserves lengths and handedness. | $R \in SO(3)$ | 03–04 | `SO3T`, `SO3` |
| **SE(3)** | The group of rigid transforms: a rotation paired with a translation. | $T = (R, t)$ | 03–04 | `SE3T`, `SE3` |
| **Quaternion** | A 4-number representation of a rotation; the library's storage form, canonicalised to unit norm with $w \ge 0$. | $q = (w, \mathbf{v})$ | 03 | `SO3T::quaternion` |
| **Axis–angle / rotation vector** | A rotation as an axis $\hat u$ scaled by an angle $\theta$; the tangent-space coordinate of a rotation. | $\omega = \theta\,\hat u$ | 03–04 | `SO3T::log` |
| **Hat / skew operator** | Maps a 3-vector to the skew-symmetric matrix implementing the cross product, $[\mathbf{v}]_\times\,\mathbf{w} = \mathbf{v}\times\mathbf{w}$. | $[\mathbf{v}]_\times$ | 03 | `skew`, `unskew` |
| **Exponential map** | Sends a tangent vector (constant velocity, held for unit time) to the group element it produces. | $\exp(\omega)$ | 04 | `SO3T::exp`, `SE3T::exp` |
| **Logarithm map** | Inverse of `exp`: recovers the tangent vector that produces a given group element. | $\log(R)$ | 04 | `SO3T::log`, `SE3T::log` |
| **Left / right Jacobian** | The linear map relating a tangent-space perturbation to the resulting on-manifold change, measured on the left or right. | $J_l,\ J_r$ | 04 | `left_jacobian`, `right_jacobian` |
| **Adjoint** | The $6\times6$ map that re-expresses a twist from one frame in another. | $\mathrm{Ad}_T$ | 04–05 | `SE3T::adjoint` |
| **Twist (spatial velocity)** | A 6-vector bundling angular and linear velocity, angular-first. Transforms under the adjoint. | $m = (\omega, v)$ | 05 | `MotionT`, `Motion` |
| **Wrench (spatial force)** | A 6-vector bundling moment and linear force. Transforms under the dual adjoint. | $f = (\tau, F)$ | 05 | `ForceT`, `Force` |
| **Spatial cross product** | The Lie bracket of $\mathfrak{se}(3)$; the velocity-product operator in the dynamics recurrences. | $m\,\times,\ m\,\times^*$ | 04–05 | `cross_motion`, `cross_force` |
| **Plücker transform** | The change-of-frame map for spatial vectors; the motion form is the adjoint, the force form its inverse-transpose. | $X,\ X^{*}$ | 05 | `force_plucker` |
| **Spatial inertia** | A rigid body's mass distribution: mass, centre of mass, and rotational inertia, mapping a twist to a momentum. | $I_s$ | 05 | `SpatialInertiaT` |
| **Angular-first ordering** | The convention placing angular components in indices 0–2 and linear in 3–5. Diverges from Pinocchio's linear-first order. | $(\omega; v)$ | 05 | — |
| **Kinematic tree** | The robot as a topologically-ordered set of links connected by joints, each with a single parent. | — | 06 | `ModelT`, `Model` |
| **Configuration / velocity** | The joint-space position vector ($n_q$ coordinates) and its time-derivative ($n_v$ coordinates). | $q,\ \dot q$ | 06 | `Model::nq`, `Model::nv` |
| **Motion subspace** | The $6 \times n_v$ matrix $S$ mapping a joint's velocity coordinates to the spatial velocity it produces. | $S_i$ | 06, 10 | `Model::motion_subspace` |
| **URDF** | The XML format describing a robot's links, joints, and inertias. | — | 07 | `build_model_from_urdf_file` |
| **Forward kinematics** | The map from configuration to every link's pose. | $q \mapsto T_i(q)$ | 08 | `forward_kinematics` |
| **Jacobian** | The configuration-dependent linear map from joint velocity to end-effector spatial velocity. | $J(q)$ | 09 | `compute_jacobian` |
| **Singularity** | A configuration where $J$ loses rank and the end-effector cannot move in some direction. | $\det J = 0$ | 09 | — |
| **RNEA** | Recursive Newton–Euler: inverse dynamics, computing the torques for a desired acceleration. | $\tau = \mathrm{ID}(q,\dot q,\ddot q)$ | 10 | `rnea` |
| **CRBA** | Composite-Rigid-Body Algorithm: the joint-space mass matrix $M(q)$. | $M(q)$ | 11 | `crba` |
| **ABA** | Articulated-Body Algorithm: forward dynamics, the acceleration produced by given torques. | $\ddot q = \mathrm{FD}(q,\dot q,\tau)$ | 11 | `aba` |
| **Analytical derivatives** | Closed-form partials $\partial\tau/\partial q$, $\partial\tau/\partial v$, $\partial\tau/\partial a$ in $O(N^2)$. | $\partial\tau/\partial\cdot$ | 10b | `rnea_derivatives` |
| **DLS IK** | Damped-least-squares inverse kinematics: an iterative, singularity-robust solver. | — | 12 | `solve_ik_dls` |
| **Null-space control** | Using redundant degrees of freedom for a secondary objective without disturbing the primary task. | — | 12 | `solve_ik_nullspace` |
| **Implicit-function derivative** | The analytical $\partial q^{*}/\partial T^{*}$ of an IK solution, via the implicit function theorem. | $\partial q^{*}/\partial T^{*}$ | 13 | `ik_implicit_derivative` |
| **Jet (dual number)** | A forward-mode autodiff scalar carrying a value and its partial derivatives, so any templated algorithm becomes differentiable. | $\mathrm{Jet}\langle N\rangle$ | 13 | `Jet`, `model_cast` |
