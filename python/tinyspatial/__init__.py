"""tinyspatial — rigid-body kinematics and dynamics, in Python.

A thin nanobind layer on top of the C++20 library. Conventions:

- Spatial vectors are **angular-first**: a twist is ``(omega; v)``, a wrench is
  ``(tau; f)``, both 6-vectors. This is Featherstone's convention and diverges
  from Pinocchio's linear-first ordering — the validation suite converts at the
  boundary.
- ``SO3`` and ``SE3`` are lightweight value types. Build them from NumPy arrays
  via ``SO3.from_quaternion(w, x, y, z)``, ``SO3(rotation_matrix)``, or
  ``SE3.from_matrix(matrix_4x4)``.
- Functions returning poses use 4×4 homogeneous matrices; vectors are 1-D NumPy
  arrays.

See the ``course/14_python_bindings/`` chapters for a tour.
"""

from ._tinyspatial import (  # noqa: F401
    LOCAL,
    LOCAL_WORLD_ALIGNED,
    WORLD,
    Data,
    DlsOptions,
    IkResult,
    JacobianFrame,
    Model,
    NullspaceOptions,
    SE3,
    SO3,
    aba,
    build_model_from_urdf_file,
    compute_jacobian,
    compute_joint_jacobians,
    crba,
    forward_kinematics,
    ik_implicit_derivative,
    rnea,
    rnea_derivatives,
    solve_ik_dls,
    solve_ik_nullspace,
)

__all__ = [
    "LOCAL",
    "LOCAL_WORLD_ALIGNED",
    "WORLD",
    "Data",
    "DlsOptions",
    "IkResult",
    "JacobianFrame",
    "Model",
    "NullspaceOptions",
    "SE3",
    "SO3",
    "aba",
    "build_model_from_urdf_file",
    "compute_jacobian",
    "compute_joint_jacobians",
    "crba",
    "forward_kinematics",
    "ik_implicit_derivative",
    "rnea",
    "rnea_derivatives",
    "solve_ik_dls",
    "solve_ik_nullspace",
]
