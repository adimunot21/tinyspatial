"""Type stubs for tinyspatial.

These describe shapes only — runtime dtype is always float64. All matrices /
vectors are NumPy arrays with the indicated shape.
"""
from __future__ import annotations

from enum import Enum
from typing import overload

import numpy as np
from numpy.typing import NDArray

F64 = NDArray[np.float64]

# ---------------------------------------------------------------------------
# Lie groups
# ---------------------------------------------------------------------------

class SO3:
    @overload
    def __init__(self) -> None: ...
    @overload
    def __init__(self, rotation_matrix: F64) -> None: ...
    @staticmethod
    def identity() -> SO3: ...
    @staticmethod
    def from_quaternion(w: float, x: float, y: float, z: float) -> SO3: ...
    @staticmethod
    def exp(omega: F64) -> SO3: ...
    def log(self) -> F64: ...
    def matrix(self) -> F64: ...
    def quaternion(self) -> F64: ...
    def inverse(self) -> SO3: ...
    def act(self, point: F64) -> F64: ...
    def __mul__(self, other: SO3) -> SO3: ...

class SE3:
    @overload
    def __init__(self) -> None: ...
    @overload
    def __init__(self, rotation: SO3, translation: F64) -> None: ...
    @staticmethod
    def identity() -> SE3: ...
    @staticmethod
    def from_matrix(matrix: F64) -> SE3: ...
    @staticmethod
    def exp(xi: F64) -> SE3: ...
    def log(self) -> F64: ...
    def matrix(self) -> F64: ...
    def rotation(self) -> SO3: ...
    def translation(self) -> F64: ...
    def inverse(self) -> SE3: ...
    def adjoint(self) -> F64: ...
    def adjoint_inverse(self) -> F64: ...
    def act(self, point: F64) -> F64: ...
    def __mul__(self, other: SE3) -> SE3: ...

# ---------------------------------------------------------------------------
# Kinematic tree
# ---------------------------------------------------------------------------

class JacobianFrame(Enum):
    LOCAL = 0
    WORLD = 1
    LOCAL_WORLD_ALIGNED = 2

LOCAL: JacobianFrame
WORLD: JacobianFrame
LOCAL_WORLD_ALIGNED: JacobianFrame

class Model:
    @property
    def name(self) -> str: ...
    @property
    def njoints(self) -> int: ...
    @property
    def nq(self) -> int: ...
    @property
    def nv(self) -> int: ...
    @property
    def joint_names(self) -> list[str]: ...
    @property
    def link_names(self) -> list[str]: ...
    @property
    def parent(self) -> list[int]: ...
    @property
    def idx_q(self) -> list[int]: ...
    @property
    def idx_v(self) -> list[int]: ...
    def joint_id(self, name: str) -> int: ...

class Data:
    def __init__(self, model: Model) -> None: ...
    def pose_in_world(self, joint_id: int) -> F64: ...
    def pose_in_parent(self, joint_id: int) -> F64: ...

def build_model_from_urdf_file(path: str) -> Model: ...

# ---------------------------------------------------------------------------
# Kinematics
# ---------------------------------------------------------------------------

def forward_kinematics(model: Model, q: F64) -> list[F64]: ...
def compute_jacobian(
    model: Model,
    q: F64,
    link_id: int,
    frame: JacobianFrame = ...,
) -> F64: ...
def compute_joint_jacobians(model: Model, q: F64) -> list[F64]: ...

# ---------------------------------------------------------------------------
# Dynamics
# ---------------------------------------------------------------------------

def rnea(
    model: Model,
    q: F64,
    v: F64,
    a: F64,
    gravity: F64 = ...,
) -> F64: ...
def crba(model: Model, q: F64) -> F64: ...
def aba(
    model: Model,
    q: F64,
    v: F64,
    tau: F64,
    gravity: F64 = ...,
) -> F64: ...
def rnea_derivatives(
    model: Model,
    q: F64,
    v: F64,
    a: F64,
    gravity: F64 = ...,
) -> tuple[F64, F64, F64]: ...

# ---------------------------------------------------------------------------
# Inverse kinematics
# ---------------------------------------------------------------------------

class DlsOptions:
    def __init__(self) -> None: ...
    max_iters: int
    tolerance: float
    damping: float
    step_size: float

class NullspaceOptions:
    def __init__(self) -> None: ...
    max_iters: int
    tolerance: float
    damping: float
    step_size: float
    secondary_gain: float

class IkResult:
    @property
    def q(self) -> F64: ...
    @property
    def error(self) -> F64: ...
    @property
    def iterations(self) -> int: ...
    @property
    def converged(self) -> bool: ...

def solve_ik_dls(
    model: Model,
    link_id: int,
    target_in_world: F64,
    q_init: F64,
    options: DlsOptions | None = None,
) -> IkResult: ...
def solve_ik_nullspace(
    model: Model,
    link_id: int,
    target_in_world: F64,
    q_init: F64,
    q_rest: F64,
    options: NullspaceOptions | None = None,
) -> IkResult: ...
def ik_implicit_derivative(
    model: Model,
    link_id: int,
    q_star: F64,
    damping: float = 1e-2,
) -> F64: ...
