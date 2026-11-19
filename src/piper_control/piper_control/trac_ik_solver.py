"""Adaptador de TRAC-IK para la cadena base_link -> link6 del PiPER."""

from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation
from trac_ik import TracIK


class PiperTracIkSolver:
    """Calcula cinemática directa e inversa del brazo PiPER."""

    def __init__(self, urdf_path, timeout=0.005):
        urdf_path = Path(urdf_path).expanduser()

        if not urdf_path.is_file():
            raise FileNotFoundError(
                f"No se encontró el URDF del PiPER: {urdf_path}"
            )

        self.solver = TracIK(
            base_link_name="base_link",
            tip_link_name="link6",
            urdf_path=str(urdf_path),
            timeout=float(timeout),
            epsilon=1e-5,
            solver_type="Speed",
        )

    def forward(self, joints):
        """Calcula la pose xyz/rpy de link6 desde seis articulaciones."""
        values = np.asarray(joints, dtype=float)

        if values.shape != (6,) or not np.all(np.isfinite(values)):
            raise ValueError(
                "Se necesitan seis posiciones articulares finitas."
            )

        position, rotation_matrix = self.solver.fk(values)
        rpy = Rotation.from_matrix(rotation_matrix).as_euler("xyz")

        return np.concatenate((position, rpy))

    def solve(self, pose_xyz_rpy, seed_joints):
        """Busca posiciones articulares para una pose xyz/rpy de link6."""
        pose = np.asarray(pose_xyz_rpy, dtype=float)
        seed = np.asarray(seed_joints, dtype=float)

        if pose.shape != (6,) or not np.all(np.isfinite(pose)):
            raise ValueError(
                "La pose debe contener seis valores finitos: xyz y rpy."
            )

        if seed.shape != (6,) or not np.all(np.isfinite(seed)):
            raise ValueError(
                "La semilla debe contener seis posiciones finitas."
            )

        rotation_matrix = Rotation.from_euler(
            "xyz",
            pose[3:],
        ).as_matrix()

        result = self.solver.ik(
            pose[:3],
            rotation_matrix,
            seed_jnt_values=seed,
        )

        if result is None:
            return None

        joints = np.asarray(result, dtype=float)

        if joints.shape != (6,) or not np.all(np.isfinite(joints)):
            return None
        
        return joints