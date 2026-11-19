"""Comprobación FK -> IK -> FK del solver de PiPER."""

from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

from piper_control.trac_ik_solver import PiperTracIkSolver


# Modelo generado desde el Xacro de Gazebo.
urdf_path = Path.home() / "piper_ws/modelos/piper_gazebo.urdf"

# Crear el adaptador de TRAC-IK.
solver = PiperTracIkSolver(urdf_path)

# Postura inicial en radianes, en el orden joint1 a joint6.
articulaciones_iniciales = np.array([
    0.0,
    0.4,
    -0.8,
    0.0,
    0.4,
    0.0,
])

print("\n1. Articulaciones iniciales [rad]")
print(articulaciones_iniciales)

# FK: articulaciones -> pose de link6.
pose_inicial = solver.forward(articulaciones_iniciales)

print("\n2. Pose calculada con FK [x, y, z, roll, pitch, yaw]")
print(pose_inicial)

# IK: pose de link6 -> posible solución articular.
solucion = solver.solve(pose_inicial, articulaciones_iniciales)

if solucion is None:
    print("\nTRAC-IK no encontró una solución para esta pose.")
else:
    print("\n3. Solución encontrada con IK [rad]")
    print(solucion)

    # Calcular la pose que produce la solución de IK.
    pose_recuperada = solver.forward(solucion)

    print("\n4. Pose recuperada con FK")
    print(pose_recuperada)

    # Comparar posiciones en metros.
    error_posicion = np.linalg.norm(
        pose_recuperada[:3] - pose_inicial[:3]
    )

    # Comparar orientaciones mediante la rotación entre ambas.
    rotacion_inicial = Rotation.from_euler("xyz", pose_inicial[3:])
    rotacion_recuperada = Rotation.from_euler("xyz", pose_recuperada[3:])
    error_orientacion = (
        rotacion_inicial.inv() * rotacion_recuperada
    ).magnitude()

    print(f"\nError de posición: {error_posicion:.8f} m")
    print(f"Error de orientación: {error_orientacion:.8f} rad")

# Probar una posición alejada del espacio de trabajo del PiPER.
pose_inalcanzable = np.array([
    5.0,
    0.0,
    5.0,
    0.0,
    0.0,
    0.0,
])

resultado = solver.solve(
    pose_inalcanzable,
    articulaciones_iniciales,
)

print("\n5. Resultado para el objetivo alejado")
print(resultado)
