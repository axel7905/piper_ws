"""Transformaciones entre link6 y el TCP del PiPER."""

import numpy as np
from scipy.spatial.transform import Rotation


# Desplazamiento fijo desde link6 hasta el TCP, en metros.
LINK6_A_TCP = np.array([0.0, 0.0, 0.1358])


def leer_pose(pose):
    """Separa una pose [x, y, z, roll, pitch, yaw]."""
    valores = np.asarray(pose, dtype=float)

    if valores.shape != (6,) or not np.all(np.isfinite(valores)):
        raise ValueError(
            "La pose debe contener seis valores finitos: xyz y rpy."
        )

    posicion = valores[:3]
    orientacion = Rotation.from_euler("xyz", valores[3:])

    return posicion, orientacion


def construir_pose(posicion, orientacion):
    """Construye una pose xyz/rpy a partir de posición y rotación."""
    rpy = orientacion.as_euler("xyz")
    return np.concatenate((posicion, rpy))


def link6_a_tcp(pose_link6):
    """Calcula la pose del TCP respecto a la base."""
    posicion_link6, orientacion_link6 = leer_pose(pose_link6)

    # El desplazamiento fijo está expresado en los ejes de link6.
    desplazamiento_en_base = orientacion_link6.apply(LINK6_A_TCP)

    posicion_tcp = posicion_link6 + desplazamiento_en_base

    # En este modelo, link6 y TCP no tienen rotación fija entre sí.
    return construir_pose(posicion_tcp, orientacion_link6)


def tcp_a_link6(pose_tcp):
    """Calcula la pose de link6 que corresponde a una pose del TCP."""
    posicion_tcp, orientacion_tcp = leer_pose(pose_tcp)

    desplazamiento_en_base = orientacion_tcp.apply(LINK6_A_TCP)

    posicion_link6 = posicion_tcp - desplazamiento_en_base

    return construir_pose(posicion_link6, orientacion_tcp)


def incrementar_tcp(pose_tcp, desplazamiento_local, giro_local):
    """Aplica un incremento expresado en los ejes actuales del TCP.

    desplazamiento_local contiene [dx, dy, dz] en metros.
    giro_local contiene [droll, dpitch, dyaw] en radianes.
    """
    posicion_tcp, orientacion_tcp = leer_pose(pose_tcp)

    desplazamiento = np.asarray(desplazamiento_local, dtype=float)
    giro = np.asarray(giro_local, dtype=float)

    if desplazamiento.shape != (3,) or not np.all(np.isfinite(desplazamiento)):
        raise ValueError(
            "El desplazamiento debe contener tres valores finitos."
        )

    if giro.shape != (3,) or not np.all(np.isfinite(giro)):
        raise ValueError(
            "El giro debe contener tres valores finitos."
        )

    # Convertir el desplazamiento local a los ejes de la base.
    desplazamiento_en_base = orientacion_tcp.apply(desplazamiento)
    nueva_posicion = posicion_tcp + desplazamiento_en_base

    # Aplicar el giro respecto a los ejes actuales de la herramienta.
    rotacion_incremental = Rotation.from_euler("xyz", giro)
    nueva_orientacion = orientacion_tcp * rotacion_incremental

    return construir_pose(nueva_posicion, nueva_orientacion)