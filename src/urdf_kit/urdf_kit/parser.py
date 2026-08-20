"""Parseo de .urdf a RobotDescription vía urdf_parser_py.

Recorre la cadena serie base_link->tip_link componiendo (via matrices
homogéneas) el origin de cada <joint type="fixed"> intermedio en el
siguiente joint móvil -- así el RobotDescription resultante tiene un
JointDescription por articulación móvil, con el origin ya relativo a la
articulación móvil anterior (o a base_link, para la primera), igual que
_JOINT_ORIGINS en poe_adapter.py asumía a mano para el CR5 (que no tiene
fixed joints intermedios -- por eso ahí nunca hizo falta este plegado).

Nota sobre defaults del propio formato URDF: cuando <origin> se omite es
identidad (xyz=0, rpy=0); cuando <axis> se omite el default es (1,0,0), NO
(0,0,1) -- confirmado contra el comportamiento real de urdf_parser_py.
"""

from __future__ import annotations

import math
from pathlib import Path
from typing import List, Tuple, Union

import numpy as np
from urdf_parser_py.urdf import URDF

from shared_kernel import JointDescription, RobotDescription

_MOVABLE_JOINT_TYPES = {"revolute", "continuous", "prismatic"}
_DEFAULT_AXIS = (1.0, 0.0, 0.0)


class UnsupportedUrdfChainError(Exception):
    """No hay una cadena serie válida de base_link a tip_link, o contiene
    un tipo de joint no soportado (floating, planar)."""


def parse_urdf_file(
    path: Union[str, Path], base_link: str, tip_link: str
) -> RobotDescription:
    xml_text = Path(path).read_text()
    return parse_urdf_string(xml_text, base_link, tip_link)


def parse_urdf_string(
    xml_text: str, base_link: str, tip_link: str
) -> RobotDescription:
    robot = URDF.from_xml_string(xml_text)
    joint_names = _serial_chain(robot, base_link, tip_link)
    joints = _fold_fixed_joints(robot, joint_names)

    result = RobotDescription.create(joints, base_link=base_link, tip_link=tip_link)
    if result.is_left():
        raise ValueError(str(result.value))
    return result.value


def _serial_chain(robot: URDF, base_link: str, tip_link: str) -> List[str]:
    """Nombres de joint (móviles y fixed) de base_link a tip_link, en orden.

    URDF.get_chain recorre solo los ancestros de tip_link (cada link tiene
    un único padre) -- lanza KeyError si base_link no es ancestro de
    tip_link (incluida la dirección invertida, o links que no existen).
    """
    try:
        return robot.get_chain(base_link, tip_link, joints=True, links=False, fixed=True)
    except KeyError as error:
        raise UnsupportedUrdfChainError(
            f'No hay una cadena serie de "{base_link}" a "{tip_link}": '
            f"{error} no es alcanzable en esa dirección"
        ) from error


def _fold_fixed_joints(robot: URDF, joint_names: List[str]) -> List[JointDescription]:
    descriptions: List[JointDescription] = []
    accumulated = np.eye(4)

    for joint_name in joint_names:
        joint = robot.joint_map[joint_name]
        xyz, rpy = _joint_origin(joint)
        accumulated = accumulated @ _origin_to_transform(xyz, rpy)

        if joint.joint_type == "fixed":
            continue
        if joint.joint_type not in _MOVABLE_JOINT_TYPES:
            raise UnsupportedUrdfChainError(
                f'Tipo de articulación no soportado: "{joint.joint_type}" en '
                f'"{joint.name}" (solo revolute/continuous/prismatic/fixed)'
            )

        folded_xyz, folded_rpy = _transform_to_origin(accumulated)
        axis = _normalize(joint.axis if joint.axis is not None else _DEFAULT_AXIS)
        descriptions.append(
            JointDescription(
                name=joint.name,
                joint_type=joint.joint_type,
                origin_xyz=folded_xyz,
                origin_rpy=folded_rpy,
                axis=axis,
            )
        )
        accumulated = np.eye(4)

    return descriptions


def _joint_origin(joint) -> Tuple[Tuple[float, float, float], Tuple[float, float, float]]:
    if joint.origin is None:
        return (0.0, 0.0, 0.0), (0.0, 0.0, 0.0)
    xyz = tuple(float(v) for v in joint.origin.xyz) if joint.origin.xyz else (0.0, 0.0, 0.0)
    rpy = tuple(float(v) for v in joint.origin.rpy) if joint.origin.rpy else (0.0, 0.0, 0.0)
    return xyz, rpy


def _normalize(vector: Tuple[float, float, float]) -> Tuple[float, float, float]:
    array = np.array(vector, dtype=float)
    norm = float(np.linalg.norm(array))
    if norm < 1e-9:
        return tuple(array.tolist())  # RobotDescription.create rechazará esto
    return tuple((array / norm).tolist())


def _rpy_to_matrix(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """R = Rz(yaw)·Ry(pitch)·Rx(roll) -- misma convención rpy de ejes fijos
    que poe_adapter.py._rpy_to_matrix (duplicado deliberadamente: urdf_kit
    no debe depender de controller_node)."""
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    rot_x = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    rot_y = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    rot_z = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return rot_z @ rot_y @ rot_x


def _matrix_to_rpy(rotation: np.ndarray) -> Tuple[float, float, float]:
    """Inversa de _rpy_to_matrix. sin(pitch) = -R[2,0]; caso degenerado
    (gimbal lock, |cos(pitch)|~0) resuelto con roll=0."""
    sin_pitch = min(1.0, max(-1.0, -rotation[2, 0]))
    pitch = math.asin(sin_pitch)
    cos_pitch = math.cos(pitch)
    if abs(cos_pitch) > 1e-6:
        roll = math.atan2(rotation[2, 1], rotation[2, 2])
        yaw = math.atan2(rotation[1, 0], rotation[0, 0])
    else:
        roll = 0.0
        yaw = math.atan2(-rotation[0, 1], rotation[1, 1])
    return (roll, pitch, yaw)


def _origin_to_transform(
    xyz: Tuple[float, float, float], rpy: Tuple[float, float, float]
) -> np.ndarray:
    transform = np.eye(4)
    transform[:3, :3] = _rpy_to_matrix(*rpy)
    transform[:3, 3] = xyz
    return transform


def _transform_to_origin(
    transform: np.ndarray,
) -> Tuple[Tuple[float, float, float], Tuple[float, float, float]]:
    xyz = tuple(transform[:3, 3].tolist())
    rpy = _matrix_to_rpy(transform[:3, :3])
    return xyz, rpy
