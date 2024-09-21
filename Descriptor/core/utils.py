from typing import Any, Dict, NoReturn, Optional, Tuple
import adsk.core

def format_name(input: str):
    translation_table = str.maketrans({':':'_', '-':'_', '.':'_', ' ':'', '(':'{', ')':'}'})
    return input.translate(translation_table)

def rename_if_duplicate(input: str, in_dict: Dict[str, Any]) -> str:
    count = 0
    new_name = input
    while in_dict.get(new_name) is not None:
        new_name = f"{input}_{count}"
        count += 1
    return new_name

def convert_german(str_in):
    translation_table = str.maketrans({'ä': 'ae', 'ö': 'oe', 'ü': 'ue', 'Ä': 'Ae', 'Ö': 'Oe', 'Ü': 'Ue', 'ß': 'ss'})
    return str_in.translate(translation_table)

def log(msg: str, level: Optional[adsk.core.LogLevels] = None) -> None:
    if level is None:
        level = adsk.core.LogLevels.InfoLogLevel
        if msg.startswith("WARN"):
            level = adsk.core.LogLevels.WarningLogLevel
        elif msg.startswith("FATAL") or msg.startswith("ERR"):
            level = adsk.core.LogLevels.ErrorLogLevel
    adsk.core.Application.log(msg, level)
    print(msg)

def fatal(msg: str) -> NoReturn:
    log("FATAL ERROR: " + msg)
    raise RuntimeError(msg)

# Utilities below follow https://github.com/ethz-asl/robotcar_tools/blob/master/python/transform.py
import numpy as np
from math import sin, cos, atan2, sqrt
MATRIX_MATCH_TOLERANCE = 1e-4

def euler_to_so3(rpy: Tuple[float, float, float]):
    """Converts Euler angles to an SO3 rotation matrix.

    Args:
        rpy (list[float]): Euler angles (in radians). Must have three components.

    Returns:
        numpy.matrixlib.defmatrix.matrix: 3x3 SO3 rotation matrix

    Raises:
        ValueError: if `len(rpy) != 3`.

    """
    if len(rpy) != 3:
        raise ValueError("Euler angles must have three components")

    R_x = np.matrix([[1, 0, 0],
                     [0, cos(rpy[0]), -sin(rpy[0])],
                     [0, sin(rpy[0]), cos(rpy[0])]])
    R_y = np.matrix([[cos(rpy[1]), 0, sin(rpy[1])],
                     [0, 1, 0],
                     [-sin(rpy[1]), 0, cos(rpy[1])]])
    R_z = np.matrix([[cos(rpy[2]), -sin(rpy[2]), 0],
                     [sin(rpy[2]), cos(rpy[2]), 0],
                     [0, 0, 1]])
    R_zyx = R_z * R_y * R_x
    return R_zyx


def so3_to_euler(mat: adsk.core.Matrix3D) -> Tuple[float, float, float]:
    """Converts an SO3 rotation matrix to Euler angles

    Args:
        so3: 3x3 rotation matrix

    Returns:
        numpy.matrixlib.defmatrix.matrix: list of Euler angles (size 3)

    Raises:
        ValueError: if so3 is not 3x3
        ValueError: if a valid Euler parametrisation cannot be found

    """
    so3 = np.zeros((3,3))
    for i in range(3):
        for j in range(3):
            so3[i,j] = mat.getCell(i,j)
    roll = atan2(so3[2, 1], so3[2, 2])
    yaw = atan2(so3[1, 0], so3[0, 0])
    denom = sqrt(so3[0, 0] ** 2 + so3[1, 0] ** 2)
    pitch_poss = [atan2(-so3[2, 0], denom), atan2(-so3[2, 0], -denom)]

    rpy1 = (roll, pitch_poss[0], yaw)
    rpy2 = (roll, pitch_poss[1], yaw)
    R1 = euler_to_so3(rpy1)
    R2 = euler_to_so3(rpy2)

    one = (so3 - R1).sum()
    two = (so3 - R2).sum()

    if min(one, two) > MATRIX_MATCH_TOLERANCE:
        raise ValueError("Could not find valid pitch angle")
    return rpy1 if one < two else rpy2