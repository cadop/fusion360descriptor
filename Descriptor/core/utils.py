from typing import Any, Dict, NoReturn, Optional, Tuple
import adsk.core

LOG_DEBUG = True

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

viewport: Optional[adsk.core.Viewport] = None

def log(msg: str, level: Optional[adsk.core.LogLevels] = None) -> None:
    if not LOG_DEBUG and msg.startswith("DEBUG"):
        return
    if level is None:
        level = adsk.core.LogLevels.InfoLogLevel
        if msg.startswith("WARN"):
            level = adsk.core.LogLevels.WarningLogLevel
        elif msg.startswith("FATAL") or msg.startswith("ERR"):
            level = adsk.core.LogLevels.ErrorLogLevel
    adsk.core.Application.log(msg, level)
    if viewport is not None:
        viewport.refresh()
    print(msg)

def fatal(msg: str) -> NoReturn:
    log("FATAL ERROR: " + msg)
    raise RuntimeError(msg)

import numpy as np
from scipy.spatial.transform import Rotation   

def so3_to_euler(mat: adsk.core.Matrix3D) -> Tuple[float, float, float]:
    """Converts an SO3 rotation matrix to Euler angles

    Args:
        so3: Matrix3D coordinate transform

    Returns:
        tuple of Euler angles (size 3)

    """
    so3 = np.zeros((3,3))
    for i in range(3):
        for j in range(3):
            so3[i,j] = mat.getCell(i,j)
    ### first transform the matrix to euler angles
    r =  Rotation.from_matrix(so3)
    yaw, pitch, roll = r.as_euler("ZYX", degrees=False)
    return (float(roll), float(pitch), float(yaw))

def mat_str(m: adsk.core.Matrix3D) -> str:
    return f"xyz={m.translation.asArray()} rpy={so3_to_euler(m)}"