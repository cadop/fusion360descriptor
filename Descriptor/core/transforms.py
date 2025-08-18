from typing import Tuple
import adsk.core

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

def origin2center_of_mass(inertia, center_of_mass, mass):
    """
    convert the moment of the inertia about the world coordinate into 
    that about center of mass coordinate


    Parameters
    ----------
    moment of inertia about the world coordinate:  [xx, yy, zz, xy, yz, xz]
    center_of_mass: [x, y, z]
    
    
    Returns
    ----------
    moment of inertia about center of mass : [xx, yy, zz, xy, yz, xz]
    """
    x = center_of_mass[0]
    y = center_of_mass[1]
    z = center_of_mass[2]
    translation_matrix = [y**2+z**2, x**2+z**2, x**2+y**2,
                               -x*y,      -y*z,       -x*z]
    return [ round(i - mass*t, 6) for i, t in zip(inertia, translation_matrix)]
