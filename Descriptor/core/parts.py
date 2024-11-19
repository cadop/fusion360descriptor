# -*- coding: utf-8 -*-
"""
Created on Sun May 12 20:17:17 2019

@author: syuntoku

Modified by cadop Dec 19 2021
"""

import math
from typing import List, Optional, Sequence
from xml.etree.ElementTree import Element, SubElement
from xml.etree import ElementTree
from xml.dom import minidom
from . import utils

MAX_ROUND_TO_ZERO = 1e-8 # in mm, radians
HALF_PI = math.pi * 0.5
def _round(val: float, unit: float) -> float:
    units = val/unit
    r = round(units)
    if abs(units - r) <= MAX_ROUND_TO_ZERO:
        return r * unit
    return val

def round_mm(val:float) -> float:
    return _round(val, 0.001)

def round_rads(val:float) -> float:
    return _round(val, HALF_PI)
class Joint:

    # Defaults for all joints. Need be be floats, not ints
    effort_limit = 100.0
    vel_limit = 100.0

    def __init__(self, name: str, xyz: Sequence[float], rpy: Sequence[float], axis: Sequence[float], parent: str, child:str, joint_type: str, upper_limit: float, lower_limit: float):
        """
        Attributes
        ----------
        name: str
            name of the joint
        type: str
            type of the joint(ex: rev)
        xyz: [x, y, z]
            coordinate of the joint
        axis: [x, y, z]
            coordinate of axis of the joint
        parent: str
            parent link
        child: str
            child link
        joint_xml: str
            generated xml describing about the joint
        tran_xml: str
            generated xml describing about the transmission
        """
        self.name = name
        self.type = joint_type
        self.xyz = [round_mm(x) for x in xyz]
        self.rpy = [round_rads(r) for r in rpy]
        self.parent = parent
        self.child = child
        self._joint_xml = None
        self._tran_xml = None
        self.axis = [round_mm(a) for a in axis]  # for 'revolute' and 'continuous'
        self.upper_limit = upper_limit  # for 'revolute' and 'prismatic'
        self.lower_limit = lower_limit  # for 'revolute' and 'prismatic'

    def joint_xml(self) -> Element:
        """
        Generate the joint xml
        """

        joint = Element('joint')
        joint.attrib = {'name':utils.format_name(self.name), 'type':self.type}

        origin = SubElement(joint, 'origin')
        origin.attrib = {'xyz':' '.join([str(_) for _ in self.xyz]), 'rpy':' '.join([str(_) for _ in self.rpy])}

        parent = SubElement(joint, 'parent')
        self.parent = utils.format_name(self.parent)
        parent.attrib = {'link':self.parent}

        child = SubElement(joint, 'child')
        self.child = utils.format_name(self.child)
        child.attrib = {'link':self.child}

        if self.type == 'revolute' or self.type == 'continuous' or self.type == 'prismatic':        
            axis = SubElement(joint, 'axis')
            axis.attrib = {'xyz':' '.join([str(_) for _ in self.axis])}
        if self.type == 'revolute' or self.type == 'prismatic':
            limit = SubElement(joint, 'limit')
            limit.attrib = {'upper': str(self.upper_limit), 'lower': str(self.lower_limit),
                            'effort': f'{Joint.effort_limit}', 'velocity': f'{Joint.vel_limit}'}

        return joint

    def transmission_xml(self) -> Element:
        """
        Generate the tran_xml and hold it by self.tran_xml
        
        
        Notes
        -----------
        mechanicalTransmission: 1
        type: transmission interface/SimpleTransmission
        hardwareInterface: PositionJointInterface        
        """        
        
        tran = Element('transmission')
        tran.attrib = {'name':utils.format_name(self.name) + '_tran'}
        
        joint_type = SubElement(tran, 'type')
        joint_type.text = 'transmission_interface/SimpleTransmission'
        
        joint = SubElement(tran, 'joint')
        joint.attrib = {'name':utils.format_name(self.name)}
        hardwareInterface_joint = SubElement(joint, 'hardwareInterface')
        hardwareInterface_joint.text = 'hardware_interface/EffortJointInterface'
        
        actuator = SubElement(tran, 'actuator')
        actuator.attrib = {'name':utils.format_name(self.name) + '_actr'}
        hardwareInterface_actr = SubElement(actuator, 'hardwareInterface')
        hardwareInterface_actr.text = 'hardware_interface/EffortJointInterface'
        mechanicalReduction = SubElement(actuator, 'mechanicalReduction')
        mechanicalReduction.text = '1'

        return tran

class Link:

    scale = '0.001'

    def __init__(self, name, xyz, rpy, center_of_mass, sub_folder, mass, inertia_tensor, bodies, sub_mesh, material_dict, visible):
        """
        Parameters
        ----------
        name: str
            name of the link
        xyz: [x, y, z]
            coordinate for the visual and collision
        center_of_mass: [x, y, z]
            coordinate for the center of mass
        link_xml: str
            generated xml describing about the link
        sub_folder: str
            the name of the repository to save the xml file
        mass: float
            mass of the link
        inertia_tensor: [ixx, iyy, izz, ixy, iyz, ixz]
            tensor of the inertia
        bodies = [body1, body2, body3]
            list of visible bodies
        """

        self.name = name
        # xyz for visual
        self.xyz = [round_mm(x) for x in xyz]
        self.rpy = [round_rads(r) for r in rpy]
        # xyz for center of mass
        self.center_of_mass = [round_mm(x) for x in center_of_mass]
        self._link_xml = None
        self.sub_folder = sub_folder
        self.mass = mass
        self.inertia_tensor = inertia_tensor
        self.bodies = bodies
        self.sub_mesh = sub_mesh # if we want to export each body as a separate mesh
        self.material_dict = material_dict
        self.visible = visible

        
    def link_xml(self) -> Optional[Element]:
        """
        Generate the link_xml and hold it by self.link_xml
        """
        link = Element('link')
        link.attrib = {'name':self.name}
        rpy = ' '.join([str(_) for _ in self.rpy])
        scale = ' '.join([self.scale]*3)

        #inertial
        inertial = SubElement(link, 'inertial')
        origin_i = SubElement(inertial, 'origin')
        origin_i.attrib = {'xyz':' '.join([str(_) for _ in self.center_of_mass]), 'rpy':rpy}       
        mass = SubElement(inertial, 'mass')
        mass.attrib = {'value':str(self.mass)}
        inertia = SubElement(inertial, 'inertia')
        inertia.attrib = {'ixx':str(self.inertia_tensor[0]), 'iyy':str(self.inertia_tensor[1]),
                        'izz':str(self.inertia_tensor[2]), 'ixy':str(self.inertia_tensor[3]),
                        'iyz':str(self.inertia_tensor[4]), 'ixz':str(self.inertia_tensor[5])}        
        
        if not self.visible:
            return link

        # visual
        if self.sub_mesh and len(self.bodies) > 1: # if we want to export each as a separate mesh
            for body_name in self.bodies:
                visual = SubElement(link, 'visual')
                origin_v = SubElement(visual, 'origin')
                origin_v.attrib = {'xyz':' '.join([str(_) for _ in self.xyz]), 'rpy':rpy}
                geometry_v = SubElement(visual, 'geometry')
                mesh_v = SubElement(geometry_v, 'mesh')
                mesh_v.attrib = {'filename':f'package://{self.sub_folder}{body_name}.stl','scale':scale}
                material = SubElement(visual, 'material')
                material.attrib = {'name': self.material_dict[body_name]}
        else:
            visual = SubElement(link, 'visual')
            origin_v = SubElement(visual, 'origin')
            origin_v.attrib = {'xyz':' '.join([str(_) for _ in self.xyz]), 'rpy':rpy}
            geometry_v = SubElement(visual, 'geometry')
            mesh_v = SubElement(geometry_v, 'mesh')
            mesh_v.attrib = {'filename':f'package://{self.sub_folder}{self.name}.stl','scale':scale}
            material = SubElement(visual, 'material')
            material.attrib = {'name': self.material_dict[self.name]}
    
        
        # collision
        collision = SubElement(link, 'collision')
        origin_c = SubElement(collision, 'origin')
        origin_c.attrib = {'xyz':' '.join([str(_) for _ in self.xyz]), 'rpy':rpy}
        geometry_c = SubElement(collision, 'geometry')
        mesh_c = SubElement(geometry_c, 'mesh')
        mesh_c.attrib = {'filename':f'package://{self.sub_folder}{self.name}.stl','scale':scale}

        return link
