# -*- coding: utf-8 -*-
"""
Created on Sun May 12 20:17:17 2019

@author: syuntoku

Modified by cadop Dec 19 2021
"""

from xml.etree.ElementTree import Element, SubElement
from xml.etree import ElementTree
from xml.dom import minidom

class Joint:

    # Defaults for all joints 
    effort_limit = 100
    vel_limit = 100

    def __init__(self, name, xyz, axis, parent, child, joint_type, upper_limit, lower_limit):
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
        self.xyz = xyz
        self.parent = parent
        self.child = child
        self._joint_xml = None
        self._tran_xml = None
        self.axis = axis  # for 'revolute' and 'continuous'
        self.upper_limit = upper_limit  # for 'revolute' and 'prismatic'
        self.lower_limit = lower_limit  # for 'revolute' and 'prismatic'
        
    @property
    def joint_xml(self):
        """
        Generate the joint_xml and hold it by self.joint_xml
        """

        joint = Element('joint')
        joint.attrib = {'name':self.name.replace(':','_').replace(' ',''), 'type':self.type}

        origin = SubElement(joint, 'origin')
        origin.attrib = {'xyz':' '.join([str(_) for _ in self.xyz]), 'rpy':'0 0 0'}

        parent = SubElement(joint, 'parent')
        self.parent = self.parent.replace(':','_').replace(' ','')
        parent.attrib = {'link':self.parent}

        child = SubElement(joint, 'child')
        self.child = self.child.replace(':','_').replace(' ','')
        child.attrib = {'link':self.child}

        if self.type == 'revolute' or self.type == 'continuous' or self.type == 'prismatic':        
            axis = SubElement(joint, 'axis')
            axis.attrib = {'xyz':' '.join([str(_) for _ in self.axis])}
        if self.type == 'revolute' or self.type == 'prismatic':
            limit = SubElement(joint, 'limit')
            limit.attrib = {'upper': str(self.upper_limit), 'lower': str(self.lower_limit),
                            'effort': f'{Joint.effort_limit}', 'velocity': f'{Joint.vel_limit}'}

        rough_string = ElementTree.tostring(joint, 'utf-8')
        reparsed = minidom.parseString(rough_string)
        self._joint_xml = "\n".join(reparsed.toprettyxml(indent="  ").split("\n")[1:])

        return self._joint_xml

    @property
    def transmission_xml(self):
        """
        Generate the tran_xml and hold it by self.tran_xml
        
        
        Notes
        -----------
        mechanicalTransmission: 1
        type: transmission interface/SimpleTransmission
        hardwareInterface: PositionJointInterface        
        """        
        
        tran = Element('transmission')
        tran.attrib = {'name':self.name.replace(':','_').replace(' ','') + '_tran'}
        
        joint_type = SubElement(tran, 'type')
        joint_type.text = 'transmission_interface/SimpleTransmission'
        
        joint = SubElement(tran, 'joint')
        joint.attrib = {'name':self.name.replace(':','_').replace(' ','')}
        hardwareInterface_joint = SubElement(joint, 'hardwareInterface')
        hardwareInterface_joint.text = 'hardware_interface/EffortJointInterface'
        
        actuator = SubElement(tran, 'actuator')
        actuator.attrib = {'name':self.name.replace(':','_').replace(' ','') + '_actr'}
        hardwareInterface_actr = SubElement(actuator, 'hardwareInterface')
        hardwareInterface_actr.text = 'hardware_interface/EffortJointInterface'
        mechanicalReduction = SubElement(actuator, 'mechanicalReduction')
        mechanicalReduction.text = '1'

        rough_string = ElementTree.tostring(tran, 'utf-8')
        reparsed = minidom.parseString(rough_string)
        self._tran_xml  = "\n".join(reparsed.toprettyxml(indent="  ").split("\n")[1:])

        return self._tran_xml

class Link:

    mesh_scale = '0.001'

    def __init__(self, name, xyz, center_of_mass, sub_folder, mass, inertia_tensor, body_dict, sub_mesh):
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
        body_lst = [body1, body2, body3]
            list of visible bodies
        body_dict = {body.entityToken: name of occurrence}
            dictionary of body entity tokens to the occurrence name
        """

        self.name = name
        # xyz for visual
        self.xyz = [-(_) for _ in xyz]  # reverse the sign of xyz
        # xyz for center of mass
        self.center_of_mass = [x for x in center_of_mass]
        self._link_xml = None
        self.sub_folder = sub_folder
        self.mass = mass
        self.inertia_tensor = inertia_tensor
        self.body_dict = body_dict
        self.sub_mesh = sub_mesh # if we want to export each body as a separate mesh

        
    @property
    def link_xml(self):
        """
        Generate the link_xml and hold it by self.link_xml
        """
        
        link = Element('link')
        self.name = self.name.replace(':','_').replace(' ','')
        link.attrib = {'name':self.name}
        
        #inertial
        inertial = SubElement(link, 'inertial')
        origin_i = SubElement(inertial, 'origin')
        origin_i.attrib = {'xyz':' '.join([str(_) for _ in self.center_of_mass]), 'rpy':'0 0 0'}       
        mass = SubElement(inertial, 'mass')
        mass.attrib = {'value':str(self.mass)}
        inertia = SubElement(inertial, 'inertia')
        inertia.attrib = {'ixx':str(self.inertia_tensor[0]), 'iyy':str(self.inertia_tensor[1]),
                          'izz':str(self.inertia_tensor[2]), 'ixy':str(self.inertia_tensor[3]),
                          'iyz':str(self.inertia_tensor[4]), 'ixz':str(self.inertia_tensor[5])}        
        
        # visual
        if self.sub_mesh: # if we want to export each as a separate mesh
            for body_name in self.body_dict[self.name]:
                # body_name = body_name.replace(':','_').replace(' ','')
                visual = SubElement(link, 'visual')
                origin_v = SubElement(visual, 'origin')
                origin_v.attrib = {'xyz':' '.join([str(_) for _ in self.xyz]), 'rpy':'0 0 0'}
                geometry_v = SubElement(visual, 'geometry')
                mesh_v = SubElement(geometry_v, 'mesh')
                mesh_v.attrib = {'filename':f'package://{self.sub_folder}{body_name}.stl','scale':f'{Link.mesh_scale} {Link.mesh_scale} {Link.mesh_scale}'}
                material = SubElement(visual, 'material')
                material.attrib = {'name':'silver'}
        else:
            visual = SubElement(link, 'visual')
            origin_v = SubElement(visual, 'origin')
            origin_v.attrib = {'xyz':' '.join([str(_) for _ in self.xyz]), 'rpy':'0 0 0'}
            geometry_v = SubElement(visual, 'geometry')
            mesh_v = SubElement(geometry_v, 'mesh')
            mesh_v.attrib = {'filename':f'package://{self.sub_folder}{self.name.replace('_virtual', '')}.stl','scale':f'{Link.mesh_scale} {Link.mesh_scale} {Link.mesh_scale}'}
            material = SubElement(visual, 'material')
            material.attrib = {'name':'silver'}
    
        
        # collision
        collision = SubElement(link, 'collision')
        origin_c = SubElement(collision, 'origin')
        origin_c.attrib = {'xyz':' '.join([str(_) for _ in self.xyz]), 'rpy':'0 0 0'}
        geometry_c = SubElement(collision, 'geometry')
        mesh_c = SubElement(geometry_c, 'mesh')
        mesh_c.attrib = {'filename':'package://' + self.sub_folder + self.name.replace('_virtual', '') + '.stl','scale':'0.001 0.001 0.001'}

        rough_string = ElementTree.tostring(link, 'utf-8')
        reparsed = minidom.parseString(rough_string)
        self._link_xml  = "\n".join(reparsed.toprettyxml(indent="  ").split("\n")[1:])
        return self._link_xml
