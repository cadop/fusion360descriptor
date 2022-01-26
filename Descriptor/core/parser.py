'''
module to parse fusion file 
'''

import copy
import adsk, adsk.core, adsk.fusion
from . import transforms
from . import parts

class Hierarchy:
    ''' hierarchy of the design space '''

    def __init__(self, component) -> None:
        self.children = []
        self.component = component
        self.name = component.name
        self._parent = None

    def add_child(self, c):
        self.children.append(c)
        c.parent = self 

    def get_children(self):
        return self.children        

    def get_all_children(self):
        ''' get all children and sub children '''
        child_map = {}
        parent_stack = set()
        parent_stack.update(self.get_children())
        while len(parent_stack) != 0:
            # Pop an element form the stack (order shouldn't matter)
            tmp = parent_stack.pop()
            # Add this child to the map
            # use the entity token, more accurate than the name of the component (since there are multiple)
            child_map[tmp.component.entityToken] = tmp 
            # Check if this child has children
            if len(tmp.get_children())> 0:
                # add them to the parent_stack
                parent_stack.update(tmp.get_children())

        return child_map

    def get_all_parents(self):
        ''' get all the parents of this node '''
        child_stack = set()
        child_stack.add(self)
        parent_map = []
        while len(child_stack) != 0:
            tmp = child_stack.pop()
            if tmp.parent is None:
                return parent_map
            parent_map.append(tmp.parent.component.entityToken)    
            child_stack.add(tmp.parent)

        return parent_map
            
    @property
    def parent(self):
        if self._parent is None:
            return None
        return self._parent

    @parent.setter
    def parent(self,v):
        self._parent = v

    @staticmethod
    def traverse(occurrences, parent=None):
        '''
        Based on the fusion 360 API docs
        '''
        
        for i in range(0, occurrences.count):
            occ = occurrences.item(i)
            cur = Hierarchy(occ)
            if parent is None: 
                pass
            else: 
                parent.add_child(cur)
            if occ.childOccurrences:
                Hierarchy.traverse(occ.childOccurrences, parent=cur)
        return cur

class Configurator:

    joint_type_list = [ 'fixed', 'revolute', 'prismatic', 'Cylinderical',
                        'PinSlot', 'Planner', 'Ball']  # these are the names in urdf

    def __init__(self, root) -> None:
        # Export top-level occurrences
        self.root = root
        self.occ = root.occurrences.asList
        self.inertial_dict = {}
        self.inertia_accuracy = adsk.fusion.CalculationAccuracy.LowCalculationAccuracy

        self.links_xyz_dict = {} # needed ?

        self.joints_dict = {}
        self.links = {} # Link class
        self.joints = {} # Joint class for writing to file
        self.joint_order = ('p','c') # Order of joints defined by components
        self.scale = 100.0 # Units to convert to meters (or whatever simulator takes)
        self.inertia_scale = 10000.0 # units to convert mass
        self.base_links= set()

    def get_scene_configuration(self):
        '''
        Build the graph of how the scene components are related
        '''
        print(f'{self.root}')
        root_node = Hierarchy(self.root)
        print(f'{self.root}')

        occ_list=self.root.occurrences.asList
        print(f'{self.root}')
        Hierarchy.traverse(occ_list, root_node)
        print(f'{self.root}')
        self.component_map = root_node.get_all_children()
        return

    def get_joint_preview(self):
        ''' Get the scenes joint relationships without calculating links '''

        self._joints()
        return self.joints_dict

    def parse(self):
        ''' parse the scene '''
        self._inertia()
        self._joints()
        self._base()
        self._build_links()
        self._build_joints()

    @property
    def name(self):
        ''' Name of the root component '''
        return self.root.name.split()[0]

    def _base(self):
        ''' Get the base link(s) '''
        
        for oc in self.occ:
            if oc.isGrounded:
                self.base_links.add(oc.name)

    def _inertia(self):
        '''
        Define inertia values
        '''
        
        for oc in self.occ:       
            occs_dict = {}
            prop = oc.getPhysicalProperties(self.inertia_accuracy)
            
            occs_dict['name'] = oc.name

            mass = prop.mass  # kg
            occs_dict['mass'] = mass
            center_of_mass = [_/self.scale for _ in prop.centerOfMass.asArray()] ## cm to m
            occs_dict['center_of_mass'] = center_of_mass

            # https://help.autodesk.com/view/fusion360/ENU/?guid=GUID-ce341ee6-4490-11e5-b25b-f8b156d7cd97
            (_, xx, yy, zz, xy, yz, xz) = prop.getXYZMomentsOfInertia()
            moment_inertia_world = [_ / self.inertia_scale for _ in [xx, yy, zz, xy, yz, xz] ] ## kg / cm^2 -> kg/m^2

            occs_dict['inertia'] = transforms.origin2center_of_mass(moment_inertia_world, center_of_mass, mass)

            self.inertial_dict[oc.name] = occs_dict

    def _joints(self):

        for joint in self.root.joints:
            
            joint_dict = {}
            joint_type = Configurator.joint_type_list[joint.jointMotion.jointType]
            joint_dict['type'] = joint_type

            # switch by the type of the joint
            joint_dict['axis'] = [0, 0, 0]
            joint_dict['upper_limit'] = 0.0
            joint_dict['lower_limit'] = 0.0

            occ_one = joint.occurrenceOne
            occ_two = joint.occurrenceTwo
            
            geom_one_origin = joint.geometryOrOriginOne.origin.asArray()
            geom_one_primary = joint.geometryOrOriginOne.primaryAxisVector.asArray()
            geom_one_secondary = joint.geometryOrOriginOne.secondaryAxisVector.asArray()
            geom_one_third = joint.geometryOrOriginOne.thirdAxisVector.asArray()

            print('')
            print(joint.name)
            print(f'Occurence One: {occ_one.name}')
            print(occ_one.entityToken)
            print(occ_one.fullPathName)

            print(f'Origin one {geom_one_origin}')
            print(f'geom_one_primary {geom_one_primary}')
            print(f'geom_one_secondary {geom_one_secondary}')
            print(f'geom_one_third {geom_one_third}')

            # Check if this is already top level
            # Check if the parent_list only contains one entity
            parent_list = self.component_map[occ_one.entityToken].get_all_parents()
            if len(parent_list) == 1:
                print(f'   Exists Occurence ONE: {occ_one.entityToken}')
            # If it is not, get the mapping and trace it back up
            else: 
                # the expectation is there is at least two items, the last is the full body
                # the second to last should be the next top-most component
                print(f' PARENT: {parent_list}')
                # reset occurrence one
                occ_one = self.component_map[parent_list[-2]].component
                print(f'Occurence One New: {occ_one.name}')

            print(f'Occurence Two: {occ_two.name}')
            print(f'Occurence Two entity: {occ_two.entityToken}')
            print(f'Occurence Two full path: {occ_two.fullPathName}')

            parent_list = self.component_map[occ_two.entityToken].get_all_parents()
            if len(parent_list) == 1:
                print(f'   Exists Occurence TWO: {occ_two.entityToken}')
            else:
                # the expectation is there is at least two items, the last is the full body
                # the second to last should be the next top-most component
                print(f' PARENT: {parent_list}')
                # reset occurrence two
                occ_two = self.component_map[parent_list[-2]].component
                print(f'Occurence Two New: {occ_two.name}')

            if occ_one.isGrounded:
                print('--------OCCURRENCE ONE IS GROUNDED -----')
            if occ_two.isGrounded:
                print('--------OCCURRENCE TWO IS GROUNDED -----')

            ############ JOINT #####################

            geom_two_origin = joint.geometryOrOriginTwo.origin.asArray()
            geom_two_primary = joint.geometryOrOriginTwo.primaryAxisVector.asArray()
            geom_two_secondary = joint.geometryOrOriginTwo.secondaryAxisVector.asArray()
            geom_two_third = joint.geometryOrOriginTwo.thirdAxisVector.asArray()
            
            print(f'Origin two {geom_two_origin}')
            print(f'geom_two_primary {geom_two_primary}')
            print(f'geom_two_secondary {geom_two_secondary}')
            print(f'geom_two_third {geom_two_third}')

            joint_type = joint.jointMotion.objectType # string 
            print(f'joint_type {joint_type}')
            
            # Only Revolute joints have rotation axis 
            if 'RigidJointMotion' in joint_type:
                pass
            else:
                
                joint_vector = joint.jointMotion.rotationAxisVector.asArray() 

                joint_rot_val = joint.jointMotion.rotationValue
                joint_limit_max = joint.jointMotion.rotationLimits.maximumValue
                joint_limit_min = joint.jointMotion.rotationLimits.minimumValue
                
                if abs(joint_limit_max - joint_limit_min) == 0:
                    joint_limit_min = -180.0
                    joint_limit_max = 180.0

                joint_angle = joint.angle.value 

                print(f'joint_vector {joint_vector}')
                print(f'joint_rot_val {joint_rot_val}')
                print(f'joint_limit_max {joint_limit_max}')
                print(f'joint_limit_min {joint_limit_min}')
                print(f'joint_angle {joint_angle}')

                joint_dict['axis'] = joint_vector
                joint_dict['upper_limit'] = joint_limit_max
                joint_dict['lower_limit'] = joint_limit_min

            # Reverses which is parent and child
            if self.joint_order == ('p','c'):
                joint_dict['parent'] = occ_one.name
                joint_dict['child'] = occ_two.name
            elif self.joint_order == ('c','p'):
                joint_dict['child'] = occ_one.name
                joint_dict['parent'] = occ_two.name
            else:
                raise ValueError(f'Order {self.joint_order} not supported')

            joint_dict['xyz'] = [ x/self.scale for x in geom_one_origin]

            self.joints_dict[joint.name] = joint_dict

    def _build_links(self):
        ''' create links '''

        mesh_folder = 'meshes/'

        base_link = self.base_links.pop()
        center_of_mass = self.inertial_dict[base_link]['center_of_mass']
        link = parts.Link(name=base_link, 
                        xyz=[0,0,0], 
                        center_of_mass=center_of_mass, 
                        sub_folder=mesh_folder,
                        mass=self.inertial_dict[base_link]['mass'],
                        inertia_tensor=self.inertial_dict[base_link]['inertia'])

        self.links_xyz_dict[link.name] = link.xyz
        self.links[link.name] = link

        for k, joint in self.joints_dict.items():
            name = joint['child']

            center_of_mass = [ i-j for i, j in zip(self.inertial_dict[name]['center_of_mass'], joint['xyz'])]
            link = parts.Link(name=name, 
                            xyz=(joint['xyz'][0], joint['xyz'][1], joint['xyz'][2]),
                            center_of_mass=center_of_mass,
                            sub_folder=mesh_folder, 
                            mass=self.inertial_dict[name]['mass'],
                            inertia_tensor=self.inertial_dict[name]['inertia'])

            self.links_xyz_dict[link.name] = (link.xyz[0], link.xyz[1], link.xyz[2])   

            self.links[link.name] = link

    def _build_joints(self):
        ''' create joints '''

        for k, j in self.joints_dict.items():

            xyz = []
            for p,c in zip(self.links_xyz_dict[j['parent']], self.links_xyz_dict[j['child']]):
                xyz.append(p-c)
                
            joint = parts.Joint(name=k , joint_type=j['type'], 
                                xyz=xyz, axis=j['axis'], 
                                parent=j['parent'], child=j['child'], 
                                upper_limit=j['upper_limit'], lower_limit=j['lower_limit'])

            self.joints[k] = joint

