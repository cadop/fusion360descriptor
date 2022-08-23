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
        ''' Initialize Hierarchy class to parse document and define component relationships.
        Uses a recursive traversal (based off of fusion example) and provides helper functions
        to get specific children and parents for nodes. 
        
        Parameters
        ----------
        component : [type]
            fusions root component to use for traversal
        '''        

        self.children = []
        self.component = component
        self.name = component.name
        self._parent = None

    def _add_child(self, c):
        self.children.append(c)
        c.parent = self 

    def get_children(self):
        return self.children        

    def get_all_children(self):
        ''' Get all children and sub children of this instance '''

        child_map = {}
        parent_stack = set()
        parent_stack.update(self.get_children())
        while len(parent_stack) != 0:
            # Pop an element from the stack (order shouldn't matter)
            tmp = parent_stack.pop()
            # Add this child to the map
            # Use the entity token, more accurate than the name of the component (since there are multiple)
            child_map[tmp.component.entityToken] = tmp 
            # Check if this child has children
            if len(tmp.get_children())> 0:
                # Add them to the parent_stack
                parent_stack.update(tmp.get_children())

        return child_map

    def get_flat_body(self):
        ''' Get a flat list of all components and child components '''

        child_list = []
        body_list = []
        parent_stack = set()

        child_set = list(self.get_all_children().values())

        if len(child_set) == 0:
            body_list.append([self.component.bRepBodies.item(x) for x in range(0, self.component.bRepBodies.count) ])

        child_list = [x.children for x in child_set if len(x.children)>0]
        childs = []
        for c in child_list:
            for _c in c:
                childs.append(_c)

        parent_stack.update(childs)
        closed_set = set()

        while len(parent_stack) != 0:
            # Pop an element from the stack (order shouldn't matter)
            tmp = parent_stack.pop()
            closed_set.add(tmp)
            # Get any bodies directly associated with this component
            if tmp.component.bRepBodies.count > 0:
                body_list.append([tmp.component.bRepBodies.item(x) for x in range(0, tmp.component.bRepBodies.count) ])

            # Check if this child has children
            if len(tmp.children)> 0:
                # Add them to the parent_stack
                child_set = list(self.get_all_children().values())

                child_list = [x.children for x in child_set if len(x.children)>0]
                childs = []
                for c in child_list:
                    for _c in c:
                        if _c not in closed_set:
                            childs.append(_c)

                parent_stack.update(childs)

        flat_bodies = []
        for body in body_list:
            flat_bodies.extend(body)

        return flat_bodies

    def get_all_parents(self):
        ''' Get all the parents of this instance '''

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
        ''' Recursively create class instances and define a parent->child structure
        Based on the fusion 360 API docs
        
        Parameters
        ----------
        occurrences : [type]
            [description]
        parent : [type], optional
            [description], by default None
        Returns
        -------
        Hierarchy
            Instance of the class
        '''        
        
        for i in range(0, occurrences.count):
            occ = occurrences.item(i)
            cur = Hierarchy(occ)
            # print(cur.name)
            if parent is None: 
                pass
            else: 
                parent._add_child(cur)
            if occ.childOccurrences:
                Hierarchy.traverse(occ.childOccurrences, parent=cur)
        return cur

class Configurator:

    joint_type_list = [ 'fixed', 'revolute', 'prismatic', 'Cylinderical',
                        'PinSlot', 'Planner', 'Ball']  # these are the names in urdf

    def __init__(self, root) -> None:
        ''' Initializes Configurator class to handle building hierarchy and parsing

        Parameters
        ----------
        root : [type]
            root component of design document
        '''        
        # Export top-level occurrences
        self.root = root
        self.occ = root.occurrences.asList
        self.inertial_dict = {}
        self.inertia_accuracy = adsk.fusion.CalculationAccuracy.LowCalculationAccuracy

        self.links_xyz_dict = {} # needed ?
        self.automatic_joints = False # if joint relationships need to be automatically calculated
        self.joints_dict = {}
        self.links = {} # Link class
        self.joints = {} # Joint class for writing to file
        self.joint_order = ('p','c') # Order of joints defined by components
        self.scale = 100.0 # Units to convert to meters (or whatever simulator takes)
        self.inertia_scale = 10000.0 # units to convert mass
        self.base_links= set()

    def get_scene_configuration(self):
        ''' Build the graph of how the scene components are related
        '''        
        
        root_node = Hierarchy(self.root)
        occ_list=self.root.occurrences.asList
        Hierarchy.traverse(occ_list, root_node)
        self.component_map = root_node.get_all_children()
        return self.component_map

    def get_joint_preview(self):
        ''' Get the scenes joint relationships without calculating links 

        Returns
        -------
        dict
            joint relationships
        '''

        self._joints()
        if self.automatic_joints:
            self._create_structure()
        return self.joints_dict

    def parse(self):
        ''' Parse the scene by building up inertia and joints'''

        self._inertia()
        self._joints()
        if self.automatic_joints:
            self._create_structure()
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
                name = oc.name
                self.base_links.add(name)

    def _inertia(self):
        '''
        Define inertia values
        
        Notes
        -----
        Original Authors: @syuntoku, @yanshil
        Modified by @cadop
        '''
        
        for oc in self.occ:       
            occs_dict = {}
            prop = oc.getPhysicalProperties(self.inertia_accuracy)
            
            occs_dict['name'] = oc.name

            mass = prop.mass  # kg

            # Iterate through bodies, only add mass of bodies that are visible (lightbulb)
            # body_cnt = oc.bRepBodies.count
            # mapped_comp =self.component_map[oc.entityToken]
            body_lst = self.component_map[oc.entityToken].get_flat_body()
            if len(body_lst) > 0:
                for body in body_lst:
                    # Check if this body is hidden
                    #  
                    # body = oc.bRepBodies.item(i)
                    if not body.isLightBulbOn:
                        mass -= body.physicalProperties.mass


            occs_dict['mass'] = mass
            center_of_mass = [_/self.scale for _ in prop.centerOfMass.asArray()] ## cm to m
            occs_dict['center_of_mass'] = center_of_mass


            # https://help.autodesk.com/view/fusion360/ENU/?guid=GUID-ce341ee6-4490-11e5-b25b-f8b156d7cd97
            (_, xx, yy, zz, xy, yz, xz) = prop.getXYZMomentsOfInertia()
            moment_inertia_world = [_ / self.inertia_scale for _ in [xx, yy, zz, xy, yz, xz] ] ## kg / cm^2 -> kg/m^2

            occs_dict['inertia'] = transforms.origin2center_of_mass(moment_inertia_world, center_of_mass, mass)

            self.inertial_dict[oc.name] = occs_dict

    def _is_joint_valid(self, joint):
        '''_summary_

        Parameters
        ----------
        joint : _type_
            _description_
        '''

        try: 
            joint.geometryOrOriginOne.origin.asArray()
            joint.geometryOrOriginTwo.origin.asArray()
            return True 
        
        except:
            return False


    def _joints(self):
        ''' Iterates over joints list and defines properties for each joint
        (along with its relationship)
        '''        

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
            
            # Check that both bodies are valid (e.g. there is no missing reference)
            if not self._is_joint_valid(joint):
                # TODO: Handle in a better way (like warning message)
                continue

            geom_one_origin = joint.geometryOrOriginOne.origin.asArray()
            geom_one_primary = joint.geometryOrOriginOne.primaryAxisVector.asArray()
            geom_one_secondary = joint.geometryOrOriginOne.secondaryAxisVector.asArray()
            geom_one_third = joint.geometryOrOriginOne.thirdAxisVector.asArray()

            # Check if this is already top level
            # Check if the parent_list only contains one entity
            parent_list = self.component_map[occ_one.entityToken].get_all_parents()
            if len(parent_list) == 1:
                pass
            # If it is not, get the mapping and trace it back up
            else: 
                # The expectation is there is at least two items, the last is the full body
                # The second to last should be the next top-most component
                # Reset occurrence one
                occ_one = self.component_map[parent_list[-2]].component

            parent_list = self.component_map[occ_two.entityToken].get_all_parents()
            if len(parent_list) == 1:
                pass
            else:
                # The expectation is there is at least two items, the last is the full body
                # The second to last should be the next top-most component

                # Reset occurrence two
                occ_two = self.component_map[parent_list[-2]].component

            geom_two_origin = joint.geometryOrOriginTwo.origin.asArray()
            geom_two_primary = joint.geometryOrOriginTwo.primaryAxisVector.asArray()
            geom_two_secondary = joint.geometryOrOriginTwo.secondaryAxisVector.asArray()
            geom_two_third = joint.geometryOrOriginTwo.thirdAxisVector.asArray()
            
            joint_type = joint.jointMotion.objectType # string 
            
            # Only Revolute joints have rotation axis 
            if 'RigidJointMotion' in joint_type:
                pass
            else:
                
                joint_vector = joint.jointMotion.rotationAxisVector.asArray() 

                joint_rot_val = joint.jointMotion.rotationValue
                joint_limit_max = joint.jointMotion.rotationLimits.maximumValue
                joint_limit_min = joint.jointMotion.rotationLimits.minimumValue
                
                if abs(joint_limit_max - joint_limit_min) == 0:
                    joint_limit_min = -3.14159
                    joint_limit_max = 3.14159

                joint_angle = joint.angle.value 

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
    

    def _create_structure(self):
        ''' Define parent -> child structure by finding which components are
        connected to each other, building off of root component

        Returns
        -------
        Hierarchy
            List of [joint, parent, child] relationships
        '''  
        
        open_set = set() # Set to test (roots) 
        closed_set = set() # Set to hold roots that have been accounted for
        closed_joint_set = set() # Joints that have been accounted for

        # Finds grounded root
        map = {}
        for oc in self.occ:
            if oc.isGrounded:
                map[oc.name] = oc
                open_set.add(oc.name)
        
        while len(open_set)!=0:
            # Pop an element from the stack (order shouldn't matter)
            root = open_set.pop()
            # Add this root to the map
            root = map[root]
            # Add to the closed set
            closed_set.add(root.name)

            # Creates set of joints that are connected to root
            open_joint_set = set()
            for joint in root.joints: 
                map[joint.name] = joint
                open_joint_set.add(joint.name)

            # If the root has child occurrences, add the joints that are connected to root component 
            if root.childOccurrences: 
                for child in root.childOccurrences:
                    for joint in child.joints:
                        map[joint.name] = joint
                        open_joint_set.add(joint.name)

                        key = joint.name
                        if key not in self.joints_dict.keys(): # Make sure that this joint exists in self.joints_dict
                            self.joints_dict[joint.name] = {} 
                    

            for joint in open_joint_set:
                # If joint is not accounted for
                if joint not in closed_joint_set:
                    closed_joint_set.add(joint)
                    parent = root # Make the root the parent
                    
                    # Make other occurrence the joint is connected the child
                    if map[joint].occurrenceOne == root:
                        child = map[joint].occurrenceTwo
                    elif map[joint].occurrenceTwo == root:
                        child = map[joint].occurrenceOne
                    
                    # Add parent child to self.joints_dict 
                    self.joints_dict[joint]["parent"] = parent.name
                    self.joints_dict[joint]["child"] = child.name
                    
                    open_set.add(child.name) # Add child to open set / map
                    map[child.name] = child

                    if parent.childOccurrences: # Adds child occurrences of parent to open set / map
                        for c in parent.childOccurrences: 
                            open_set.add(c.name)
                            map[c.name] = c
                    
                    if child.childOccurrences:
                        for c in child.childOccurrences: # Adds child occurrences of child to open set / map
                            open_set.add(c.name)
                            map[c.name] = c


    def _build_links(self):
        ''' Create links '''

        mesh_folder = 'meshes/'    

        # Creates list of bodies that are visible

        visible_bodies = [] # List of bodies that are visible
        body_dict = {}
        oc_name = ''
        for oc in self.occ:
            oc_name = oc.name.replace(':','_').replace(' ','')
            body_lst = self.component_map[oc.entityToken].get_flat_body() # Gets list of all bodies in the occurrence
            # Checks for child component within "oc" component
            if oc.childOccurrences:
                body_lst.extend([oc.bRepBodies.item(x) for x in range(0, oc.bRepBodies.count) ])
                oc_list = oc.childOccurrences
                for o in oc_list:
                    body_lst_ext = self.component_map[o.entityToken].get_flat_body()
                    body_lst.extend(body_lst_ext)

            if len(body_lst) > 0:
                for body in body_lst:
                    # Check if this body is hidden
                    if body.isLightBulbOn:
                        visible_bodies.append(body)
                        body_dict[body.entityToken] = oc_name

        base_link = self.base_links.pop()
        center_of_mass = self.inertial_dict[base_link]['center_of_mass']
        link = parts.Link(name=base_link, 
                        xyz=[0,0,0], 
                        center_of_mass=center_of_mass, 
                        sub_folder=mesh_folder,
                        mass=self.inertial_dict[base_link]['mass'],
                        inertia_tensor=self.inertial_dict[base_link]['inertia'],
                        body_lst = visible_bodies,
                        body_dict = body_dict)

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
                            inertia_tensor=self.inertial_dict[name]['inertia'],
                            body_lst=visible_bodies,
                            body_dict = body_dict)

            self.links_xyz_dict[link.name] = (link.xyz[0], link.xyz[1], link.xyz[2])   

            self.links[link.name] = link

    def _build_joints(self):
        ''' Create joints by setting parent and child relationships and constructing
        the XML formats to be exported later '''

        for k, j in self.joints_dict.items():

            xyz = []
            for p,c in zip(self.links_xyz_dict[j['parent']], self.links_xyz_dict[j['child']]):
                xyz.append(p-c)
            
            joint = parts.Joint(name=k , joint_type=j['type'], 
                                xyz=xyz, axis=j['axis'], 
                                parent=j['parent'], child=j['child'], 
                                upper_limit=j['upper_limit'], lower_limit=j['lower_limit'])

            self.joints[k] = joint
