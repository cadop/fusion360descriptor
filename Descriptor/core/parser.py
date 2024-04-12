'''
module to parse fusion file 
'''

import adsk, adsk.core, adsk.fusion
from . import transforms
from . import parts
from . import utils
from collections import Counter, defaultdict

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
        self.component: adsk.fusion.Occurrence = component
        self.name: str = component.name
        self._parent = None

    def _add_child(self, c) -> None:
        self.children.append(c)
        c.parent = self 

    def get_children(self) -> list:
        return self.children        

    def get_all_children(self) -> dict[str]:
        ''' get all children and sub children of this instance '''

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

    def get_flat_body(self) -> list:
        ''' get a flat list of all components and child components '''

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
            # Pop an element form the stack (order shouldn't matter)
            tmp = parent_stack.pop()
            closed_set.add(tmp)
            # Get any bodies directly associated with this component
            if tmp.component.bRepBodies.count > 0:
                body_list.append([tmp.component.bRepBodies.item(x) for x in range(0, tmp.component.bRepBodies.count) ])

            # Check if this child has children
            if len(tmp.children)> 0:
                # add them to the parent_stack
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

    def get_all_parents(self) -> list:
        ''' get all the parents of this instance '''

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
        '''Recursively create class instances and define a parent->child structure
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

            # Break links to avoid unwanted changes
            if occ.isReferencedComponent:
                occ.breakLink()

            cur = Hierarchy(occ)

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
        self.root: adsk.fusion.Component = root
        self.occ = root.occurrences.asList
        self.inertial_dict = {}
        self.inertia_accuracy = adsk.fusion.CalculationAccuracy.LowCalculationAccuracy

        self.links_xyz_dict = {} # needed ?

        self.sub_mesh = False
        self.joints_dict = {}
        self.body_dict = {}
        self.material_dict = {}
        self.color_dict = {}
        self.links = {} # Link class
        self.joints = {} # Joint class for writing to file
        self.joint_order = ('p','c') # Order of joints defined by components
        self.scale = 100.0 # Units to convert to meters (or whatever simulator takes)
        self.inertia_scale = 10000.0 # units to convert mass
        self.base_link: adsk.fusion.Occurrence = None
        self.component_map: dict[str, Hierarchy] = dict() # Entity tokens for each component

        self.root_node = None

    def get_scene_configuration(self):
        '''Build the graph of how the scene components are related
        '''        
        
        self.root_node = Hierarchy(self.root)
        occ_list=self.root.occurrences.asList

        Hierarchy.traverse(occ_list, self.root_node)
        self.component_map = self.root_node.get_all_children()

        self.get_sub_bodies()

        return self.component_map



    def get_sub_bodies(self):
        ''' temp fix for ensuring that a top-level component is associated with bodies'''

        # write the immediate children of root node
        self.body_mapper = defaultdict(list)

        # for k,v in self.component_map.items():
        for v in self.root_node.children:
            
            children = set()
            children.update(v.children)

            top_level_body = [v.component.bRepBodies.item(x) for x in range(0, v.component.bRepBodies.count) ]
            top_level_body = [x for x in top_level_body if x.isLightBulbOn]
            
            # add to the body mapper
            if top_level_body != []:
                self.body_mapper[v.component.entityToken].extend(top_level_body)

            while children:
                cur = children.pop()
                children.update(cur.children)
                sub_level_body = [cur.component.bRepBodies.item(x) for x in range(0, cur.component.bRepBodies.count) ]
                sub_level_body = [x for x in sub_level_body if x.isLightBulbOn ]
                
                # add to this body mapper again 
                self.body_mapper[cur.component.entityToken].extend(sub_level_body)

    def get_joint_preview(self):
        ''' Get the scenes joint relationships without calculating links 
        Returns
        -------
        dict
            joint relationships
        '''

        self._joints()
        return self.joints_dict

    def parse(self):
        ''' parse the scene by building up inertia and joints'''

        self._base()
        self._inertia()
        self._joints()
        self._materials()
        self._build_links()
        self._build_joints()

    @property
    def name(self):
        ''' Name of the root component '''
        return self.root.name.split()[0]

    def _base(self):
        ''' Get the base link '''
        for oc in self._iterate_through_occurrences():
            # Get only the first grounded link
            if oc.isGrounded:
                # We must store this object because we cannot occurrences
                self.base_link = oc
                break
        if self.base_link is None:
            # TODO: Improve handling if there is no grounded occurrence
            print("ERROR: Failed to find a grounded occurrence for base_link")
            exit("Failed to find a grounded occurrence for base_link")
    
    def _get_inertia(self, oc: adsk.fusion.Occurrence):
        occs_dict = {}

        prop = oc.getPhysicalProperties(self.inertia_accuracy)
        
        if oc.entityToken == self.base_link.entityToken:
            occ_name = "base_link"
        else:
            occ_name = oc.name
        occs_dict['name'] = utils.rename_if_duplicate(occ_name, self.inertial_dict)

        mass = prop.mass  # kg

        # Iterate through bodies, only add mass of bodies that are visible (lightbulb)
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

        return occs_dict

    def _iterate_through_occurrences(self):
        for key, token in self.component_map.items():
            yield token.component

    def _inertia(self):
        '''
        Define inertia values
        
        Notes
        -----
        Original Authors: @syuntoku, @yanshil
        Modified by @cadop
        '''
        # Build a flat inertial dict
        for occ in self._iterate_through_occurrences():
            # "An occurrence will only be visible if the light bulb is switched on.
            # However, the light bulb can be on and the occurrence still invisible
            # if a higher level occurrence in the assembly context is not visible
            # because its light bulb is off."
            if occ.isLightBulbOn:
                self.inertial_dict[occ.entityToken] = self._get_inertia(occ)

    def _is_joint_valid(self, joint):
        '''_summary_
        Parameters
        ----------
        joint : _type_
            _description_
        '''

        try: 
            joint.geometryOrOriginOne.origin.asArray()
            return True
        except:
            return False


    def _joints(self):
        ''' Iterates over joints list and defines properties for each joint
        (along with its relationship)
        '''

        for joint in self.root.allJoints:
            joint_dict = {}

            # Check that both bodies are valid (e.g. there is no missing reference)
            if not self._is_joint_valid(joint):
                # TODO: Handle in a better way (like warning message)
                continue

            # Rename if the joint already exists in our dictionary
            joint.name = utils.rename_if_duplicate(joint.name, self.joints_dict)
            joint_dict['token'] = joint.entityToken

            joint_type = Configurator.joint_type_list[joint.jointMotion.jointType]
            joint_dict['type'] = joint_type

            occ_one = joint.occurrenceOne
            occ_two = joint.occurrenceTwo

            geom_one_origin = joint.geometryOrOriginOne.origin.asArray()
            
            joint_type = joint.jointMotion.objectType # string 
            
            # Only Revolute joints have rotation axis 
            if 'RevoluteJointMotion' in joint_type:
                joint_vector = joint.jointMotion.rotationAxisVector.asArray() 
                joint_limit_max = joint.jointMotion.rotationLimits.maximumValue
                joint_limit_min = joint.jointMotion.rotationLimits.minimumValue
                
                if abs(joint_limit_max - joint_limit_min) == 0:
                    joint_limit_min = -3.14159
                    joint_limit_max = 3.14159
            elif 'SliderJointMotion' in joint_type:
                joint_vector=joint.jointMotion.slideDirectionVector.asArray()
                joint_limit_max = joint.jointMotion.slideLimits.maximumValue/100.0
                joint_limit_min = joint.jointMotion.slideLimits.minimumValue/100.0
            else:
                # Keep default limits for 'RigidJointMotion' or others
                joint_vector = [0, 0, 0]
                joint_limit_max = 0.0
                joint_limit_min = 0.0

            joint_dict['axis'] = joint_vector
            joint_dict['upper_limit'] = joint_limit_max
            joint_dict['lower_limit'] = joint_limit_min

            # Reverses which is parent and child
            if occ_one.entityToken == self.base_link.entityToken:
                occ_one_name = "base_link"
                occ_two_name = occ_two.name
            elif occ_two.entityToken == self.base_link.entityToken:
                occ_one_name = occ_one.name
                occ_two_name = "base_link"
            else:
                occ_one_name = occ_one.name
                occ_two_name = occ_two.name
            if self.joint_order == ('p','c'):
                joint_dict['parent'] = occ_one_name
                joint_dict['child'] = occ_two_name
                joint_dict['parent_token'] = occ_one.entityToken
                joint_dict['child_token'] = occ_two.entityToken
            elif self.joint_order == ('c','p'):
                joint_dict['child'] = occ_one_name
                joint_dict['parent'] = occ_two_name
                joint_dict['child_token'] = occ_one.entityToken
                joint_dict['parent_token'] = occ_two.entityToken
            else:
                raise ValueError(f'Order {self.joint_order} not supported')

            joint_dict['xyz'] = [ x/self.scale for x in geom_one_origin]
            self.joints_dict[joint.name] = joint_dict

        # Add RigidGroups as fixed joints
        for group in self.root.allRigidGroups:
            original_group_name = group.name
            for i, occ in enumerate(group.occurrences):
                # Assumes that the first occurrence will be the parent
                if i == 0:
                    parent_occ = occ
                    continue
                joint_dict = {}
                rigid_group_occ_name = utils.rename_if_duplicate(original_group_name, self.joints_dict)
                joint_dict['token'] = occ.entityToken
                joint_dict['type'] = 'fixed'

                # Unneeded for fixed joints
                joint_dict['axis'] = [0, 0, 0]
                joint_dict['upper_limit'] = 0
                joint_dict['lower_limit'] = 0

                if parent_occ.entityToken == self.base_link.entityToken:
                    parent_occ_name = "base_link"
                    occ_name = occ.name
                elif occ.entityToken == self.base_link.entityToken:
                    parent_occ_name = parent_occ.name
                    occ_name = "base_link"
                else:
                    parent_occ_name = parent_occ.name
                    occ_name = occ.name
                joint_dict['parent'] = parent_occ_name
                joint_dict['child'] = occ_name
                joint_dict['parent_token'] = parent_occ.entityToken
                joint_dict['child_token'] = occ.entityToken

                joint_dict['xyz'] = [0,0,0] # Not sure if this will always work
                self.joints_dict[rigid_group_occ_name] = joint_dict

        occurrences = defaultdict(list)
        for joint_name, joint_dict in self.joints_dict.items():
            occurrences[joint_dict["parent"]].append(joint_name)
            occurrences[joint_dict["child"]].append(joint_name)
        grounded_occ = {"base_link"}
        boundary = grounded_occ
        while boundary:
            new_boundary = set()
            for occ_name in boundary:
                for joint_name in occurrences[occ_name]:
                    joint = self.joints_dict[joint_name]
                    if joint["parent"] == occ_name:
                        if joint["child"] not in grounded_occ:
                            new_boundary.add(joint["child"])
                    else:
                        assert joint["child"] == occ_name
                        if joint["parent"] not in grounded_occ:
                            # Parent is further away from base_link than the child, swap them
                            original_child_token = joint["child_token"]
                            joint["child"] = joint["parent"]
                            joint["child_token"] = joint["parent_token"]
                            joint["parent"] = occ_name
                            joint["parent_token"] = original_child_token
                            joint["xyz"] = [-x for x in joint["xyz"]]
                            new_boundary.add(joint["child"])
            grounded_occ.update(new_boundary)
            boundary = new_boundary

    def __add_recursive_links(self, inertia_occurrence, mesh_folder):
        for k, inertia in inertia_occurrence.items():
            if isinstance(inertia, dict) and len(inertia) == 1:
                result = self.__add_recursive_links(inertia)
                if isinstance(result, dict):
                    return result
            else:
                # Defaulting to just the center of mass alone
                center_of_mass = inertia['center_of_mass']
                xyz = [0,0,0]
                for jk, joint in self.joints_dict.items():
                    if joint['child_token'] == k or joint['parent_token'] == k:
                        center_of_mass = [ i-j for i, j in zip(inertia['center_of_mass'], joint['xyz'])]
                        xyz = joint['xyz']
                        break

                link = parts.Link(name = inertia['name'],
                                xyz = (xyz[0], xyz[1], xyz[2]),
                                center_of_mass = center_of_mass,
                                sub_folder = mesh_folder,
                                mass = inertia['mass'],
                                inertia_tensor = inertia['inertia'],
                                body_dict = self.body_dict_urdf,
                                sub_mesh = self.sub_mesh,
                                material_dict = self.material_dict)
                self.links_xyz_dict[k] = (link.xyz[0], link.xyz[1], link.xyz[2])
                self.links[link.name] = link

    def __get_appearance(self, occ: adsk.fusion.Occurrence):
        # Prioritize appearance properties, but it could be null
        appearance = None
        if occ.appearance:
            appearance = occ.appearance
        elif occ.bRepBodies:
            for body in occ.bRepBodies:
                if body.appearance:
                    appearance = body.appearance
                    break
        elif occ.component.material:
            appearance = occ.component.material.appearance

        # Material should always have an appearance, but just in case
        if appearance is not None:
            # Only supports one appearance per occurrence so return the first
            for prop in appearance.appearanceProperties:
                if type(prop) == adsk.core.ColorProperty:
                    return(appearance.name, prop)
        return (None, None)

    def _materials(self) -> None:
        # Adapted from SpaceMaster85/fusion2urdf
        self.color_dict['silver_default'] = "0.700 0.700 0.700 1.000"

        for occ in self._iterate_through_occurrences():
            occ_material_dict = {}
            occ_material_dict['material'] = "silver_default"
            prop_name, prop = self.__get_appearance(occ)

            if prop:
                color_name = utils.convert_german(prop_name)
                color_name = utils.format_name(color_name)
                occ_material_dict['material'] = color_name
                self.color_dict[color_name] = f"{prop.value.red/255} {prop.value.green/255} {prop.value.blue/255} {prop.value.opacity/255}"
            if occ.entityToken == self.base_link.entityToken:
                occ_name = "base_link"
            else:
                occ_name = occ.name
            self.material_dict[utils.format_name(occ_name)] = occ_material_dict


    def _build_links(self):
        ''' create links '''

        mesh_folder = f'{self.name}/meshes/'

        #creates list of bodies that are visible

        self.body_dict = defaultdict(list) # key : occurrence name -> value : list of bodies under that occurrence
        body_dict_urdf = defaultdict(list) # list to send to parts.py
        duplicate_bodies = defaultdict(int) # key : name -> value : # of instances

        oc_name = ''
        # Make sure no repeated body names
        body_count = Counter()
        
        for oc in self._iterate_through_occurrences():
            if oc.entityToken == self.base_link.entityToken:
                occ_name = "base_link"
            else:
                occ_name = oc.name
            oc_name = utils.format_name(occ_name)
            # self.body_dict[oc_name] = []
            # body_lst = self.component_map[oc.entityToken].get_flat_body() #gets list of all bodies in the occurrence

            body_lst = self.body_mapper[oc.entityToken]
            
            if len(body_lst) > 0:
                for body in body_lst:
                    # Check if this body is hidden
                    if body.isLightBulbOn:
                        if body.name in duplicate_bodies:
                            duplicate_bodies[body.name] +=1
                        self.body_dict[oc_name].append(body)

                        body_name = utils.format_name(body.name)
                        body_name_cnt = f'{body_name}_{body_count[body_name]}'
                        body_count[body_name] += 1

                        unique_bodyname = f'{oc_name}_{body_name_cnt}'
                        body_dict_urdf[oc_name].append(unique_bodyname)
                    
        # Make the actual urdf names accessible
        self.body_dict_urdf = body_dict_urdf
        
        self.__add_recursive_links(self.inertial_dict, mesh_folder)


    def _build_joints(self):
        ''' create joints by setting parent and child relationships and constructing
        the XML formats to be exported later '''

        for k, j in self.joints_dict.items():
            # Do not add the joint if one of the links does not exist
            if any(name not in self.links for name in (j['parent'], j['child'])):
                continue

            xyz = []
            for p,c in zip(self.links_xyz_dict[j['parent_token']], self.links_xyz_dict[j['child_token']]):
                xyz.append(p-c)

            joint = parts.Joint(name=k , joint_type=j['type'], 
                                xyz=xyz, axis=j['axis'], 
                                parent=j['parent'], child=j['child'], 
                                upper_limit=j['upper_limit'], lower_limit=j['lower_limit'])
            self.joints[k] = joint
