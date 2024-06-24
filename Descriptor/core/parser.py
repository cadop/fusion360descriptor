'''
module to parse fusion file 
'''

from typing import Dict, List, Optional, Tuple, Union
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

def get_origin(o) -> Union[adsk.core.Vector3D, adsk.core.Point3D, None]:
    if isinstance(o, adsk.fusion.JointGeometry):
        return get_origin(o.origin)
    elif o is None:
        return None
    elif isinstance(o, adsk.fusion.JointOrigin):
        return get_origin(o.geometry)
    elif isinstance(o, (adsk.core.Vector3D, adsk.core.Point3D)):
        return o
    else:
        raise ValueError(f"get_origin: unexpected {o} of type {type(o)}")

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

        self.sub_mesh = False
        self.links_by_token = {}
        self.links_by_name = {}
        self.joints_dict = {}
        self.body_dict = {}
        self.material_dict = {}
        self.color_dict = {}
        self.links = {} # Link class
        self.joints = {} # Joint class for writing to file
        self.joint_order = ('p','c') # Order of joints defined by components
        self.scale = 100.0 # Units to convert to meters (or whatever simulator takes)
        self.eps = 1e-7 * self.scale
        self.inertia_scale = 10000.0 # units to convert mass
        self.base_link: adsk.fusion.Occurrence = None
        self.component_map: dict[str, Hierarchy] = dict() # Entity tokens for each component

        self.root_node = None

    def close_enough(self, a, b) -> bool:
        if isinstance(a, float) and isinstance(b, float):
            return abs(a-b) < self.eps
        elif isinstance(a, list) and isinstance(b, list):
            assert len(a) == len(b)
            return all((self.close_enough(aa,bb) for aa,bb in zip(a,b)))
        elif isinstance(a, tuple) and isinstance(b, tuple):
            assert len(a) == len(b)
            return all((self.close_enough(aa,bb) for aa,bb in zip(a,b)))
        elif isinstance(a, adsk.core.Vector3D) and isinstance(b, adsk.core.Vector3D):
            return self.close_enough(a.asArray(), b.asArray())
        elif isinstance(a, adsk.core.Point3D) and isinstance(b, adsk.core.Point3D):
            return self.close_enough(a.asArray(), b.asArray())
        else:
            raise ValueError(f"close_enough: {type(a)} and {type(b)}: not supported")

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
        self._joints()
        self._materials()
        self._build()

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
        
        self.links_by_token[self.base_link.entityToken] = "base_link"
        self.links_by_name["base_link"] = self.base_link

    def get_name(self, oc: adsk.fusion.Occurrence) -> str:
        if oc.entityToken in self.links_by_token:
            return self.links_by_token[oc.entityToken]
        name = utils.rename_if_duplicate(oc.name, self.links_by_name)
        self.links_by_name[name] = oc
        self.links_by_token[oc.entityToken] = name
        return name   
    
    def _get_inertia(self, oc: adsk.fusion.Occurrence):
        occs_dict = {}

        prop = oc.getPhysicalProperties(self.inertia_accuracy)
        
        occ_name = self.get_name(oc)
        occs_dict['name'] = occ_name

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

        center_of_mass = prop.centerOfMass.copy()
        transform = oc.transform2.copy()
        if not transform.invert():
            raise RuntimeError("Inverse transform failed")
        if not center_of_mass.transformBy(transform):
            raise RuntimeError("Center of mass transform failed")

        print(f"{oc.name}: origin={oc.transform2.getAsCoordinateSystem()[0].asArray()}, translation={oc.transform2.translation.asArray()}, center_mass(global)={prop.centerOfMass.asArray()}, center_mass(transformed)={center_of_mass.asArray()}")

        # cm -> m
        origin = self.link_origins[self.links_by_token[oc.entityToken]]
        occs_dict['center_of_mass'] = [(c-o)/self.scale for c, o in zip(center_of_mass.asArray(), origin)]

        moments = prop.getXYZMomentsOfInertia()
        if not moments[0]:
            raise RuntimeError("Retrieving moments of inertia failed")

        # https://help.autodesk.com/view/fusion360/ENU/?guid=GUID-ce341ee6-4490-11e5-b25b-f8b156d7cd97
        occs_dict['inertia'] = [_ / self.inertia_scale for _ in transforms.origin2center_of_mass(moments[1:], prop.centerOfMass.asArray(), mass) ] ## kg / cm^2 -> kg/m^2

        return occs_dict

    def _iterate_through_occurrences(self):
        for key, token in self.component_map.items():
            yield token.component


    def _joints(self):
        ''' Iterates over joints list and defines properties for each joint
        (along with its relationship)
        '''

        for joint in self.root.allJoints:
            joint_dict = {}

            orig_name = joint.name
            # Rename if the joint already exists in our dictionary
            name = utils.rename_if_duplicate(joint.name, self.joints_dict)
            joint_dict['token'] = joint.entityToken
            joint_dict['name'] = name

            joint_type = Configurator.joint_type_list[joint.jointMotion.jointType]
            joint_dict['type'] = joint_type

            occ_one = joint.occurrenceOne
            occ_two = joint.occurrenceTwo

            # XXX: check isLightBulbOn???

            print(f"Processing joint {orig_name} of type {joint_type}, between {occ_one.name} and {occ_two.name}")

            try:
                geom_one_origin = get_origin(joint.geometryOrOriginOne)
            except RuntimeError:
                geom_one_origin = None
            try:
                geom_two_origin = get_origin(joint.geometryOrOriginTwo)
            except RuntimeError:
                geom_two_origin = None

            if joint_type != "fixed":
                if geom_one_origin is None:
                    raise RuntimeError(f'Non-fixed joint {orig_name} does not have an origin, aborting')
                elif geom_two_origin is not None and not self.close_enough(geom_two_origin, geom_one_origin):
                    raise RuntimeError(f'Occurrences {occ_one.name} and {occ_two.name} of non-fixed {orig_name}' +
                                       f' have origins {geom_one_origin.asArray()} and {geom_two_origin.asArray()}'
                                       f' that do not coincide. Make sure the joint is "at 0 / at home" before exporting')
            
                joint_dict['origin'] = geom_one_origin.asArray()
            
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

            occ_one_name = self.get_name(occ_one)
            occ_two_name = self.get_name(occ_two)
            
            # Reverses which is parent and child
            if self.joint_order == ('p','c'):
                joint_dict['parent'] = occ_one_name
                joint_dict['child'] = occ_two_name
            elif self.joint_order == ('c','p'):
                joint_dict['child'] = occ_one_name
                joint_dict['parent'] = occ_two_name
            else:
                raise ValueError(f'Order {self.joint_order} not supported')

            self.joints_dict[name] = joint_dict
            print(f"Got from Fusion: {joint_dict['type']} {name} connecting",
                  f"{occ_one_name} @ {occ_one.transform2.getAsCoordinateSystem()[0].asArray()} and",
                  f"{occ_two_name} @ {occ_two.transform2.getAsCoordinateSystem()[0].asArray()}", sep="\n\t")
            print("\tOrigin 1:", geom_one_origin.asArray() if geom_one_origin is not None else None)
            print("\tOrigin 2:", geom_two_origin.asArray() if geom_two_origin is not None else None)

        # Add RigidGroups as fixed joints
        for group in self.root.allRigidGroups:
            original_group_name = group.name
            print(f"Processing Rigid Group {original_group_name}")
            for i, occ in enumerate(group.occurrences):
                # Assumes that the first occurrence will be the parent
                if i == 0:
                    parent_occ = occ
                    continue
                joint_dict = {}
                rigid_group_occ_name = utils.rename_if_duplicate(original_group_name, self.joints_dict)
                joint_dict['name'] = rigid_group_occ_name
                joint_dict['type'] = 'fixed'

                # Unneeded for fixed joints
                joint_dict['axis'] = [0, 0, 0]
                joint_dict['upper_limit'] = 0
                joint_dict['lower_limit'] = 0

                parent_occ_name = self.get_name(parent_occ)
                occ_name = self.get_name(occ)
                joint_dict['parent'] = parent_occ_name
                joint_dict['child'] = occ_name
                print(f"Got from Fusion: {rigid_group_occ_name}, connecting",
                      f"parent {parent_occ_name} @ {parent_occ.transform2.getAsCoordinateSystem()[0].asArray()} and"
                      f"child {occ_name} {occ.transform2.getAsCoordinateSystem()[0].asArray()}")
                self.joints_dict[rigid_group_occ_name] = joint_dict

    def __add_link(self, occ: adsk.fusion.Occurrence):
        inertia = self._get_inertia(occ)
        urdf_origin = self.link_origins[inertia['name']]
        #fusion_origin = occ.transform2.getAsCoordinateSystem()[0].asArray()

        link = parts.Link(name = inertia['name'],
                        xyz = (-u/self.scale  for u in urdf_origin),
                        center_of_mass = inertia['center_of_mass'],
                        sub_folder = self.mesh_folder,
                        mass = inertia['mass'],
                        inertia_tensor = inertia['inertia'],
                        body_dict = self.body_dict_urdf,
                        sub_mesh = self.sub_mesh,
                        material_dict = self.material_dict)
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
            occ_name = self.get_name(occ)
            self.material_dict[utils.format_name(occ_name)] = occ_material_dict


    def _build(self):
        ''' create links and joints by setting parent and child relationships and constructing
        the XML formats to be exported later'''

        self.mesh_folder = f'{self.name}/meshes/'

        #creates list of bodies that are visible

        self.body_dict = defaultdict(list) # key : occurrence name -> value : list of bodies under that occurrence
        self.body_dict_urdf = defaultdict(list) # list to send to parts.py
        duplicate_bodies = defaultdict(int) # key : name -> value : # of instances
        self.link_origins: Dict[str, Tuple[float,float,float]] = {} # Location of the URDF link origin w.r.t Fusion global frame in Fusion units

        oc_name = ''
        # Make sure no repeated body names
        body_count = Counter()
        
        for oc in self._iterate_through_occurrences():
            occ_name = self.get_name(oc)
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
                        self.body_dict_urdf[oc_name].append(unique_bodyname)
        
        occurrences = defaultdict(list)
        for joint_name, joint_dict in self.joints_dict.items():
            occurrences[joint_dict["parent"]].append(joint_name)
            occurrences[joint_dict["child"]].append(joint_name)
        grounded_occ = {"base_link"}
        # URDF origin at base link origin "by definition"
        self.link_origins["base_link"] = self.base_link.transform2.getAsCoordinateSystem()[0].asArray()
        self.__add_link(self.base_link)
        boundary = grounded_occ
        while boundary:
            new_boundary = set()
            for occ_name in boundary:
                for joint_name in occurrences[occ_name]:
                    joint = self.joints_dict[joint_name]
                    if joint["parent"] == occ_name:
                        child_name = joint["child"]
                        if child_name in grounded_occ:
                            continue
                    else:
                        assert joint["child"] == occ_name
                        if joint["parent"] not in grounded_occ:
                            # Parent is further away from base_link than the child, swap them
                            child_name = joint["parent"]
                        else:
                            continue

                    new_boundary.add(child_name)

                    (child_origin, child_x, child_y, child_z) = self.links_by_name[child_name].transform2.getAsCoordinateSystem()
                    (_, parent_x, parent_y, parent_z) = self.links_by_name[occ_name].transform2.getAsCoordinateSystem()

                    if not self.close_enough([child_x, child_y, child_z], [parent_x, parent_y, parent_z]):
                        raise RuntimeError(f"child {child_name} is rotated w.r.t parent {occ_name} in link {joint['name']}: not supported")
                    
                    parent_origin = self.link_origins[occ_name]
                    if joint['type'] != 'fixed':
                        child_origin = joint["origin"]
                    else:
                        child_origin = child_origin.asArray()

                    self.link_origins[child_name] = child_origin
                    
                    xyz = [(c-p)/self.scale for c,p in zip(child_origin, parent_origin)]

                    self.joints[joint['name']] = parts.Joint(name=joint['name'] , joint_type=joint['type'], 
                                        xyz=xyz, axis=joint['axis'], 
                                        parent=occ_name, child=child_name, 
                                        upper_limit=joint['upper_limit'], lower_limit=joint['lower_limit'])
                    
                    self.__add_link(self.links_by_name[child_name])

            grounded_occ.update(new_boundary)
            boundary = new_boundary

        # Sanity check
        not_in_joints = set()
        unreachable = set()
        for component in self._iterate_through_occurrences():
            if component.isLightBulbOn and self.body_dict.get(self.name) is not None:
                if component.entityToken not in self.links_by_token:
                    not_in_joints.add(component.name)
                elif self.links_by_token[component.entityToken] not in grounded_occ:
                    unreachable.add(component.name)
        if not_in_joints or unreachable:
            error = "Not all components were included in the export:"
            if not_in_joints:
                error += "Not a part of any joint or rigid group: " + ", ".join(not_in_joints) + "."
            if unreachable:
                error += "Unreacheable from the grounded component via joints+links: " + ", ".join(unreachable) + "."
            raise RuntimeError(error)
