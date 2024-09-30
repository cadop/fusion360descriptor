'''
module to parse fusion file 
'''

import math
from typing import Any, Dict, List, Literal, Optional, Sequence, Set, Tuple, Union, cast
from dataclasses import dataclass, field

import adsk.core, adsk.fusion
from . import transforms
from . import parts
from . import utils
from collections import Counter, defaultdict

@dataclass(frozen=True, kw_only=True, eq=False)
class JointInfo:
    name: str
    parent: str
    child: str
    type: str = "fixed"
    origin: adsk.core.Vector3D = field(default_factory=adsk.core.Vector3D.create)
    axis: adsk.core.Vector3D = field(default_factory=adsk.core.Vector3D.create)
    upper_limit: float = 0.0
    lower_limit: float = 0.0

class Hierarchy:
    ''' hierarchy of the design space '''

    total_components = 0

    def __init__(self, component) -> None:
        ''' Initialize Hierarchy class to parse document and define component relationships.
        Uses a recursive traversal (based off of fusion example) and provides helper functions
        to get specific children and parents for nodes. 
        Parameters
        ----------
        component : [type]
            fusions root component to use for traversal
        '''        

        self.children: List["Hierarchy"] = []
        self.component: adsk.fusion.Occurrence = component
        self.name: str = component.name
        self.parent: Optional["Hierarchy"] = None
        Hierarchy.total_components += 1
        utils.log(f"... {Hierarchy.total_components}. Collected {self.name}...")

    def _add_child(self, c: "Hierarchy") -> None:
        self.children.append(c)
        c.parent = self 

    def get_children(self) -> List["Hierarchy"]:
        return self.children        

    def get_all_children(self) -> Dict[str, "Hierarchy"]:
        ''' get all children and sub children of this instance '''

        child_map = {}
        parent_stack: Set["Hierarchy"] = set()
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

    def get_flat_body(self) -> List[adsk.fusion.BRepBody]:
        ''' get a flat list of all components and child components '''

        child_list = []
        body_list: List[List[adsk.fusion.BRepBody]] = []
        parent_stack = set()

        child_set = list(self.get_all_children().values())

        if len(child_set) == 0:
            body_list.append([self.component.bRepBodies.item(x) for x in range(0, self.component.bRepBodies.count) ])

        child_list = [x.children for x in child_set if len(x.children)>0]
        childs : List[Hierarchy] = []
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

        flat_bodies: List[adsk.fusion.BRepBody] = []
        for body in body_list:
            flat_bodies.extend(body)

        return flat_bodies

    def get_all_parents(self) -> List[str]:
        ''' get all the parents of this instance '''

        child_stack: Set[Hierarchy] = set()
        child_stack.add(self)
        parent_map: List[str] = []
        while len(child_stack) != 0:
            tmp = child_stack.pop()
            if tmp.parent is None:
                return parent_map
            parent_map.append(tmp.parent.component.entityToken)    
            child_stack.add(tmp.parent)

        return parent_map

    @staticmethod
    def traverse(occurrences: adsk.fusion.OccurrenceList, parent: Optional["Hierarchy"] = None) -> "Hierarchy":
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
        
        assert occurrences
        for i in range(0, occurrences.count):
            occ = occurrences.item(i)

            cur = Hierarchy(occ)

            if parent is None: 
                pass
            else: 
                parent._add_child(cur)

            if occ.childOccurrences:
                Hierarchy.traverse(occ.childOccurrences, parent=cur)
        return cur  # type: ignore[undef]

def vector_to_str(v: Union[adsk.core.Vector3D, adsk.core.Point3D, Sequence[float]], prec: int = 2) -> str:
    """Turn a verctor into a debug printout"""
    if isinstance(v, (adsk.core.Vector3D, adsk.core.Point3D)):
        v = v.asArray()
    return f"[{', '.join(f'{x:.{prec}f}' for x in v)}]"

def rpy_to_str(rpy: Tuple[float, float, float]) -> str:
    return f"({', '.join(str(int(x/math.pi*180)) for x in rpy)})"

def ct_to_str(ct: adsk.core.Matrix3D) -> str:
    """Turn a coordinate transform matrix into a debug printout"""
    rpy = transforms.so3_to_euler(ct)
    return (f"@{vector_to_str(ct.translation)}"
            f" rpy={rpy_to_str(rpy)}"
            f" ({' '.join(('[' + ','.join(str(int(x)) for x in v.asArray()) + ']') for v in ct.getAsCoordinateSystem()[1:])})")

def get_origin(o: Optional[adsk.core.Base]) -> Union[adsk.core.Vector3D, None]:
    if isinstance(o, adsk.fusion.JointGeometry):
        return o.origin.asVector()
    elif o is None:
        return None
    elif isinstance(o, adsk.fusion.JointOrigin):
        return get_origin(o.geometry)
    else:
        raise ValueError(f"get_origin: unexpected {o} of type {type(o)}")
    
def get_context_name(c: Optional[adsk.fusion.Occurrence]) -> str:
    return c.name if c is not None else 'ROOT level'

def getMatrixFromRoot(occ: Optional[adsk.fusion.Occurrence]) -> adsk.core.Matrix3D:
    """
    Given an occurence, return its coordinate transform w.r.t the global frame
    This is inspired by https://forums.autodesk.com/t5/fusion-api-and-scripts/how-to-get-the-joint-origin-in-world-context/m-p/10051818/highlight/true#M12545,
    but somehow this is completely unneeded (and results in incorrect values) as .transform2 is already
    always in global frame, despite what the documentation says!!!
    """
    mat = adsk.core.Matrix3D.create()
    while occ is not None:
        mat.transformBy(occ.transform2)
        occ = occ.assemblyContext
    return mat

class Configurator:

    # Map to URDF type
    joint_types: Dict[adsk.fusion.JointTypes, str] = {
        adsk.fusion.JointTypes.RigidJointType: "fixed",
        adsk.fusion.JointTypes.RevoluteJointType: "revolute",
        adsk.fusion.JointTypes.SliderJointType: "prismatic",
        adsk.fusion.JointTypes.CylindricalJointType: "Cylindrical_unsupported",
        adsk.fusion.JointTypes.PinSlotJointType: "PinSlot_unsupported",
        adsk.fusion.JointTypes.PlanarJointType: "planar",
        adsk.fusion.JointTypes.BallJointType: "Ball_unsupported",
    }

    def __init__(self, root, scale: float, cm: float, name: str, name_map: Dict[str, str]) -> None:
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
        self.links_by_token: Dict[str, str] = {}
        self.links_by_name : Dict[str, adsk.fusion.Occurrence] = {}
        self.joints_dict: Dict[str, JointInfo] = {}
        self.body_dict: Dict[str, List[adsk.fusion.BRepBody]] = {}
        self.material_dict: Dict[str, Dict[str, str]] = {}
        self.color_dict: Dict[str, str] = {}
        self.links: Dict[str, parts.Link] = {} # Link class
        self.joints: Dict[str, parts.Joint] = {} # Joint class for writing to file
        self.scale = scale # Convert autodesk units to meters (or whatever simulator takes)
        self.cm = cm # Convert cm units to meters (or whatever simulator takes)
        parts.Link.scale = str(self.scale)
        self.eps = 1e-7 / self.scale
        self.base_link: Optional[adsk.fusion.Occurrence] = None
        self.component_map: Dict[str, Hierarchy] = {} # Entity tokens for each component
        self.name_map = name_map

        self.root_node: Optional[Hierarchy] = None

        self.name = name

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
        elif isinstance(a, adsk.core.Matrix3D) and isinstance(b, adsk.core.Matrix3D):
            return self.close_enough(a.asArray(), b.asArray())
        else:
            raise ValueError(f"close_enough: {type(a)} and {type(b)}: not supported")
        
    def get_scene_configuration(self):
        '''Build the graph of how the scene components are related
        '''        
        Hierarchy.total_components = 0
        utils.log("* Traversing the hierarchy *")
        self.root_node = Hierarchy(self.root)
        occ_list=self.root.occurrences.asList
        Hierarchy.traverse(occ_list, self.root_node)
        utils.log(f"* Collected {Hierarchy.total_components} components, processing *")
        self.component_map = self.root_node.get_all_children()
        utils.log("* Processing sub-bodies *")
        self.get_sub_bodies()

        return self.component_map


    def get_sub_bodies(self) -> None:
        ''' temp fix for ensuring that a top-level component is associated with bodies'''

        # write the immediate children of root node
        self.body_mapper: Dict[str, List[adsk.fusion.BRepBody]] = defaultdict(list)

        assert self.root_node is not None

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

    def get_joint_preview(self) -> Dict[str, JointInfo]:
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
        name = utils.rename_if_duplicate(self.name_map.get(oc.name, oc.name), self.links_by_name)
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

        # transform, cm -> m
        transform = self.link_origins[self.links_by_token[oc.entityToken]].copy()
        assert transform.invert()
        c_o_m = center_of_mass.copy()
        assert c_o_m.transformBy(transform)
        # It is in cm, not in design units.
        occs_dict['center_of_mass'] = [c * self.cm for c in c_o_m.asArray()]

        utils.log(f"DEBUG: {oc.name}: origin={vector_to_str(oc.transform2.translation)}, center_mass(global)={vector_to_str(prop.centerOfMass)}, center_mass(URDF)={occs_dict['center_of_mass']}")

        moments = prop.getXYZMomentsOfInertia()
        if not moments[0]:
            utils.fatal(f"Retrieving moments of inertia for {oc.name} failed")

        # https://help.autodesk.com/view/fusion360/ENU/?guid=GUID-ce341ee6-4490-11e5-b25b-f8b156d7cd97
        occs_dict['inertia'] = [_ * self.cm * self.cm for _ in transforms.origin2center_of_mass(moments[1:], prop.centerOfMass.asArray(), mass) ] ## kg / cm^2 -> kg/m^2

        return occs_dict

    def _iterate_through_occurrences(self):
        for token in self.component_map.values():
            yield token.component


    def _joints(self):
        ''' Iterates over joints list and defines properties for each joint
        (along with its relationship)
        '''

        for joint in self.root.allJoints:
            if joint.healthState in [adsk.fusion.FeatureHealthStates.SuppressedFeatureHealthState, adsk.fusion.FeatureHealthStates.RolledBackFeatureHealthState]:
                utils.log(f"Skipping joint {joint.name} (child of {joint.parentComponent.name}) as it is suppressed or rolled back")
                continue

            if joint.healthState != adsk.fusion.FeatureHealthStates.HealthyFeatureHealthState:
                utils.fatal(f"Joint {joint.name} (child of {joint.parentComponent.name}) is in unexpected Health State {joint.healthState}, {joint.errorOrWarningMessage=}")

            orig_name = joint.name
            # Rename if the joint already exists in our dictionary
            try:
                _ = joint.entityToken # Just making sure it exists

                joint_type = Configurator.joint_types[joint.jointMotion.jointType]

                occ_one = joint.occurrenceOne
                occ_two = joint.occurrenceTwo
            except RuntimeError as e:
                utils.log(f"WARNING: Failed to process joint {joint.name} (child of {joint.parentComponent.name}): {e}, {joint.isValid=}. This is likely a Fusion bug - the joint was likely deleted, but somehow we still see it. Will ignore it.")
                continue

            name = utils.rename_if_duplicate(self.name_map.get(joint.name, joint.name), self.joints_dict)
            utils.log(f"Processing joint {orig_name} of type {joint_type}, between {occ_one.name} and {occ_two.name}")

            parent = self.get_name(occ_one)
            child = self.get_name(occ_two)

            utils.log(f"DEBUG: Got from Fusion: {joint_type} {name} connecting")
            utils.log(f"DEBUG: ... {parent} @ {occ_one.transform2.translation.asArray()} and")
            utils.log(f"DEBUG: ... {child} @ {occ_two.transform2.translation.asArray()}")

            if joint_type == "fixed":
                info = JointInfo(name=name, child=child, parent=parent)

            else:
                try:
                    geom_one_origin = get_origin(joint.geometryOrOriginOne)
                except RuntimeError:
                    geom_one_origin = None
                try:
                    geom_two_origin = get_origin(joint.geometryOrOriginTwo)
                except RuntimeError:
                    geom_two_origin = None

                utils.log(f"DEBUG: ... Origin 1: {vector_to_str(geom_one_origin) if geom_one_origin is not None else None}")
                utils.log(f"DEBUG: ... Origin 2: {vector_to_str(geom_two_origin) if geom_two_origin is not None else None}")

                if occ_one.assemblyContext != occ_two.assemblyContext:
                    utils.log(f"WARNING: Non-fixed joint {name} crosses the assembly context boundary:"
                                f" {parent} is in {get_context_name(occ_one.assemblyContext)}"
                                f" but {child} is in {get_context_name(occ_two.assemblyContext)}")

                if geom_one_origin is None:
                    utils.fatal(f'Non-fixed joint {orig_name} does not have an origin, aborting')
                elif geom_two_origin is not None and not self.close_enough(geom_two_origin, geom_one_origin):
                    utils.fatal(f'Occurrences {occ_one.name} and {occ_two.name} of non-fixed {orig_name}' +
                                       f' have origins {geom_one_origin.asArray()} and {geom_two_origin.asArray()}'
                                       f' that do not coincide. Make sure the joint is "at 0 / at home" before exporting')
                        
                # Only Revolute joints have rotation axis 
                if isinstance(joint.jointMotion, adsk.fusion.RevoluteJointMotion):
                    assert joint.jointMotion.rotationLimits.isMaximumValueEnabled
                    assert joint.jointMotion.rotationLimits.isMinimumValueEnabled
                    joint_vector = joint.jointMotion.rotationAxisVector
                    # The values are in radians per
                    # https://help.autodesk.com/view/fusion360/ENU/?guid=GUID-e3fb19a1-d7ef-4b34-a6f5-76a907d6a774
                    joint_limit_max = joint.jointMotion.rotationLimits.maximumValue
                    joint_limit_min = joint.jointMotion.rotationLimits.minimumValue
                    
                    if abs(joint_limit_max - joint_limit_min) == 0:
                        joint_limit_min = -3.14159
                        joint_limit_max = 3.14159
                elif isinstance(joint.jointMotion, adsk.fusion.SliderJointMotion):
                    assert joint.jointMotion.slideLimits.isMaximumValueEnabled
                    assert joint.jointMotion.slideLimits.isMinimumValueEnabled
                    joint_vector=joint.jointMotion.slideDirectionVector

                    # The values are in cm per
                    # https://help.autodesk.com/view/fusion360/ENU/?guid=GUID-e3fb19a1-d7ef-4b34-a6f5-76a907d6a774
                    joint_limit_max = joint.jointMotion.slideLimits.maximumValue * self.cm
                    joint_limit_min = joint.jointMotion.slideLimits.minimumValue * self.cm
                else:
                    # Keep default limits for 'RigidJointMotion' or others
                    joint_vector = adsk.core.Vector3D.create()
                    joint_limit_max = 0.0
                    joint_limit_min = 0.0

                info = JointInfo(
                    name=name, child=child, parent=parent, origin=geom_one_origin, type=joint_type,
                    axis=joint_vector, upper_limit=joint_limit_max, lower_limit=joint_limit_min)

            self.joints_dict[name] = info

        # Add RigidGroups as fixed joints
        for group in self.root.allRigidGroups:
            original_group_name = group.name
            utils.log(f"DEBUG: Processing Rigid Group {original_group_name}")
            for i, occ in enumerate(group.occurrences):
                # Assumes that the first occurrence will be the parent
                if i == 0:
                    parent_occ = occ
                    continue
                rigid_group_occ_name = utils.rename_if_duplicate(original_group_name, self.joints_dict)

                parent_occ_name = self.get_name(parent_occ)  # type: ignore[undef]
                occ_name = self.get_name(occ)
                print(f"Got from Fusion: {rigid_group_occ_name}, connecting",
                      f"parent {parent_occ_name} @ {vector_to_str(parent_occ.transform2.translation)} and" # type: ignore[undef]
                      f"child {occ_name} {vector_to_str(occ.transform2.translation)}")
                self.joints_dict[rigid_group_occ_name] = JointInfo(name=rigid_group_occ_name, parent=parent_occ_name, child=occ_name)

    def __add_link(self, occ: adsk.fusion.Occurrence):
        inertia = self._get_inertia(occ)
        urdf_origin = self.link_origins[inertia['name']]
        inv = urdf_origin.copy()
        assert inv.invert()
        #fusion_origin = occ.transform2.translation.asArray()

        utils.log(f"DEBUG: link {inertia['name']} urdf_origin at {vector_to_str(urdf_origin.translation)}"
                  f" (rpy={rpy_to_str(transforms.so3_to_euler(urdf_origin))})"
                  f" and inv at {vector_to_str(inv.translation)} (rpy={rpy_to_str(transforms.so3_to_euler(inv))})")

        link = parts.Link(name = inertia['name'],
                        xyz = (u * self.cm for u in inv.translation.asArray()),
                        rpy = transforms.so3_to_euler(inv),
                        center_of_mass = inertia['center_of_mass'],
                        sub_folder = self.mesh_folder,
                        mass = inertia['mass'],
                        inertia_tensor = inertia['inertia'],
                        body_dict = self.body_dict_urdf,
                        sub_mesh = self.sub_mesh,
                        material_dict = self.material_dict,
                        visible = occ.isLightBulbOn)
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

        # Location and XYZ of the URDF link origin w.r.t Fusion global frame in Fusion units
        self.link_origins: Dict[str, adsk.core.Matrix3D] = {}

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
        for joint_name, joint_info in self.joints_dict.items():
            occurrences[joint_info.parent].append(joint_name)
            occurrences[joint_info.child].append(joint_name)
        grounded_occ = {"base_link"}
        # URDF origin at base link origin "by definition"
        assert self.base_link is not None
        self.link_origins["base_link"] = self.base_link.transform2
        self.__add_link(self.base_link)
        boundary = grounded_occ
        while boundary:
            new_boundary : Set[str] = set()
            for occ_name in boundary:
                for joint_name in occurrences[occ_name]:
                    joint = self.joints_dict[joint_name]
                    if joint.parent == occ_name:
                        child_name = joint.child
                        if child_name in grounded_occ:
                            continue
                    else:
                        assert joint.child == occ_name
                        if joint.parent not in grounded_occ:
                            # Parent is further away from base_link than the child, swap them
                            child_name = joint.parent
                        else:
                            continue

                    new_boundary.add(child_name)

                    child_origin = self.links_by_name[child_name].transform2
                    parent_origin = self.link_origins[occ_name]

                    if utils.LOG_DEBUG and self.close_enough(parent_origin.getAsCoordinateSystem()[1:], adsk.core.Matrix3D.create().getAsCoordinateSystem()[1:]) and not self.close_enough(child_origin.getAsCoordinateSystem()[1:], adsk.core.Matrix3D.create().getAsCoordinateSystem()[1:]):
                        utils.log(f"***** !!!!! rotating off the global frame's orientation")
                        utils.log(f"      Child axis: {[v.asArray() for v in child_origin.getAsCoordinateSystem()[1:]]}")

                    t = parent_origin.copy()
                    assert t.invert()

                    axis = joint.axis
                    
                    if joint.type != "fixed":
                        utils.log(f"DEBUG: for non-fixed joint {joint.name}, updating child origin from {child_origin.translation.asArray()} to {joint.origin.asArray()}")
                        child_origin = child_origin.copy()
                        child_origin.translation = joint.origin
                        tt = t.copy()
                        tt.translation = adsk.core.Vector3D.create()
                        axis = axis.copy()
                        assert axis.transformBy(tt)
                        utils.log(f"DEBUG:    and updating axis from {joint.axis.asArray()} to {axis.asArray()}")

                    self.link_origins[child_name] = child_origin

                    #transform = (*child_origin.getAsCoordinateSystem(), *parent_origin.getAsCoordinateSystem())
                    #transform = (*parent_origin.getAsCoordinateSystem(), *child_origin.getAsCoordinateSystem())
                    #t = adsk.core.Matrix3D.create()
                    #assert t.setToAlignCoordinateSystems(*transform)

                    ct = child_origin.copy()
                    assert ct.transformBy(t)

                    xyz = [c * self.cm for c in ct.translation.asArray()]
                    rpy = transforms.so3_to_euler(ct)

                    utils.log(f"DEBUG: joint {joint.name} (type {joint.type}) from {occ_name} at {vector_to_str(parent_origin.translation)} to {child_name} at {vector_to_str(child_origin.translation)} -> xyz={vector_to_str(xyz,5)} rpy={rpy_to_str(rpy)}")

                    self.joints[joint.name] = parts.Joint(name=joint.name , joint_type=joint.type, 
                                        xyz=xyz, rpy=rpy, axis=axis.asArray(), 
                                        parent=occ_name, child=child_name, 
                                        upper_limit=joint.upper_limit, lower_limit=joint.lower_limit)
                    
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
            utils.fatal(error)
