'''
module to parse fusion file 
'''

import math
from typing import Any, Dict, Iterable, List, Literal, Optional, Sequence, Set, Tuple, Union, cast
from dataclasses import dataclass, field

import adsk.core, adsk.fusion
import numpy as np
from . import transforms
from . import parts
from . import utils
from collections import OrderedDict, defaultdict

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
        if utils.LOG_DEBUG:
            utils.log(f"... {Hierarchy.total_components}. Collected {self.name}...")

    def _add_child(self, c: "Hierarchy") -> None:
        self.children.append(c)
        c.parent = self 

    def get_children(self) -> List["Hierarchy"]:
        return self.children        

    def get_all_children(self) -> Dict[str, "Hierarchy"]:
        ''' get all children and sub children of this instance '''

        child_map = OrderedDict()
        parent_stack: List["Hierarchy"] = []
        parent_stack += self.get_children()
        while parent_stack:
            # Pop an element form the stack (order shouldn't matter)
            tmp = parent_stack.pop(0)
            # Add this child to the map
            # use the entity token, more accurate than the name of the component (since there are multiple)
            child_map[tmp.component.entityToken] = tmp 
            parent_stack += tmp.get_children()
        return child_map

    def get_flat_body(self) -> List[adsk.fusion.BRepBody]:
        ''' get a flat list of all components and child components '''

        child_list = []
        body_list: List[List[adsk.fusion.BRepBody]] = []

        child_set = list(self.get_all_children().values())

        if len(child_set) == 0:
            body_list.append(list(self.component.bRepBodies))

        child_list = [x.children for x in child_set if len(x.children)>0]
        parent_stack : List[Hierarchy] = []
        for c in child_list:
            for _c in c:
                parent_stack.append(_c)

        closed_set = set()

        while len(parent_stack) != 0:
            # Pop an element form the stack (order shouldn't matter)
            tmp = parent_stack.pop()
            closed_set.add(tmp)
            # Get any bodies directly associated with this component
            if tmp.component.bRepBodies.count > 0:
                body_list.append(list(tmp.component.bRepBodies))

            # Check if this child has children
            if len(tmp.children)> 0:
                # add them to the parent_stack
                child_set = list(self.get_all_children().values())

                child_list = [x.children for x in child_set if len(x.children)>0]
                for c in child_list:
                    for _c in c:
                        if _c not in closed_set:
                            parent_stack.append(_c)

        flat_bodies: List[adsk.fusion.BRepBody] = []
        for body in body_list:
            flat_bodies.extend(body)

        return flat_bodies

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
        utils.fatal(f"parser.get_origin: unexpected {o} of type {type(o)}")
    
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

    def __init__(self, root: adsk.fusion.Component, scale: float, cm: float, name: str, name_map: Dict[str, str], merge_links: Dict[str, List[str]], locations: Dict[str, Dict[str, str]], extra_links: Sequence[str], root_name: Optional[str]) -> None:
        ''' Initializes Configurator class to handle building hierarchy and parsing
        Parameters
        ----------
        root : [type]
            root component of design document
        '''        
        # Export top-level occurrences
        self.root = root
        self.occ = root.occurrences.asList
        self.inertia_accuracy = adsk.fusion.CalculationAccuracy.LowCalculationAccuracy

        self.sub_mesh = False
        self.links_by_token: Dict[str, str] = OrderedDict()
        self.links_by_name : Dict[str, adsk.fusion.Occurrence] = OrderedDict()
        self.joints_dict: Dict[str, JointInfo] = OrderedDict()
        self.body_dict: Dict[str, List[Tuple[adsk.fusion.BRepBody, str]]] = OrderedDict()
        self.material_dict: Dict[str, str] = OrderedDict()
        self.color_dict: Dict[str, str] = OrderedDict()
        self.links: Dict[str, parts.Link] = OrderedDict() # Link class
        self.joints: Dict[str, parts.Joint] = OrderedDict() # Joint class for writing to file
        self.locs: Dict[str, List[parts.Location]] = OrderedDict()
        self.scale = scale # Convert autodesk units to meters (or whatever simulator takes)
        self.cm = cm # Convert cm units to meters (or whatever simulator takes)
        parts.Link.scale = str(self.scale)
        self.eps = 1e-7 / self.scale
        self.base_link: Optional[adsk.fusion.Occurrence] = None
        self.component_map: Dict[str, Hierarchy] = OrderedDict() # Entity tokens for each component
        self.bodies_collected: Set[str] = set() # For later sanity checking - all bodies passed to URDF
        self.name_map = name_map
        self.merge_links = merge_links
        self.locations = locations
        self.extra_links = set(extra_links)

        self.root_node: Optional[Hierarchy] = None
        self.root_name = root_name

        self.name = name
        self.mesh_folder = f'{name}/meshes/'

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
            utils.fatal(f"parser.Configurator.close_enough: {type(a)} and {type(b)}: not supported")
        
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
            top_level_body = [x for x in top_level_body if x.isVisible]
            
            # add to the body mapper
            if top_level_body != []:
                self.body_mapper[v.component.entityToken].extend(top_level_body)

            while children:
                cur = children.pop()
                children.update(cur.children)
                sub_level_body = [cur.component.bRepBodies.item(x) for x in range(0, cur.component.bRepBodies.count) ]
                sub_level_body = [x for x in sub_level_body if x.isVisible ]
                
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
        self._links()
        self._materials()
        self._build()

    def _base(self):
        ''' Get the base link '''
        for oc in self._iterate_through_occurrences():
            # Get only the first grounded link
            if (
                (self.root_name is None and oc.isGrounded) or 
                (self.root_name is not None and self.root_name == oc.name) or 
                (self.root_name is not None and self.root_name in self.merge_links and self.merge_links[self.root_name][0] == oc.name)
            ):
                # We must store this object because we cannot occurrences
                self.base_link = oc
                break
        if self.base_link is None:
            if self.root_name is None:
                utils.fatal("Failed to find a grounded occurrence for URDF root. Make one of the Fusion occurrences grounded or specify 'Root: name' in the configuration file")
            else:
                utils.fatal(f"Occurrence or merge link '{self.root_name}' specified in the 'Root:' section of the configuration file not found in the design")
        self.get_name(self.base_link)

    def get_name(self, oc: adsk.fusion.Occurrence) -> str:
        if oc.entityToken in self.links_by_token:
            return self.links_by_token[oc.entityToken]
        name = utils.rename_if_duplicate(self.name_map.get(oc.name, oc.name), self.links_by_name)
        self.links_by_name[name] = oc
        self.links_by_token[oc.entityToken] = name
        utils.log(f"DEBUG: link '{oc.name}' ('{oc.fullPathName}') became '{name}'")
        return name   
    
    def _get_inertia(self, oc: adsk.fusion.Occurrence):
        occs_dict = {}

        prop = oc.getPhysicalProperties(self.inertia_accuracy)

        mass = prop.mass  # kg

        # Iterate through bodies, only add mass of bodies that are visible (lightbulb)
        body_lst = self.component_map[oc.entityToken].get_flat_body()

        if len(body_lst) > 0:
            for body in body_lst:
                # Check if this body is hidden
                #  
                # body = oc.bRepBodies.item(i)
                if not body.isVisible:
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

    def _iterate_through_occurrences(self) -> Iterable[adsk.fusion.Occurrence]:
        for token in self.component_map.values():
            yield token.component


    def _joints(self):
        ''' Iterates over joints list and defines properties for each joint
        (along with its relationship)
        '''

        for joint in self.root.allJoints:
            if joint.healthState in [adsk.fusion.FeatureHealthStates.SuppressedFeatureHealthState, adsk.fusion.FeatureHealthStates.RolledBackFeatureHealthState]:
                continue
            if isinstance(joint.jointMotion, adsk.fusion.RevoluteJointMotion):
                if joint.jointMotion.rotationValue != 0.0:
                    utils.log(f"WARNING: joint {joint.name} was not at 0, rotating it to 0")
                    joint.jointMotion.rotationValue = 0.0
            elif isinstance(joint.jointMotion, adsk.fusion.SliderJointMotion):
                if joint.jointMotion.slideValue != 0.0:
                    utils.log(f"WARNING: joint {joint.name} was not at 0, sliding it to 0")
                    joint.jointMotion.slideValue = 0.0

        for joint in sorted(self.root.allJoints, key=lambda joint: joint.name):
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

            if occ_one is None or occ_two is None:
                utils.log(f"WARNING: Failed to process joint {joint.name} (child of {joint.parentComponent.name}): {joint.isValid=}: occ_one is {None if occ_one is None else occ_one.name}, occ_two is {None if occ_two is None else occ_two.name}")
                continue

            name = utils.rename_if_duplicate(self.name_map.get(joint.name, joint.name), self.joints_dict)

            parent = self.get_name(occ_one)
            child = self.get_name(occ_two)

            if utils.LOG_DEBUG:
                utils.log(f"... Processing joint {orig_name}->{name} of type {joint_type}, between {occ_one.name}->{parent} and {occ_two.name}->{child}")

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
                    utils.log(f"DEBUG: Non-fixed joint {name} crosses the assembly context boundary:"
                                f" {parent} is in {get_context_name(occ_one.assemblyContext)}"
                                f" but {child} is in {get_context_name(occ_two.assemblyContext)}")

                if geom_one_origin is None:
                    utils.fatal(f'Non-fixed joint {orig_name} does not have an origin, aborting')
                elif geom_two_origin is not None and not self.close_enough(geom_two_origin, geom_one_origin):
                    utils.log(f'WARNING: Occurrences {occ_one.name} and {occ_two.name} of non-fixed {orig_name}' +
                                       f' have origins {geom_one_origin.asArray()} and {geom_two_origin.asArray()}'
                                       f' that do not coincide.')
                        
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
                        # Rotation is unlimited
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
        for group in sorted(self.root.allRigidGroups, key=lambda group: group.name):
            original_group_name = group.name
            try:
                if group.isSuppressed:
                    utils.log(f"WARNING: Skipping suppressed rigid group {original_group_name} (child of {group.parentComponent.name})")
                    continue
                if not group.isValid:
                    utils.log(f"WARNING: skipping invalid rigid group {original_group_name} (child of {group.parentComponent.name})")
                    continue
            except RuntimeError as e:
                utils.log(f"WARNING: skipping invalid rigid group {original_group_name}: (child of {group.parentComponent.name}) {e}")
                continue
            utils.log(f"DEBUG: Processing rigid group {original_group_name}: {[(occ.name if occ else None) for occ in group.occurrences]}")
            parent_occ: Optional[adsk.fusion.Occurrence] = None
            for occ in group.occurrences:
                if occ is None:
                    continue
                elif parent_occ is None:
                    # Assumes that the first occurrence will be the parent
                    parent_occ = occ
                    continue
                rigid_group_occ_name = utils.rename_if_duplicate(original_group_name, self.joints_dict)

                parent_occ_name = self.get_name(parent_occ)  # type: ignore[undef]
                occ_name = self.get_name(occ)
                print(f"Got from Fusion: {rigid_group_occ_name}, connecting",
                      f"parent {parent_occ_name} @ {vector_to_str(parent_occ.transform2.translation)} and" # type: ignore[undef]
                      f"child {occ_name} {vector_to_str(occ.transform2.translation)}")
                self.joints_dict[rigid_group_occ_name] = JointInfo(name=rigid_group_occ_name, parent=parent_occ_name, child=occ_name)
        
        self.assembly_tokens: Set[str] = set()
        for occ in self.root.allOccurrences:
            if occ.childOccurrences.count > 0:
                # It's an assembly
                self.assembly_tokens.add(occ.entityToken)

    def get_assembly_links(self, occ: adsk.fusion.Occurrence, parent_included: bool) -> List[str]:
        result: List[str] = []
        for child in occ.childOccurrences:
            child_included = child.entityToken in self.links_by_token
            if child_included:
                result.append(self.links_by_token[child.entityToken])
            child_included = parent_included or child_included
            if child.entityToken in self.assembly_tokens:
                result += self.get_assembly_links(child, child_included)
            elif not child_included:
                result.append(self.get_name(child))
        utils.log(f"DEBUG: get_assembly_links({occ.name}) = {result}")
        return result
    
    @staticmethod
    def _mk_pattern(name: str) -> Union[str, Tuple[str,str]]:
        c = name.count("*")
        if c > 1:
            utils.fatal(f"Occurrance name pattern '{name}' is invalid: only one '*' is supported")
        if c:
            pref, suff = name.split("*", 1)
            return (pref, suff)
        return name
    
    @staticmethod
    def _match(candidate: str, pattern: Union[str, Tuple[str,str]]) -> bool:
        if isinstance(pattern, str):
            return candidate == pattern
        pref, suff = pattern
        return len(candidate) >= len(pref) + len(suff) and candidate.startswith(pref) and candidate.endswith(suff)

    def _resolve_name(self, name:str) -> adsk.fusion.Occurrence:
        if "+" in name:
            name_parts = name.split("+")
            l = len(name_parts)
            patts = [self._mk_pattern(p) for p in name_parts]
            candidate: Optional[adsk.fusion.Occurrence]= None
            for occ in self._iterate_through_occurrences():
                path = occ.fullPathName.split("+")
                if len(path) < l:
                    continue
                mismatch = False
                for (cand, patt) in zip(path[-l:], patts):
                    if not self._match(cand, patt):
                        mismatch = True
                        break
                if mismatch:
                    continue
                if candidate is None:
                    candidate = occ
                else:
                    utils.fatal(f"Name/pattern '{name}' in configuration file matches at least two occurrences: '{candidate.fullPathName}' and '{occ.fullPathName}', update to be more specific")
            if not candidate:
                utils.fatal(f"Name/pattern '{name}' in configuration file does not match any occurrences")
            return candidate
        patt = self._mk_pattern(name)
        candidates = [occ for occ in self._iterate_through_occurrences() if self._match(occ.name, patt)]
        if not candidates:
            utils.fatal(f"Name/pattern '{name}' in configuration file does not match any occurrences")
        if len(candidates) > 1:
            utils.fatal(f"Name/pattern '{name}' in configuration file matches at least two occurrences: '{candidates[0].fullPathName}' and '{candidates[1].fullPathName}', update to be more specific")
        return candidates[0]

    def _links(self):
        self.merged_links_by_link: Dict[str, Tuple[str, List[str], List[adsk.fusion.Occurrence]]] = OrderedDict()
        self.merged_links_by_name: Dict[str, Tuple[str, List[str], List[adsk.fusion.Occurrence]]] = OrderedDict()

        for name, names in self.merge_links.items():
            if not names:
                utils.fatal(f"Invalid MergeLinks YAML config setting: merged link '{name}' is empty, which is not allowed")
            link_names = []
            for n in names:
                occ = self._resolve_name(n)
                if occ.entityToken in self.links_by_token or occ.entityToken in self.assembly_tokens:
                    link_names.append(self.get_name(occ))
                if occ.entityToken in self.assembly_tokens:
                    try:
                        link_names += self.get_assembly_links(occ, occ.entityToken in self.links_by_token)
                    except ValueError as e:
                        utils.fatal(f"Invalid MergeLinks YAML config setting: assembly '{n}' for merged link '{name}' could not be processed: {e.args[0]}")
            if name in self.links_by_name and name not in names:
                utils.fatal(f"Invalid MergeLinks YAML config setting: merged '{name}' clashes with existing Fusion link '{self.links_by_name[name].fullPathName}'; add the latter to NameMap in YAML to avoid the name clash")
            link_names = list(OrderedDict.fromkeys(link_names)) # Remove duplicates
            val = name, link_names, [self.links_by_name[n] for n in link_names]
            utils.log(f"Merged link {name} <- occurrences {link_names}")
            self.merged_links_by_name[name] = val
            for link_name in link_names:
                if link_name in self.merged_links_by_link:
                    utils.fatal(f"Invalid MergeLinks YAML config setting: {link_name} is included in two merged links: '{name}' and '{self.merged_links_by_link[link_name][0]}'")
                self.merged_links_by_link[link_name] = val

        body_names: Dict[str, Tuple[()]] = OrderedDict()
        
        renames = set(self.name_map)

        for oc in self._iterate_through_occurrences():
            renames.difference_update([oc.name])
            occ_name, _, occs = self._get_merge(oc)
            if occ_name in self.body_dict:
                continue

            oc_name = utils.format_name(occ_name)
            self.body_dict[occ_name] = []
            bodies = set()

            for sub_oc in occs:                
                sub_oc_name = utils.format_name(self.get_name(sub_oc))
                if sub_oc_name != oc_name:
                    sub_oc_name = f"{oc_name}__{sub_oc_name}"
                for body in self.body_mapper[sub_oc.entityToken]:
                    # Check if this body is hidden
                    if body.isVisible and body.entityToken not in bodies:
                        body_name = f"{sub_oc_name}__{utils.format_name(body.name)}"
                        unique_bodyname = utils.rename_if_duplicate(body_name, body_names)
                        body_names[unique_bodyname] = ()
                        self.body_dict[occ_name].append((body, unique_bodyname))
                        bodies.add(body.entityToken)
        
        if renames:
            ValueError("Invalid NameMap YAML config setting: some of the links are not in Fusion: '" + "', '".join(renames) + "'")

    def __add_link(self, name: str, occs: List[adsk.fusion.Occurrence]):
        urdf_origin = self.link_origins[name]
        inv = urdf_origin.copy()
        assert inv.invert()
        #fusion_origin = occ.transform2.translation.asArray()

        mass = 0.0
        visible = False
        center_of_mass = np.zeros(3)
        inertia_tensor = np.zeros(6)
        for occ in occs:
            inertia = self._get_inertia(occ)
            utils.log(f"DEBUG: link {occ.name} urdf_origin at {vector_to_str(urdf_origin.translation)}"
                    f" (rpy={rpy_to_str(transforms.so3_to_euler(urdf_origin))})"
                    f" and inv at {vector_to_str(inv.translation)} (rpy={rpy_to_str(transforms.so3_to_euler(inv))})")
            mass += inertia['mass']
            visible += visible or occ.isVisible
            center_of_mass += np.array(inertia['center_of_mass']) * inertia['mass']
            inertia_tensor += np.array(inertia['inertia'])
        if len(occs) == 1:
            inertia_tensor = inertia['inertia']  # type: ignore
            center_of_mass = inertia['center_of_mass']  # type: ignore
        else:
            inertia_tensor = list(inertia_tensor)
            center_of_mass = list(center_of_mass/mass)

        self.bodies_collected.update(body.entityToken for body, _ in self.body_dict[name])

        self.links[name] = parts.Link(name = utils.format_name(name),
                        xyz = (u * self.cm for u in inv.translation.asArray()),
                        rpy = transforms.so3_to_euler(inv),
                        center_of_mass = center_of_mass,
                        sub_folder = self.mesh_folder,
                        mass = mass,
                        inertia_tensor = inertia_tensor,
                        bodies = [body_name for _, body_name in self.body_dict[name]],
                        sub_mesh = self.sub_mesh,
                        material_dict = self.material_dict,
                        visible = visible)

    def __get_material(self, appearance: Optional[adsk.core.Appearance]) -> str:
        # Material should always have an appearance, but just in case
        if appearance is not None:
            # Only supports one appearance per occurrence so return the first
            for prop in appearance.appearanceProperties:
                if type(prop) == adsk.core.ColorProperty:
                    prop_name = appearance.name
                    color_name = utils.convert_german(prop_name)
                    color_name = utils.format_name(color_name)
                    self.color_dict[color_name] = f"{prop.value.red/255} {prop.value.green/255} {prop.value.blue/255} {prop.value.opacity/255}"
                    return color_name
        return "silver_default"

    def _materials(self) -> None:
        # Adapted from SpaceMaster85/fusion2urdf
        self.color_dict['silver_default'] = "0.700 0.700 0.700 1.000"

        if self.sub_mesh:
            for occ_name, bodies in self.body_dict.items():
                if len(bodies) > 1:
                    for body, body_name in bodies:
                        self.material_dict[body_name] = self.__get_material(body.appearance)
                else:
                    appearance = self.__get_material(bodies[0][0].appearance) if bodies else 'silver_default'
                    self.material_dict[utils.format_name(occ_name)] = appearance 
        else:
            for occ in self.links_by_name.values():
                occ_name, _, occs = self._get_merge(occ)
                occ = occs[0]
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
                self.material_dict[utils.format_name(occ_name)] = self.__get_material(appearance)

    def _get_merge(self, occ: adsk.fusion.Occurrence) -> Tuple[str, List[str], List[adsk.fusion.Occurrence]]:
        name = self.get_name(occ)
        if name in self.merged_links_by_link:
            return self.merged_links_by_link[name]
        return name, [name], [occ]

    def _build(self) -> None:
        ''' create links and joints by setting parent and child relationships and constructing
        the XML formats to be exported later'''

        # Location and XYZ of the URDF link origin w.r.t Fusion global frame in Fusion units
        self.link_origins: Dict[str, adsk.core.Matrix3D] = {}

        occurrences = defaultdict(list)
        for joint_name, joint_info in self.joints_dict.items():
            occurrences[joint_info.parent].append(joint_name)
            occurrences[joint_info.child].append(joint_name)
        for link_name, joints in occurrences.items():
            utils.log(f"DEBUG: {link_name} touches joints {joints}")
        assert self.base_link is not None
        self.base_link_name, base_link_names, base_link_occs = self._get_merge(self.base_link)
        grounded_occ = set(base_link_names)
        for name in [self.base_link_name] + base_link_names:
            # URDF origin at base link origin "by definition"
            self.link_origins[name] = base_link_occs[0].transform2
        self.__add_link(self.base_link_name, base_link_occs)
        boundary = grounded_occ.copy()
        fixed_links: Dict[Tuple[str,str], str] = {}
        while boundary:
            new_boundary : Set[str] = set()
            for occ_name in boundary:
                for joint_name in occurrences[occ_name]:
                    joint = self.joints_dict[joint_name]
                    if joint.parent == occ_name:
                        child_name = joint.child
                        if child_name in grounded_occ:
                            continue
                        flip_axis = True
                    else:
                        assert joint.child == occ_name
                        if joint.parent in grounded_occ:
                            continue
                        # Parent is further away from base_link than the child, swap them
                        child_name = joint.parent
                        flip_axis = False


                    parent_name, _, _ = self._get_merge(self.links_by_name[occ_name])
                    child_name, child_link_names, child_link_occs = self._get_merge(self.links_by_name[child_name])

                    child_origin = child_link_occs[0].transform2
                    parent_origin = self.link_origins[parent_name]

                    if utils.LOG_DEBUG and self.close_enough(parent_origin.getAsCoordinateSystem()[1:], adsk.core.Matrix3D.create().getAsCoordinateSystem()[1:]) and not self.close_enough(child_origin.getAsCoordinateSystem()[1:], adsk.core.Matrix3D.create().getAsCoordinateSystem()[1:]):
                        utils.log(f"***** !!!!! rotating off the global frame's orientation")
                        utils.log(f"      Child axis: {[v.asArray() for v in child_origin.getAsCoordinateSystem()[1:]]}")

                    t = parent_origin.copy()
                    assert t.invert()

                    axis = joint.axis
                    
                    if joint.type == "fixed":
                        fixed_links[(child_name, parent_name)] = joint.name
                        fixed_links[(parent_name, child_name)] = joint.name
                    else:
                        utils.log(f"DEBUG: for non-fixed joint {joint.name}, updating child origin from {ct_to_str(child_origin)} to {joint.origin.asArray()}")
                        child_origin = child_origin.copy()
                        child_origin.translation = joint.origin
                        # The joint axis is specified in the joint (==child) frame
                        tt = child_origin.copy()
                        tt.translation = adsk.core.Vector3D.create()
                        assert tt.invert()
                        axis = axis.copy()
                        assert axis.transformBy(tt)
                        if flip_axis:
                            assert axis.scaleBy(-1)
                        utils.log(f"DEBUG:    and using {ct_to_str(tt)} and {flip_axis=} to update axis from {joint.axis.asArray()} to {axis.asArray()}")

                    for name in [child_name] + child_link_names:
                        self.link_origins[name] = child_origin

                    #transform = (*child_origin.getAsCoordinateSystem(), *parent_origin.getAsCoordinateSystem())
                    #transform = (*parent_origin.getAsCoordinateSystem(), *child_origin.getAsCoordinateSystem())
                    #t = adsk.core.Matrix3D.create()
                    #assert t.setToAlignCoordinateSystems(*transform)

                    ct = child_origin.copy()
                    assert ct.transformBy(t)

                    xyz = [c * self.cm for c in ct.translation.asArray()]
                    rpy = transforms.so3_to_euler(ct)

                    utils.log(f"DEBUG: joint {joint.name} (type {joint.type}) from {parent_name} at {vector_to_str(parent_origin.translation)} to {child_name} at {vector_to_str(child_origin.translation)} -> xyz={vector_to_str(xyz,5)} rpy={rpy_to_str(rpy)}")

                    self.joints[joint.name] = parts.Joint(name=joint.name , joint_type=joint.type, 
                                    xyz=xyz, rpy=rpy, axis=axis.asArray(), 
                                    parent=parent_name, child=child_name, 
                                    upper_limit=joint.upper_limit, lower_limit=joint.lower_limit)
                    
                    self.__add_link(child_name, child_link_occs)
                    new_boundary.update(child_link_names)
                    grounded_occ.update(child_link_names)

            boundary = new_boundary
        
        disconnected_external = []
        for name in self.extra_links:
            if name in self.links_by_name:
                utils.fatal(f"Link '{name}' from the 'Extras:' section of the configuration file is not known")
            if name in self.merge_links:
                name2, _, occs = self.merged_links_by_name[name]
                if name2 == self.base_link_name:
                    utils.fatal(f"Link '{name2}' is the root link, but declared as an extra (that is, not a part of the main URDF)")
                for oc in occs:
                    self.link_origins[self.get_name(oc)] = occs[0].transform2
            else:
                if name == self.base_link_name:
                    utils.fatal(f"Link '{name}' is the root link, but declared as an extra (that is, not a part of the main URDF)")
                occs = [self.links_by_name[name]]
            self.link_origins[name] = occs[0].transform2
            self.__add_link(name, occs)
            disconnected_external.append(name)
            

        joint_children: Dict[str, List[parts.Joint]] = defaultdict(list)
        for joint in self.joints.values():
            joint_children[joint.parent].append(joint)
        tree_str = []
        def get_tree(level: int, link_name: str):
            extra = f" {self.merge_links[link_name]}" if link_name in self.merge_links else ""
            tree_str.append("   "*level + f" - Link: {link_name}{extra}")
            for j in joint_children[link_name]:
                tree_str.append("   " * (level + 1) + f" - Joint [{j.type}]: {j.name}")
                get_tree(level+2, j.child)
        get_tree(1, self.base_link_name)
        if disconnected_external:
            tree_str.append("     - \"Extras\" links:")
            for extra in disconnected_external:
                get_tree(2, extra)

        # Sanity checks
        not_in_joints = set()
        unreachable = set()
        for occ in self._iterate_through_occurrences():
            if occ.isVisible and self.body_dict.get(self.name) is not None:
                if occ.fullPathName not in self.links_by_token:
                    not_in_joints.add(occ.fullPathName)
                elif self.links_by_token[occ.fullPathName] not in grounded_occ:
                    unreachable.add(occ.fullPathName)
        for occ in self.root.allOccurrences:
            if any (b.isVisible and not b.entityToken in self.bodies_collected for b in occ.bRepBodies):
                unreachable.add(occ.fullPathName)
        if not_in_joints or unreachable:
            error = "FATAL ERROR: Not all occurrences were included in the export:"
            if not_in_joints:
                error += "Not a part of any joint or rigid group: " + ", ".join(not_in_joints) + "."
            if unreachable:
                error += "Unreacheable from the grounded occurrence via joints+links: " + ", ".join(unreachable) + "."
            utils.log(error)
        missing_joints = set(self.joints_dict).difference(self.joints)
        for joint_name in missing_joints.copy():
            joint = self.joints_dict[joint_name]
            if joint.type == "fixed":
                parent_name, _, _ = self._get_merge(self.links_by_name[joint.parent])
                child_name, _, _ = self._get_merge(self.links_by_name[joint.child])
                if parent_name == child_name:
                    utils.log(f"DEBUG: Skipped Fixed Joint '{joint_name}' that is internal for merged link {self.merged_links_by_link[joint.parent][0]}")
                    missing_joints.remove(joint_name)
                elif (parent_name, child_name) in fixed_links:
                    utils.log(f"DEBUG: Skipped Fixed Joint '{joint_name}' that is duplicative of `{fixed_links[(parent_name, child_name)]}")
                    missing_joints.remove(joint_name)
        if missing_joints:
            utils.log("\n\t".join(["FATAL ERROR: Lost joints: "] + [f"{self.joints_dict[joint].name} of type {self.joints_dict[joint].type} between {self.joints_dict[joint].parent} and {self.joints_dict[joint].child}" for joint in missing_joints]))
        extra_joints = set(self.joints).difference(self.joints_dict)
        if extra_joints:
            utils.log("FATAL ERROR: Extra joints: '" + "', '".join(sorted(extra_joints)) + "'")
        if not_in_joints or unreachable or missing_joints or extra_joints:
            utils.log("Reachable from the root:")
            utils.log("\n".join(tree_str))
            utils.fatal("Fusion structure is broken or misunderstoon by the exporter, giving up! See the full output in Text Commands console for more information.")
        self.tree_str = tree_str

        for link, locations in self.locations.items():
            if link not in self.link_origins:
                utils.fatal(f"Link {link} specified in the config file 'Locations:' section does not exist. Make sure to use the URDF name (e.g. as set via MergeLink) rather than the Fusion one")
            origin = self.link_origins[link]
            t = origin.copy()
            assert t.invert()
            self.locs[link] = []
            for loc_name, loc_occurrence in locations.items():
                if loc_occurrence in self.merge_links:
                    ct = self.link_origins[loc_occurrence].copy()
                else:
                    ct = self._resolve_name(loc_occurrence).transform2.copy()
                assert ct.transformBy(t)
                self.locs[link].append(parts.Location(loc_name, [c * self.cm for c in ct.translation.asArray()], rpy = transforms.so3_to_euler(ct)))
