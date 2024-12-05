import os.path, sys, fileinput
from typing import Dict, List, Sequence, Tuple
from xml.etree.ElementTree import Element, ElementTree, SubElement
import xml.etree.ElementTree as ET
import adsk, adsk.core, adsk.fusion

from .parser import Configurator
from .parts import Joint, Link
from . import utils
from shutil import copytree


def visible_to_stl(
    design: adsk.fusion.Design,
    save_dir: str,
    root: adsk.fusion.Component,
    accuracy: adsk.fusion.MeshRefinementSettings,
    sub_mesh: bool,
    body_mapper: Dict[str, List[Tuple[adsk.fusion.BRepBody, str]]],
    _app,
):
    """
    export top-level components as a single stl file into "save_dir/"

    Parameters
    ----------
    design: adsk.fusion.Design
        fusion design document
    save_dir: str
        directory path to save
    root: adsk.fusion.Component
        root component of the design
    accuracy: adsk.fusion.MeshRefinementSettings enum
        accuracy value to use for stl export
    component_map: list
        list of all bodies to use for stl export
    """

    # create a single exportManager instance
    exporter = design.exportManager

    newDoc: adsk.core.Document = _app.documents.add(adsk.core.DocumentTypes.FusionDesignDocumentType, True)
    newDes = newDoc.products.itemByProductType("DesignProductType")
    assert isinstance(newDes, adsk.fusion.Design)
    newRoot = newDes.rootComponent

    # get the script location
    save_dir = os.path.join(save_dir, "meshes")
    try:
        os.mkdir(save_dir)
    except:
        pass

    try:
        for name, bodies in body_mapper.items():
            if not bodies:
                continue

            # Create a new exporter in case its a memory thing
            exporter = design.exportManager

            occName = utils.format_name(name)
            stl_exporter(exporter, accuracy, newRoot, [b for b, _ in bodies], os.path.join(save_dir, occName))

            if sub_mesh and len(bodies) > 1:
                for body, body_name in bodies:
                    if body.isVisible:
                        stl_exporter(exporter, accuracy, newRoot, [body], os.path.join(save_dir, body_name))
    finally:
        newDoc.close(False)


def stl_exporter(exportMgr, accuracy, newRoot, body_lst, filename):
    """Copy a component to a new document, save, then delete.

    Modified from solution proposed by BrianEkins https://EkinsSolutions.com

    Parameters
    ----------
    exportMgr : _type_
        _description_
    newRoot : _type_
        _description_
    body_lst : _type_
        _description_
    filename : _type_
        _description_
    """

    tBrep = adsk.fusion.TemporaryBRepManager.get()

    bf = newRoot.features.baseFeatures.add()
    bf.startEdit()

    for body in body_lst:
        tBody = tBrep.copy(body)
        newRoot.bRepBodies.add(tBody, bf)

    bf.finishEdit()
    stlOptions = exportMgr.createSTLExportOptions(newRoot, f"{filename}.stl")
    stlOptions.meshRefinement = accuracy
    exportMgr.execute(stlOptions)

    bf.deleteMe()


class Writer:

    def __init__(self, save_dir: str, config: Configurator, extra_links: Sequence[str]) -> None:
        self.save_dir = save_dir
        self.config = config
        self.extra_links = set(extra_links)

    def write_urdf(self) -> None:
        """Write each component of the xml structure to file

        Parameters
        ----------
        save_dir : str
            path to save file
        config : Configurator
            root nodes instance of configurator class
        """

        try:
            os.mkdir(self.save_dir)
        except:
            pass
        file_name = os.path.join(self.save_dir, f"{self.config.name}.xacro")  # the name of urdf file
        if self.extra_links:
            file_name_full = os.path.join(self.save_dir, f"{self.config.name}-full.xacro")
            self.write_xacro(file_name_full, self.config.links, self.config.joints)
            main_links = set(self.config.links).difference(self.extra_links)
            links = {link: self.config.links[link] for link in main_links}
            joints = {
                joint: self.config.joints[joint]
                for joint in self.config.joints
                if self.config.joints[joint].child not in self.extra_links
            }
            self.write_xacro(file_name, links, joints)
        else:
            self.write_xacro(file_name, self.config.links, self.config.joints)

        material_file_name = os.path.join(self.save_dir, f"materials.xacro")
        self.write_materials_xacro(material_file_name)

    def write_materials_xacro(self, material_file_name) -> None:
        robot = Element("robot", {"name": self.config.name, "xmlns:xacro": "http://www.ros.org/wiki/xacro"})
        for color in self.config.color_dict:
            material = SubElement(robot, "material", {"name": color})
            SubElement(material, "color", {"rgba": self.config.color_dict[color]})

        tree = ElementTree(robot)
        ET.indent(tree, space="   ")

        with open(material_file_name, mode="wb") as f:
            tree.write(f, "utf-8", xml_declaration=True)
            f.write(b"\n")

    def write_xacro(self, file_name: str, links: Dict[str, Link], joints: Dict[str, Joint]) -> None:
        robot = Element("robot", {"xmlns:xacro": "http://www.ros.org/wiki/xacro", "name": self.config.name})
        SubElement(robot, "xacro:include", {"filename": f"$(find {self.config.name})/urdf/materials.xacro"})

        # Add dummy link since KDL does not support a root link with an inertia
        # From https://robotics.stackexchange.com/a/97510
        SubElement(robot, "link", {"name": "dummy_link"})
        assert self.config.base_link is not None
        dummy_joint = SubElement(robot, "joint", {"name": "dummy_link_joint", "type": "fixed"})
        SubElement(dummy_joint, "parent", {"link": "dummy_link"})
        SubElement(dummy_joint, "child", {"link": self.config.base_link_name})

        for _, link in links.items():
            xml = link.link_xml()
            if xml is not None:
                robot.append(xml)

        for _, joint in joints.items():
            robot.append(joint.joint_xml())

        tree = ElementTree(robot)
        ET.indent(tree, space="   ")

        with open(file_name, mode="wb") as f:
            tree.write(f, "utf-8", xml_declaration=True)
            f.write(b"\n")


def write_hello_pybullet(robot_name, save_dir) -> None:
    """Writes a sample script which loads the URDF in pybullet

    Modified from https://github.com/yanshil/Fusion2PyBullet

    Parameters
    ----------
    robot_name : str
        name to use for directory
    save_dir : str
        path to store file
    """

    robot_urdf = f"{robot_name}.urdf"  ## basename of robot.urdf
    file_name = os.path.join(save_dir, "hello_bullet.py")
    hello_pybullet = """
import pybullet as p
import os
import time
import pybullet_data
physicsClient = p.connect(p.GUI)#or p.DIRECT for non-graphical version
p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
p.setGravity(0,0,-10)
planeId = p.loadURDF("plane.urdf")
cubeStartPos = [0,0,0]
cubeStartOrientation = p.getQuaternionFromEuler([0,0,0])
dir = os.path.abspath(os.path.dirname(__file__))
robot_urdf = "TEMPLATE.urdf"
dir = os.path.join(dir,'urdf')
robot_urdf=os.path.join(dir,robot_urdf)
robotId = p.loadURDF(robot_urdf,cubeStartPos, cubeStartOrientation, 
                   # useMaximalCoordinates=1, ## New feature in Pybullet
                   flags=p.URDF_USE_INERTIA_FROM_FILE)
for i in range (10000):
    p.stepSimulation()
    time.sleep(1./240.)
cubePos, cubeOrn = p.getBasePositionAndOrientation(robotId)
print(cubePos,cubeOrn)
p.disconnect()
"""
    hello_pybullet = hello_pybullet.replace("TEMPLATE.urdf", robot_urdf)
    with open(file_name, mode="w") as f:
        f.write(hello_pybullet)
        f.write("\n")


def copy_ros2(save_dir, package_name) -> None:
    # Use current directory to find `package_ros2`
    package_ros2_path = os.path.dirname(os.path.abspath(os.path.dirname(__file__))) + "/package_ros2/"
    copy_package(save_dir, package_ros2_path)
    update_cmakelists(save_dir, package_name)
    update_package_xml(save_dir, package_name)
    update_package_name(save_dir + "/launch/robot_description.launch.py", package_name)


def copy_gazebo(save_dir, package_name) -> None:
    # Use current directory to find `gazebo_package`
    gazebo_package_path = os.path.dirname(os.path.abspath(os.path.dirname(__file__))) + "/gazebo_package/"
    copy_package(save_dir, gazebo_package_path)
    update_cmakelists(save_dir, package_name)
    update_package_xml(save_dir, package_name)
    update_package_name(save_dir + "/launch/robot_description.launch.py", package_name)  # Also include rviz alone
    update_package_name(save_dir + "/launch/gazebo.launch.py", package_name)


def copy_moveit(save_dir, package_name) -> None:
    # Use current directory to find `moveit_package`
    moveit_package_path = os.path.dirname(os.path.abspath(os.path.dirname(__file__))) + "/moveit_package/"
    copy_package(save_dir, moveit_package_path)
    update_cmakelists(save_dir, package_name)
    update_package_xml(save_dir, package_name)
    update_package_name(save_dir + "/launch/setup_assistant.launch.py", package_name)


def copy_package(save_dir, package_dir) -> None:
    try:
        os.mkdir(save_dir + "/launch")
    except:
        pass
    try:
        os.mkdir(save_dir + "/urdf")
    except:
        pass
    copytree(package_dir, save_dir, dirs_exist_ok=True)


def update_cmakelists(save_dir, package_name) -> None:
    file_name = save_dir + "/CMakeLists.txt"

    for line in fileinput.input(file_name, inplace=True):
        if "project(fusion2urdf)" in line:
            sys.stdout.write("project(" + package_name + ")\n")
        else:
            sys.stdout.write(line)


def update_package_name(file_name, package_name) -> None:
    # Replace 'fusion2urdf' with the package_name
    for line in fileinput.input(file_name, inplace=True):
        if "fusion2urdf" in line:
            sys.stdout.write(line.replace("fusion2urdf", package_name))
        else:
            sys.stdout.write(line)


def update_package_xml(save_dir, package_name) -> None:
    file_name = save_dir + "/package.xml"

    for line in fileinput.input(file_name, inplace=True):
        if "<name>" in line:
            sys.stdout.write("  <name>" + package_name + "</name>\n")
        elif "<description>" in line:
            sys.stdout.write("<description>The " + package_name + " package</description>\n")
        else:
            sys.stdout.write(line)
