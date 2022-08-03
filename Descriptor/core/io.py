import os
import adsk, adsk.core, adsk.fusion
from . import parts
from . import parser
from . import manager

def visible_to_stl(design, save_dir, root, accuracy,component_map):  
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
    accuracy: int
        accuracy value to use for stl export
    component_map: list
        list of all bodies to use for stl export
    """
          
    # create a single exportManager instance
    exporter = design.exportManager

    # get the script location
    save_dir = os.path.join(save_dir,'meshes')
    try: os.mkdir(save_dir)
    except: pass


    # Export top-level occurrences
    occ = root.occurrences.asList
    # hack for correct stl placement by turning off all visibility first
    visible_components = []
    for oc in occ:
        if oc.isLightBulbOn:
            visible_components.append(oc)
            oc.isLightBulbOn = False

    # Go through the visible components, turning on and off
    # Filename
    for oc in visible_components:
        # hack for correct stl placement
        # Turn on body (all components should have been turned off before)
        # export full body
        # turn back off body
        # coor = oc.transform.getAsCoordinateSystem()

        

        oc.isLightBulbOn = True

        # creates STL for all bodies within component, used for URDF collision
        file_name = oc.name.replace(':','_').replace(' ','')
        file_name = os.path.join(save_dir, file_name )              
        print(f'Saving {file_name}')
        # create stl exportOptions
        stl_options = exporter.createSTLExportOptions(root, file_name)
        stl_options.sendToPrintUtility = False
        stl_options.isBinaryFormat = True
        stl_options.meshRefinement = accuracy
        exporter.execute(stl_options)
        

        # for each component, get each body within the component
        # hack to turn off each body within the component
        # creates STL for each body
        bodies = []
        bodies = component_map[oc.entityToken].get_flat_body()
        #checks for child component within "oc" component
        if oc.childOccurrences: 
                bodies.extend([oc.bRepBodies.item(x) for x in range(0, oc.bRepBodies.count) ])
                oc_list = oc.childOccurrences
                for o in oc_list:
                    body_lst_ext = component_map[o.entityToken].get_flat_body()
                    bodies.extend(body_lst_ext)

        visible_bodies = []
        for bod in bodies:
            if bod.isLightBulbOn:
                visible_bodies.append(bod)
                bod.isLightBulbOn = False #turning off all bodies
        
        for bod in visible_bodies:
            #turn on each body individually
            bod.isLightBulbOn = True

            #export the component's body              
            file_name = oc.name.replace(':','_').replace(' ','')
            file_name = os.path.join(save_dir, file_name )  
            file_name = file_name + "_" + bod.name.replace(':','_').replace(' ','')
            print(f'Saving {file_name}')

            # create stl exportOptions
            stl_options = exporter.createSTLExportOptions(root, file_name)
            stl_options.sendToPrintUtility = False
            stl_options.isBinaryFormat = True
            stl_options.meshRefinement = accuracy
            exporter.execute(stl_options)
            bod.isLightBulbOn = False
            # this way, we are able to get the correct stl placement (each body within a component needs its own stl file)

        # The occurrence back off to not intefere with next export
        oc.isLightBulbOn = False

    # Turn back on all the components that were on before
    for oc in visible_components:
        oc.isLightBulbOn = True


class Writer:

    def __init__(self) -> None:
        pass

    def write_link(self, config, file_name):
        ''' Write links information into urdf file_name
        
        Parameters
        ----------
        config : Configurator
            root nodes instance of configurator class
        file_name: str
            urdf full path

        '''

        with open(file_name, mode='a') as f:
            for _, link in config.links.items():  
                f.write(f'{link.link_xml}\n')

    def write_joint(self, file_name, config):
        ''' Write joints and transmission information into urdf file_name
            
        Parameters
        ----------
        file_name: str
            urdf full path
        config : Configurator
            root nodes instance of configurator class

        '''
        
        with open(file_name, mode='a') as f:
            for _, joint in config.joints.items():
                f.write(f'{joint.joint_xml}\n')


    def write_urdf(self, save_dir, config):
        ''' Write each component of the xml structure to file

        Parameters
        ----------
        save_dir : str
            path to save file
        config : Configurator
            root nodes instance of configurator class
        '''        

        save_dir = os.path.join(save_dir,'urdf')
        try: os.mkdir(save_dir)
        except: pass
        file_name = os.path.join(save_dir, f'{config.name}.urdf')  # the name of urdf file

        with open(file_name, mode='w') as f:
            f.write('<?xml version="1.0" ?>\n')
            f.write(f'<robot name="{config.name}">\n\n')
            f.write('<material name="silver">\n')
            f.write('  <color rgba="0.700 0.700 0.700 1.000"/>\n')
            f.write('</material>\n\n')

        self.write_link(config, file_name)
        self.write_joint(file_name, config)

        with open(file_name, mode='a') as f:
            f.write('</robot>\n')

def write_hello_pybullet(robot_name, save_dir):
    ''' Writes a sample script which loads the URDF in pybullet

    Modified from https://github.com/yanshil/Fusion2PyBullet

    Parameters
    ----------
    robot_name : str
        name to use for directory
    save_dir : str
        path to store file
    '''    

    robot_urdf = f'{robot_name}.urdf' ## basename of robot.urdf
    file_name = os.path.join(save_dir,'hello_bullet.py')
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
    hello_pybullet = hello_pybullet.replace('TEMPLATE.urdf', robot_urdf)
    with open(file_name, mode='w') as f:
        f.write(hello_pybullet)
        f.write('\n')
