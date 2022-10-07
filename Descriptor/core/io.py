import os
import adsk, adsk.core, adsk.fusion
from . import parts
from . import parser
from . import manager
from collections import Counter, defaultdict

def visible_to_stl(design, save_dir, root, accuracy, body_dict, sub_mesh, body_mapper, _app):  
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

    # Setup new document for saving to
    des: adsk.fusion.Design = _app.activeProduct

    newDoc: adsk.core.Document = _app.documents.add(adsk.core.DocumentTypes.FusionDesignDocumentType, True) 
    newDes: adsk.fusion.Design = newDoc.products.itemByProductType('DesignProductType')
    newRoot = newDes.rootComponent

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

    # Make sure no repeated body names
    body_count = Counter()

    for oc in visible_components:
        # Create a new exporter in case its a memory thing
        exporter = design.exportManager

        occName = oc.name.replace(':', '_').replace(' ','')
        
        component_exporter(exporter, newRoot, body_mapper[oc.entityToken], os.path.join(save_dir,f'{occName}'))

        if sub_mesh:
            # get the bodies associated with this top-level component (which will contain sub-components)
            bodies = body_mapper[oc.entityToken]

            for body in bodies:
                if body.isLightBulbOn:

                    # Since there are alot of similar names, we need to store the parent component as well in the filename
                    body_name = body.name.replace(':','_').replace(' ','')
                    body_name_cnt = f'{body_name}_{body_count[body_name]}'
                    body_count[body_name] += 1

                    save_name = os.path.join(save_dir,f'{occName}_{body_name_cnt}')

                    body_exporter(exporter, newRoot, body, save_name)


def component_exporter(exportMgr, newRoot, body_lst, filename):
    ''' Copy a component to a new document, save, then delete. 

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
    '''

    tBrep = adsk.fusion.TemporaryBRepManager.get()

    bf = newRoot.features.baseFeatures.add()
    bf.startEdit()

    for body in body_lst:
        if not body.isLightBulbOn: continue
        tBody = tBrep.copy(body)
        newRoot.bRepBodies.add(tBody, bf)

    bf.finishEdit()
    stlOptions = exportMgr.createSTLExportOptions(newRoot, f'{filename}.stl')
    exportMgr.execute(stlOptions)

    bf.deleteMe()

def body_exporter(exportMgr, newRoot, body, filename):
    tBrep = adsk.fusion.TemporaryBRepManager.get()
    
    tBody = tBrep.copy(body)

    bf = newRoot.features.baseFeatures.add()
    bf.startEdit()
    newRoot.bRepBodies.add(tBody, bf)
    bf.finishEdit()

    newBody = newRoot.bRepBodies[0]

    stl_options = exportMgr.createSTLExportOptions(newBody, filename)
    stl_options.sendToPrintUtility = False
    stl_options.isBinaryFormat = True
    # stl_options.meshRefinement = accuracy
    exportMgr.execute(stl_options)                

    bf.deleteMe()

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

        with open(file_name, mode='a', encoding="utf-8") as f:
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
        
        with open(file_name, mode='a', encoding="utf-8") as f:
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

        with open(file_name, mode='w', encoding="utf-8") as f:
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
