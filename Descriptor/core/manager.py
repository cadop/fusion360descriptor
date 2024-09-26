from typing import Optional
import os
import os.path

import adsk.core
import adsk.fusion

from . import parser
from . import io
from . import utils


class Manager:
    ''' Manager class for setting params and generating URDF 
    '''    

    root: Optional[adsk.fusion.Component] = None 
    design: Optional[adsk.fusion.Design] = None
    _app: Optional[adsk.core.Application] = None

    def __init__(self, save_dir, robot_name, save_mesh, sub_mesh, mesh_resolution, inertia_precision,
                target_units, joint_order, target_platform) -> None:
        '''Initialization of Manager class 

        Parameters
        ----------
        save_dir : str
            path to directory for storing data
        save_mesh : bool
            if mesh data should be exported
        mesh_resolution : str
            quality of mesh conversion
        inertia_precision : str
            quality of inertia calculations
        document_units : str
            base units of current file
        target_units : str
            target files units
        joint_order : str
            if parent or child should be component 1
        target_platform : str
            which configuration to use for exporting urdf

        '''        
        self.save_mesh = save_mesh
        self.sub_mesh = sub_mesh

        assert self.design is not None

        doc_u = 1.0
        if self.design.unitsManager.defaultLengthUnits=='mm': doc_u = 0.001
        elif self.design.unitsManager.defaultLengthUnits=='cm': doc_u = 0.01
        elif self.design.unitsManager.defaultLengthUnits=='m': doc_u = 1.0
        else:
            raise ValueError(f"Inexpected document units: '{self.design.unitsManager.defaultLengthUnits}'")

        tar_u = 1.0
        if target_units=='mm': tar_u = 0.001
        elif target_units=='cm': tar_u = 0.01
        elif target_units=='m': tar_u = 1.0
        
        self.scale = doc_u / tar_u
        self.cm = 0.01 / tar_u      

        if inertia_precision == 'Low':
            self.inert_accuracy = adsk.fusion.CalculationAccuracy.LowCalculationAccuracy
        elif inertia_precision == 'Medium':
            self.inert_accuracy = adsk.fusion.CalculationAccuracy.MediumCalculationAccuracy
        elif inertia_precision == 'High':
            self.inert_accuracy = adsk.fusion.CalculationAccuracy.VeryHighCalculationAccuracy

        if mesh_resolution == 'Low':
            self.mesh_accuracy = adsk.fusion.MeshRefinementSettings.MeshRefinementLow
        elif mesh_resolution == 'Medium':
            self.mesh_accuracy = adsk.fusion.MeshRefinementSettings.MeshRefinementMedium
        elif mesh_resolution == 'High':
            self.mesh_accuracy = adsk.fusion.MeshRefinementSettings.MeshRefinementHigh

        if joint_order == 'Parent':
            self.joint_order = ('p','c')
        elif joint_order == 'Child':
            self.joint_order = ('c','p')
        else:
            raise ValueError(f'Order method not supported')
        
        # set the target platform
        self.target_platform = target_platform

        self.robot_name = robot_name

        # Set directory 
        self._set_dir(save_dir)

    def _set_dir(self, save_dir):
        '''sets the class instance save directory

        Parameters
        ----------
        save_dir : str
            path to save
        '''        
        # set the names
        package_name = self.robot_name + '_description'

        self.save_dir = os.path.join(save_dir, package_name)
        try: os.mkdir(self.save_dir)
        except: pass     

    def preview(self):
        ''' Get all joints in the scene for previewing joints

        Returns
        -------
        dict
            mapping of joint names with parent-> child relationship
        '''        
        
        config = parser.Configurator(Manager.root, self.scale, self.cm, self.robot_name)
        config.inertia_accuracy = self.inert_accuracy
        config.joint_order = self.joint_order
        ## Return array of tuples (parent, child)
        config.get_scene_configuration()
        return config.get_joint_preview()


    def run(self):
        ''' process the scene, including writing to directory and
        exporting mesh, if applicable
        '''
        assert Manager.root is not None
        assert Manager.design is not None

        if self._app is not None and self._app.activeViewport is not None:
            utils.viewport = self._app.activeViewport
        utils.log("*** Parsing ***")
        config = parser.Configurator(Manager.root, self.scale, self.cm, self.robot_name)
        config.inertia_accuracy = self.inert_accuracy
        config.joint_order = self.joint_order
        config.sub_mesh = self.sub_mesh
        utils.log("** Getting scene configuration **")
        config.get_scene_configuration()
        utils.log("** Parsing the configuration **")
        config.parse()

        # --------------------
        # Generate URDF
        utils.log(f"*** Generating URDF under {os.path.realpath(self.save_dir)} ***")
        writer = io.Writer()
        writer.write_urdf(self.save_dir, config)

        utils.log(f"*** Generating {self.target_platform} configuration")
        if self.target_platform == 'pyBullet':
            io.write_hello_pybullet(config.name, self.save_dir)
        elif self.target_platform == 'rviz':
            io.copy_ros2(self.save_dir, config.name)
        elif self.target_platform == 'Gazebo':
            io.copy_gazebo(self.save_dir, config.name)
        elif self.target_platform == 'MoveIt':
            io.copy_moveit(self.save_dir, config.name)

        
        # Custom STL Export
        if self.save_mesh:
            io.visible_to_stl(Manager.design, self.save_dir, Manager.root, self.mesh_accuracy, config.body_dict, self.sub_mesh, config.body_mapper, config.links_by_token, Manager._app)

