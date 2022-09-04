import os 
from functools import partial

import adsk 

from . import parser
from . import io


class Manager:
    ''' Manager class for setting params and generating URDF 
    '''    

    root = None 
    design = None
    _app = None

    def __init__(self, save_dir, save_mesh, sub_mesh, mesh_resolution, inertia_precision,
                document_units, target_units, joint_order, target_platform) -> None:
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
        if document_units=='mm': doc_u = 0.001
        elif document_units=='cm': doc_u = 0.01
        elif document_units=='m': doc_u = 1.0

        if target_units=='mm': tar_u = 0.001
        elif target_units=='cm': tar_u = 0.01
        elif target_units=='m': tar_u = 1.0
        
        self.scale = tar_u / doc_u       

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
        robot_name = Manager.root.name.split()[0]
        package_name = robot_name + '_description'

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
        
        config = parser.Configurator(Manager.root)
        config.inertia_accuracy = self.inert_accuracy
        config.joint_order = self.joint_order
        config.scale = self.scale
        ## Return array of tuples (parent, child)
        config.get_scene_configuration()
        return config.get_joint_preview()


    def run(self):
        ''' process the scene, including writing to directory and
        exporting mesh, if applicable
        '''        
        
        config = parser.Configurator(Manager.root)
        config.inertia_accuracy = self.inert_accuracy
        config.scale = self.scale
        config.joint_order = self.joint_order
        config.sub_mesh = self.sub_mesh
        config.get_scene_configuration()
        config.parse()

        # --------------------
        # Generate URDF
        writer = io.Writer()
        writer.write_urdf(self.save_dir, config)

        if self.target_platform == 'pyBullet':
            io.write_hello_pybullet(config.name, self.save_dir)
        
        # Custom STL Export
        if self.save_mesh:
            io.visible_to_stl(Manager.design, self.save_dir, Manager.root, self.mesh_accuracy, config.body_dict, self.sub_mesh, config.body_mapper, Manager._app)

