import os 
from functools import partial

import adsk 

from . import parser
from . import io


class Manager:
    root = None 
    design = None

    def __init__(self, save_dir, save_mesh, mesh_resolution, inertia_precision,
                document_units, target_units, joint_order) -> None:
        
        self.save_mesh = save_mesh
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
        
        # Set directory 
        self._set_dir(save_dir)

    def _set_dir(self, save_dir):
        # set the names        
        robot_name = Manager.root.name.split()[0]
        package_name = robot_name + '_description'

        self.save_dir = os.path.join(save_dir, package_name)
        try: os.mkdir(self.save_dir)
        except: pass     

    def preview(self):
        '''
        Get all joints in the scene for previewing joints
        '''
        
        config = parser.Configurator(Manager.root)
        config.inertia_accuracy = self.inert_accuracy
        config.joint_order = self.joint_order
        config.scale = self.scale
        ## Return array of tuples (parent, child)
        config.get_scene_configuration()
        return config.get_joint_preview()


    def run(self):

        config = parser.Configurator(Manager.root)
        config.inertia_accuracy = self.inert_accuracy
        config.scale = self.scale
        config.joint_order = self.joint_order
        config.get_scene_configuration()
        config.parse()

        # --------------------
        # Generate URDF
        writer = io.Writer()
        writer.write_urdf(self.save_dir, config)

        # io.write_hello_pybullet(config.name, self.save_dir)
        
        # Custom STL Export
        if self.save_mesh:
            io.visible_to_stl(Manager.design, self.save_dir, Manager.root, self.mesh_accuracy)

