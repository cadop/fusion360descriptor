from collections import OrderedDict
from datetime import datetime
import subprocess
from typing import Dict, List, Optional
import os
import os.path
import zoneinfo
import yaml

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
                target_units, target_platform, config_file) -> None:
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
        
        # set the target platform
        self.target_platform = target_platform

        self.robot_name = robot_name

        self.name_map: Dict[str, str] = {}
        self.merge_links: Dict[str, List[str]] = {}
        self.extra_links: List[str] = []
        self.locations: Dict[str, Dict[str,str]] = {}
        self.root_name: Optional[str] = None
        if config_file:
            with open(config_file, "rb") as yml:
                configuration = yaml.load(yml, yaml.SafeLoader)
            if not isinstance(configuration, dict):
                raise(ValueError(f"Malformed config '{config_file}': top level should be a dictionary"))
            wrong_keys = set(configuration).difference([
                "RobotName", "SaveMesh", "SubMesh", "MeshResolution", "InertiaPrecision",
                "TargetUnits", "TargetPlatform", "NameMap", "MergeLinks",
                "Locations", "Extras", "Root",
            ])
            if wrong_keys:
                raise(ValueError(f"Malformed config '{config_file}': unexpected top-level keys: {list(wrong_keys)}"))
            self.name_map = configuration.get("NameMap", {})
            if self.name_map is None:
                self.name_map = {}
            self.merge_links = configuration.get("MergeLinks", {})
            if self.merge_links is None:
                self.merge_links = {}
            self.extra_links = configuration.get("Extras", [])
            if self.extra_links is None:
                self.extra_links = []
            self.locations = configuration.get("Locations", {})
            if self.locations is None:
                self.locations = {}
            self.root_name = configuration.get("Root")

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
        assert Manager.root is not None

        config = parser.Configurator(Manager.root, self.scale, self.cm, self.robot_name, self.name_map, self.merge_links, self.locations, self.extra_links, self.root_name)
        config.inertia_accuracy = self.inert_accuracy
        ## Return array of tuples (parent, child)
        config.get_scene_configuration()
        return config.get_joint_preview()

    @staticmethod
    def get_git_info() -> str:
        my_dir = os.path.abspath(os.path.dirname(__file__))
        git = ["git", "-C", my_dir]
        def call_git(cmd: List[str]) -> str:
            cmd = git + cmd
            out = subprocess.check_output(cmd, stderr=subprocess.DEVNULL)
            return out.decode("utf-8").strip()
        try:
            if call_git(['rev-parse', '--is-inside-work-tree']) != "true":
                return ""
            url = ""
            try:
                branch = call_git(['rev-parse', '--abbrev-ref', '--symbolic-full-name', '@{u}'])
                if "/" in branch:
                    remote = branch.split('/')[0]
                    url = call_git(['remote', 'get-url', remote])
            except subprocess.CalledProcessError:
                pass
            commit_hash = call_git(["rev-parse", "HEAD"])
            commit_timestamp = call_git(["log", "-1", "--format=%ct"])
            commit_author = call_git(["log", "-1", "--format=%an"])
            commit_datetime = datetime.fromtimestamp(int(commit_timestamp), tz=zoneinfo.ZoneInfo("UTC"))
            commit_date = commit_datetime.astimezone().strftime("%A, %B %d, %Y at %I:%M %p %Z")
            if url:
                repo_name = url
            else:
                repo_name = os.path.basename(os.path.dirname(os.path.dirname(my_dir)))
            return f"{repo_name} rev {commit_hash} dated {commit_date} by {commit_author}"
        except subprocess.CalledProcessError:
            return ""

    def run(self):
        ''' process the scene, including writing to directory and
        exporting mesh, if applicable
        '''
        assert Manager.root is not None
        assert Manager.design is not None

        utils.start_log_timer()
        if self._app is not None and self._app.activeViewport is not None:
            utils.viewport = self._app.activeViewport
        utils.log("*** Parsing ***")
        config = parser.Configurator(Manager.root, self.scale, self.cm, self.robot_name, self.name_map, self.merge_links, self.locations, self.extra_links, self.root_name)
        config.inertia_accuracy = self.inert_accuracy
        config.sub_mesh = self.sub_mesh
        utils.log("** Getting scene configuration **")
        config.get_scene_configuration()
        utils.log("** Parsing the configuration **")
        config.parse()

        # --------------------
        # Generate URDF
        utils.log(f"*** Generating URDF under {os.path.realpath(self.save_dir)} ***")
        self.urdf_dir = os.path.join(self.save_dir,'urdf')
        writer = io.Writer(self.urdf_dir, config)
        writer.write_urdf()

        if config.locs:
            dict = {l: {loc.name: {"xyz": loc.xyz, "rpy": loc.rpy} for loc in locs} for l, locs in config.locs.items()}
            with open(os.path.join(self.urdf_dir, "locations.yaml"), "wt") as f:
                yaml.dump(dict, f, yaml.SafeDumper, default_flow_style=None, sort_keys=False, indent=3)

        with open(os.path.join(self.urdf_dir, "fusion2urdf.txt"), "wt") as f:
            git_info = self.get_git_info()
            if git_info:
                git_info = f"\n\tusing {git_info}"
            f.write(f"URDF structure created from Fusion Model {Manager.root.name}{git_info}:\n")
            for s in config.tree_str:
                f.write(s)
                f.write("\n")

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
            utils.log("*** Generating mesh STLs ***")
            io.visible_to_stl(Manager.design, self.save_dir, Manager.root, self.mesh_accuracy, self.sub_mesh, config.body_dict, Manager._app)
        utils.log(f"*** Done! Time elapsed: {utils.time_elapsed():.1f}s ***")
        if utils.all_warnings:
            utils.log(f"There were {len(utils.all_warnings)} warnings!\n\t" + "\n\t".join(utils.all_warnings))
            utils.all_warnings = []

