''' module: user interface'''

import adsk
import adsk.core, adsk.fusion, traceback

from . import manager

def file_dialog(ui):     
    '''display the dialog to save the file

    Parameters
    ----------
    ui : [type]
        [description]

    Returns
    -------
    [type]
        [description]
    '''    

    # Set styles of folder dialog.
    folderDlg = ui.createFolderDialog()
    folderDlg.title = 'Fusion Folder Dialog' 
    
    # Show folder dialog
    dlgResult = folderDlg.showDialog()
    if dlgResult == adsk.core.DialogResults.DialogOK:
        return folderDlg.folder
    return False


class MyInputChangedHandler(adsk.core.InputChangedEventHandler):
    def __init__(self, ui):
        self.ui = ui
        super().__init__()

    def notify(self, args):
        try:
            cmd = args.firingEvent.sender
            inputs = cmd.commandInputs
            cmdInput = args.input

            # Get settings of UI
            directory_path = inputs.itemById('directory_path')
            save_mesh = inputs.itemById('save_mesh')
            sub_mesh = inputs.itemById('sub_mesh')
            mesh_resolution = inputs.itemById('mesh_resolution')
            inertia_precision = inputs.itemById('inertia_precision')
            document_units = inputs.itemById('document_units')
            target_units = inputs.itemById('target_units')
            joint_order = inputs.itemById('joint_order')
            target_platform = inputs.itemById('target_platform')

            if cmdInput.id == 'generate':
                # User asked to generate using current settings
                # print(f'{directory_path.text}, {save_mesh.value}, {mesh_resolution.selectedItem.name},\
                #         {inertia_precision.selectedItem.name}, {document_units.selectedItem.name},\
                #         {target_units.selectedItem.name}, {joint_order.selectedItem.name}' )

                document_manager = manager.Manager(directory_path.text, save_mesh.value, sub_mesh.value,
                                                   mesh_resolution.selectedItem.name, 
                                                   inertia_precision.selectedItem.name, 
                                                   document_units.selectedItem.name, 
                                                   target_units.selectedItem.name, 
                                                   joint_order.selectedItem.name, 
                                                   target_platform.selectedItem.name)
                
                # Generate
                document_manager.run()

            elif cmdInput.id == 'preview':
                # Generate Hierarchy and Preview in panel
                document_manager = manager.Manager(directory_path.text, save_mesh.value, sub_mesh.value, mesh_resolution.selectedItem.name, 
                                                   inertia_precision.selectedItem.name, document_units.selectedItem.name, target_units.selectedItem.name, 
                                                   joint_order.selectedItem.name, target_platform.selectedItem.name)
                # # Generate
                _joints = document_manager.preview()

                joints_text = inputs.itemById('jointlist')

                _txt = 'joint name: parent link-> child link\n'

                for k, j in _joints.items():
                    _txt += f'{k} : {j["parent"]} -> {j["child"]}\n' 
                joints_text.text = _txt

            elif cmdInput.id == 'save_dir':
                # User set the save directory
                save_dir = file_dialog(self.ui)
                directory_path.text = save_dir

            return True
        except:
            if self.ui:
                self.ui.messageBox('Failed:\n{}'.format(traceback.format_exc()))

class MyDestroyHandler(adsk.core.CommandEventHandler):
    def __init__(self, ui):
        self.ui = ui
        super().__init__()
    def notify(self, args):
        try:
            adsk.terminate()
        except:
            if self.ui:
                self.ui.messageBox('Failed:\n{}'.format(traceback.format_exc()))

class MyCreatedHandler(adsk.core.CommandCreatedEventHandler):
    '''[summary]

    Parameters
    ----------
    adsk : adsk.core.CommandCreatedEventHandler
        Main handler for callbacks
    '''    
    def __init__(self, ui, handlers):
        '''[summary]

        Parameters
        ----------
        ui : adsk.core.UserInterface
            main variable to interact with autodesk
        handlers : list
            list to keep reference to UI handler
        '''        
        self.ui = ui
        self.handlers = handlers
        super().__init__()

    def notify(self, args):
        ''' Construct the GUI and set aliases to be referenced later

        Parameters
        ----------
        args : adsk.core.CommandCreatedEventArgs
            UI information
        '''        
        try:
            cmd = args.command
            onDestroy = MyDestroyHandler(self.ui)
            cmd.destroy.add(onDestroy)
            
            onInputChanged = MyInputChangedHandler(self.ui)
            cmd.inputChanged.add(onInputChanged)
            
            self.handlers.append(onDestroy)
            self.handlers.append(onInputChanged)
            inputs = cmd.commandInputs


            # Show path to save
            inputs.addTextBoxCommandInput('directory_path', 'Save Directory', 'C:', 2, True)
            # Button to set the save directory
            btn = inputs.addBoolValueInput('save_dir', 'Set Save Directory', False)
            btn.isFullWidth = True

            # Add checkbox to generate/export the mesh or not
            inputs.addBoolValueInput('save_mesh', 'Save Mesh', True)

            # Add checkbox to generate/export sub meshes or not
            inputs.addBoolValueInput('sub_mesh', 'Sub Mesh', True)

            # Add dropdown to determine mesh export resolution
            di = inputs.addDropDownCommandInput('mesh_resolution', 'Mesh Resolution', adsk.core.DropDownStyles.TextListDropDownStyle)
            di = di.listItems
            di.add('Low', True, '')
            di.add('Medium', False, '')
            di.add('High', False, '')

            # Add dropdown to determine inertia calculation resolution
            di = inputs.addDropDownCommandInput('inertia_precision', 'Inertia Precision', adsk.core.DropDownStyles.TextListDropDownStyle)
            di = di.listItems
            di.add('Low', True, '')
            di.add('Medium', False, '')
            di.add('High', False, '')

            # Add dropdown to set current document units
            di = inputs.addDropDownCommandInput('document_units', 'Document Units', adsk.core.DropDownStyles.TextListDropDownStyle)
            di = di.listItems
            di.add('mm', False, '')
            di.add('cm', True, '')
            di.add('m', False, '')

            # Add dropdown to set target export units
            di = inputs.addDropDownCommandInput('target_units', 'Target Units', adsk.core.DropDownStyles.TextListDropDownStyle)
            di = di.listItems
            di.add('mm', False, '')
            di.add('cm', False, '')
            di.add('m', True, '')

            # Add dropdown to define the order that joints were defined (parent as component 1 or component 2)
            di = inputs.addDropDownCommandInput('joint_order', 'Joint Component 1', adsk.core.DropDownStyles.TextListDropDownStyle)
            di = di.listItems
            di.add('Parent', False, '')
            di.add('Child', True, '')

            # Set the type of platform to target for building XML
            di = inputs.addDropDownCommandInput('target_platform', 'Target Platform', adsk.core.DropDownStyles.TextListDropDownStyle)
            di = di.listItems
            di.add('None', True, '')
            di.add('pyBullet', False, '')
            # di.add('m', False, '') # TODO Add other methods if needed 

            # Make a button to preview the hierarchy 
            btn = inputs.addBoolValueInput('preview', 'Preview Links', False)
            btn.isFullWidth = True

            # Create tab input
            tab_input = inputs.addTabCommandInput('tab_preview', 'Preview Tabs')
            tab_input_child = tab_input.children
            # Create group
            input_group = tab_input_child.addGroupCommandInput("preview_group", "Preview")
            textbox_group = input_group.children

            # Create a textbox.
            txtbox = textbox_group.addTextBoxCommandInput('jointlist', 'Joint List', '', 8, True)
            txtbox.isFullWidth = True

            # Make a button specifically to generate based on current settings 
            btn = inputs.addBoolValueInput('generate', 'Generate', False)
            btn.isFullWidth = True


        except:
            if self.ui:
                self.ui.messageBox('Failed:\n{}'.format(traceback.format_exc()))


def config_settings(ui, ui_handlers):
    '''[summary]

    Parameters
    ----------
    ui : adsk.core.UserInterface
        part of the autodesk UI
    ui_handlers : List
        empty list to hold reference

    Returns
    -------
    bool
        success or failure
    '''

    try:
        commandId = 'Joint Configuration Descriptor'
        commandDescription = 'Settings to describe a URDF file'
        commandName = 'URDF Description App'

        cmdDef = ui.commandDefinitions.itemById(commandId)
        if not cmdDef:
            cmdDef = ui.commandDefinitions.addButtonDefinition(commandId, commandName, commandDescription)

        onCommandCreated = MyCreatedHandler(ui, ui_handlers)
        cmdDef.commandCreated.add(onCommandCreated)
        ui_handlers.append(onCommandCreated)

        cmdDef.execute()

        adsk.autoTerminate(False)

        return True 

    except:
        if ui:
            ui.messageBox('Failed:\n{}'.format(traceback.format_exc()))

            return False
