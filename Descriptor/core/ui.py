''' module: user interface'''

import adsk
import adsk.core, adsk.fusion, traceback

from . import manager

def file_dialog(ui):     
    """
    display the dialog to save the file
    """
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
            mesh_resolution = inputs.itemById('mesh_resolution')
            inertia_precision = inputs.itemById('inertia_precision')
            document_units = inputs.itemById('document_units')
            target_units = inputs.itemById('target_units')
            joint_order = inputs.itemById('joint_order')

            if cmdInput.id == 'generate':
                # User asked to generate using current settings
                print(f'{directory_path.text}, {save_mesh.value}, {mesh_resolution.selectedItem.name},\
                        {inertia_precision.selectedItem.name}, {document_units.selectedItem.name},\
                        {target_units.selectedItem.name}, {joint_order.selectedItem.name}' )

                document_manager = manager.Manager(directory_path.text, save_mesh.value, mesh_resolution.selectedItem.name, 
                                                   inertia_precision.selectedItem.name, document_units.selectedItem.name, target_units.selectedItem.name, 
                                                   joint_order.selectedItem.name)
                
                # Generate
                document_manager.run()

                print(f'{save_mesh.value}')

            elif cmdInput.id == 'preview':
                # Generate Hierarchy and Preview in panel
                document_manager = manager.Manager(directory_path.text, save_mesh.value, mesh_resolution.selectedItem.name, 
                                                   inertia_precision.selectedItem.name, document_units.selectedItem.name, target_units.selectedItem.name, 
                                                   joint_order.selectedItem.name)
                
                # Generate
                _joints = document_manager.preview()

                joints_text = inputs.itemById('jointlist')
                _txt = ''
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
    def __init__(self, ui, handlers):
        self.ui = ui
        self.handlers = handlers
        super().__init__()

    def notify(self, args):
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
            di.add('mm', True, '')
            di.add('cm', False, '')
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
            di.add('Parent', True, '')
            di.add('Child', False, '')


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


def config_settings(ui):
    '''
    
    TODO: Add preview of joint hierarchy 

    '''

    try:
        commandId = 'Joint Configuration Descriptor'
        commandDescription = 'Settings to describe a URDF file'
        commandName = 'URDF Description App'

        cmdDef = ui.commandDefinitions.itemById(commandId)
        if not cmdDef:
            cmdDef = ui.commandDefinitions.addButtonDefinition(commandId, commandName, commandDescription)

        handlers = []

        onCommandCreated = MyCreatedHandler(ui, handlers)
        cmdDef.commandCreated.add(onCommandCreated)
        handlers.append(onCommandCreated)

        cmdDef.execute()

        adsk.autoTerminate(False)

        return True 

    except:
        if ui:
            ui.messageBox('Failed:\n{}'.format(traceback.format_exc()))

            return False
