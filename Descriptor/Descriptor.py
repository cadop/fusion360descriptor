import adsk.core, adsk.fusion, adsk.cam, traceback

from .core.ui import config_settings
from .core import manager

ui_handlers = []

def run(context):
    ''' Entry point to the codebase for running the descriptor 

    Parameters
    ----------
    context : dict
        built-in requirement for Fusion link

    Returns
    -------
    bool
        success code, not really used
    '''    

    ui = None
    try:
        app = adsk.core.Application.get()
        ui  = app.userInterface
        product = app.activeProduct
        design = adsk.fusion.Design.cast(product)
        title = 'Fusion360URDF'

        root = design.rootComponent  
        # Set the global managers root component 
        manager.Manager.root = root 
        manager.Manager.design = design 

        _ = config_settings(ui, ui_handlers)
   
        print('FINISHED')
        return 0

    except:
        if ui:
            ui.messageBox('Failed:\n{}'.format(traceback.format_exc()))



