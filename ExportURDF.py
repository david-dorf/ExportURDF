import adsk.core
import adsk.fusion
import adsk.cam
import traceback


def run(context):
    ui = None
    try:
        app = adsk.core.Application.get()
        ui = app.userInterface
        design = adsk.fusion.Design.cast(app.activeProduct)
        rootComp = design.rootComponent
        allComps = rootComp.allComponents
        robotName = rootComp.name.replace(' ', '_')
        folderOpener = ui.createFolderDialog()
        folderOpener.title = 'Select folder to save URDF'
        folderOpener.showDialog()

        ui.messageBox('URDF successfully exported!')

    except:
        if ui:
            ui.messageBox('Failed:\n{}'.format(traceback.format_exc()))
