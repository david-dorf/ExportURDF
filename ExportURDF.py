import adsk.core
import adsk.fusion
import adsk.cam
import traceback
import os


def run(context):
    xmlHeader = """<?xml version = "1.0" ?>\n"""
    robotHeader = """<robot name = "%s">\n"""
    linkTemplate = """
    <link name = "%s">
        <visual>
            <origin xyz = "%f %f %f" rpy = "0 0 0"/>
            <geometry>
                <mesh filename = "%s" scale = "0.001 0.001 0.001"/>
            </geometry>
        </visual>
        <collision>
            <origin xyz = "%f %f %f" rpy = "0 0 0"/>
            <geometry>
                <mesh filename = "%s" scale = "0.001 0.001 0.001"/>
            </geometry>
        </collision>
        <inertial>
            <origin xyz = "%f %f %f" rpy = "0 0 0"/>
            <mass value = "%f"/>
            <inertia ixx = "%f" ixy = "%f" ixz = "%f" iyy = "%f" iyz = "%f" izz = "%f"/>
        </inertial>
    </link>
    """
    jointTemplate = """
    <joint name = "%s" type = "continuous">
        <origin xyz = "%f %f %f" rpy = "%f %f %f"/>
        <parent link = "%s"/>
        <child link = "%s"/>
        <axis xyz = "%f %f %f"/>
    </joint>
    """
    robotFooter = """\n</robot>"""

    ui = None
    try:
        app = adsk.core.Application.get()
        ui = app.userInterface
        design = adsk.fusion.Design.cast(app.activeProduct)
        exporter = design.exportManager
        rootComp = design.rootComponent
        robotName = rootComp.name.replace(' ', '_').replace(':', '_')
        folderOpener = ui.createFolderDialog()
        folderOpener.title = 'Select folder to save URDF'
        dialogResult = folderOpener.showDialog()
        if dialogResult != adsk.core.DialogResults.DialogOK:
            return
        folderPath = folderOpener.folder
        try:
            os.mkdir(os.path.join(folderPath, robotName))
            os.mkdir(os.path.join(folderPath, robotName, 'urdf'))
            os.mkdir(os.path.join(folderPath, robotName, 'meshes'))
        except:
            returnValue, cancelled = ui.inputBox(
                'Folder already exists. Do you want to overwrite? Enter Y or N', 'Warning', 'N')
            if returnValue.upper() == 'Y':
                pass
            elif returnValue.upper() == 'N' or cancelled:
                return
            else:
                ui.messageBox('Invalid input. Exiting...')
                return
        urdfFile = open(os.path.join(
            folderPath, robotName, 'urdf', robotName + '.urdf'), 'w')
        urdfFile.write(xmlHeader)
        urdfFile.write(robotHeader % robotName)
        for link in rootComp.occurrences:
            physProp = link.getPhysicalProperties()
            link_origin = physProp.centerOfMass
            returnValue, xx, yy, zz, xy, yz, xz = physProp.getXYZMomentsOfInertia()
            kgcm2_to_kgm2 = 1e-6
            parsed_name = link.name.replace(' ', '_').replace(':', '_')
            mesh_name = os.path.join('meshes', parsed_name + '.stl')
            urdfFile.write(linkTemplate % (parsed_name,
                                           link_origin.x,
                                           link_origin.y,
                                           link_origin.z,
                                           mesh_name,
                                           link_origin.x,
                                           link_origin.y,
                                           link_origin.z,
                                           mesh_name,
                                           link_origin.x,
                                           link_origin.y,
                                           link_origin.z,
                                           physProp.mass,
                                           xx * kgcm2_to_kgm2,
                                           xy * kgcm2_to_kgm2,
                                           xz * kgcm2_to_kgm2,
                                           yy * kgcm2_to_kgm2,
                                           yz * kgcm2_to_kgm2,
                                           zz * kgcm2_to_kgm2,))
            meshPath = os.path.join(folderPath, robotName, mesh_name)
            stlExportOptions = exporter.createSTLExportOptions(
                link.component, meshPath)
            stlExportOptions.sendToPrintUtility = False
            stlExportOptions.isBinaryFormat = True
            stlExportOptions.meshRefinement = adsk.fusion.MeshRefinementSettings.MeshRefinementLow
            exporter.execute(stlExportOptions)
        for joint in rootComp.joints:
            joint_origin = joint.geometryOrOriginOne
            joint_axis = joint.jointMotion.rotationAxisVector
            urdfFile.write(jointTemplate % (joint.name,
                                            joint_origin.origin.x,
                                            joint_origin.origin.y,
                                            joint_origin.origin.z,
                                            joint_origin.origin.x,
                                            joint_origin.origin.y,
                                            joint_origin.origin.z,
                                            joint.occurrenceOne.name,
                                            joint.occurrenceTwo.name,
                                            joint_axis.x,
                                            joint_axis.y,
                                            joint_axis.z))
        urdfFile.write(robotFooter)
        ui.messageBox('Exported URDF to ' + folderPath)

    except:
        if ui:
            ui.messageBox('Failed:\n{}'.format(traceback.format_exc()))
