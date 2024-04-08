import adsk.core
import adsk.fusion
import adsk.cam
import traceback
import os


def run(context):
    xmlHeader = """<?xml version = "1.0" ?>\n"""
    robotHeader = """<robot name = "%s">\n"""
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
            urdfFile.write(fillLinkTemplate(link))
            parsed_name = link.name.replace(' ', '_').replace(':', '_')
            mesh_name = os.path.join('meshes', parsed_name + '.stl')
            meshPath = os.path.join(folderPath, robotName, mesh_name)
            stlExportOptions = exporter.createSTLExportOptions(
                link.component, meshPath)
            stlExportOptions.sendToPrintUtility = False
            stlExportOptions.isBinaryFormat = True
            stlExportOptions.meshRefinement = adsk.fusion.MeshRefinementSettings.MeshRefinementLow
            exporter.execute(stlExportOptions)
        for joint in rootComp.joints:
            urdfFile.write(fillJointTemplate(
                joint, joint.jointMotion.jointType))
        urdfFile.write(robotFooter)
        ui.messageBox('Exported URDF to ' + folderPath)

    except:
        if ui:
            ui.messageBox('Failed:\n{}'.format(traceback.format_exc()))


def getTemplate(templateName):
    LINK = """
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
    CONTINUOUS = """
    <joint name = "%s" type = "continuous">
        <origin xyz = "%f %f %f" rpy = "%f %f %f"/>
        <parent link = "%s"/>
        <child link = "%s"/>
        <axis xyz = "%f %f %f"/>
    </joint>
    """
    FIXED = """
    <joint name = "%s" type = "fixed">
        <origin xyz = "%f %f %f" rpy = "%f %f %f"/>
        <parent link = "%s"/>
        <child link = "%s"/>
    </joint>
    """
    REVOLUTE = """
    <joint name = "%s" type = "revolute">
        <origin xyz = "%f %f %f" rpy = "%f %f %f"/>
        <parent link = "%s"/>
        <child link = "%s"/>
        <axis xyz = "%f %f %f"/>
        <limit lower = "%f" upper = "%f"/>
    </joint>
    """
    PRISMATIC = """
    <joint name = "%s" type = "prismatic">
        <origin xyz = "%f %f %f" rpy = "%f %f %f"/>
        <parent link = "%s"/>
        <child link = "%s"/>
        <axis xyz = "%f %f %f"/>
        <limit lower = "-0.1" upper = "0.1"/>
    </joint>
    """
    return locals()[templateName.upper()]


def fillLinkTemplate(link):
    link_origin = link.getPhysicalProperties().centerOfMass
    returnValue, xx, yy, zz, xy, yz, xz = link.getPhysicalProperties().getXYZMomentsOfInertia()
    kgcm2_to_kgm2 = 1e-6
    parsed_name = link.name.replace(' ', '_').replace(':', '_')
    mesh_name = os.path.join('meshes', parsed_name + '.stl')
    linkTemplate = getTemplate('link')
    return linkTemplate % (parsed_name,
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
                           link.getPhysicalProperties().mass,
                           xx * kgcm2_to_kgm2,
                           xy * kgcm2_to_kgm2,
                           xz * kgcm2_to_kgm2,
                           yy * kgcm2_to_kgm2,
                           yz * kgcm2_to_kgm2,
                           zz * kgcm2_to_kgm2,)


def fillJointTemplate(joint, jointType):
    jointDict = {0: 'continuous', 1: 'fixed',
                 2: 'revolute', 3: 'prismatic'}
    jointTypeStr = jointDict[jointType]
    jointTemplate = getTemplate(jointTypeStr)
    joint_origin = joint.geometryOrOriginOne
    if jointTypeStr == 'continuous':
        joint_axis = joint.jointMotion.rotationAxisVector
        return jointTemplate % (joint.name,
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
                                joint_axis.z)
    elif jointTypeStr == 'fixed':
        return jointTemplate % (joint.name,
                                joint_origin.origin.x,
                                joint_origin.origin.y,
                                joint_origin.origin.z,
                                joint_origin.origin.x,
                                joint_origin.origin.y,
                                joint_origin.origin.z,
                                joint.occurrenceOne.name,
                                joint.occurrenceTwo.name)
    elif jointTypeStr == 'revolute':
        joint_axis = joint.jointMotion.rotationAxisVector
        joint_limits = joint.jointMotion.rotationLimits
        return jointTemplate % (joint.name,
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
                                joint_axis.z,
                                joint_limits.minValue,
                                joint_limits.maxValue)
    elif jointTypeStr == 'prismatic':
        return jointTemplate % (joint.name,
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
                                joint_axis.z)
    else:
        raise ValueError('Invalid joint type')
