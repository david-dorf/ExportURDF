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
        if not rootComp.occurrences:
            ui.messageBox('No components found. Exiting...')
            return
        if not rootComp.joints or not rootComp.asBuiltJoints:
            ui.messageBox('No joints found. Exiting...')
            return
        if not any('base_link' in link.name for link in rootComp.occurrences):
            ui.messageBox(
                'Component named base_link not found. Please add one. Exiting...')
            return
        robotName = formatName(rootComp.name)
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
                'Folder already exists. Do you want to overwrite it? This can lead to unstable \
                 behavior. Enter Y or N', 'Warning', 'N')
            if returnValue.upper() == 'Y':
                pass
            elif returnValue.upper() == 'N' or cancelled:
                return
            else:
                ui.messageBox('Invalid input. Exiting...')
                return

        # Write URDF file header
        urdfFile = open(os.path.join(
            folderPath, robotName, 'urdf', robotName + '.urdf'), 'w')
        urdfFile.write(xmlHeader)
        urdfFile.write(robotHeader % robotName)

        # Write links and export meshes
        # TODO: Add support for rigid groups
        for link in rootComp.occurrences:
            urdfFile.write(fillLinkTemplate(link, robotName))
            parsed_name = formatName(link.name)
            mesh_name = os.path.join('meshes', parsed_name + '.stl')
            meshPath = os.path.join(folderPath, robotName, mesh_name)
            stlExportOptions = exporter.createSTLExportOptions(
                link.component, meshPath)
            stlExportOptions.sendToPrintUtility = False
            stlExportOptions.isBinaryFormat = True
            stlExportOptions.meshRefinement = adsk.fusion.MeshRefinementSettings.MeshRefinementLow
            exporter.execute(stlExportOptions)

        # Write joints
        # 0: Fixed, 1: Revolute, 2: Prismatic, 3: Continuous
        for joint in rootComp.joints:
            hasRotationLimits = joint.jointMotion.jointType == 1 and (
                joint.jointMotion.rotationLimits.isMinimumValueEnabled or
                joint.jointMotion.rotationLimits.isMaximumValueEnabled)
            if not hasRotationLimits and joint.jointMotion.jointType == 1:
                urdfFile.write(fillJointTemplate(
                    joint, 3, False))
            else:
                urdfFile.write(fillJointTemplate(
                    joint, joint.jointMotion.jointType, False))

        for joint in rootComp.asBuiltJoints:
            hasRotationLimits = joint.jointMotion.jointType == 1 and (
                joint.jointMotion.rotationLimits.isMinimumValueEnabled or
                joint.jointMotion.rotationLimits.isMaximumValueEnabled)
            if not hasRotationLimits and joint.jointMotion.jointType == 1:
                urdfFile.write(fillJointTemplate(
                    joint, 3, True))
            else:
                urdfFile.write(fillJointTemplate(
                    joint, joint.jointMotion.jointType, True))

        # Write footer
        urdfFile.write(robotFooter)
        ui.messageBox('Exported URDF to ' + folderPath)

    except:
        if ui:
            ui.messageBox('Failed:\n{}'.format(traceback.format_exc()))


def formatName(name: str) -> str:
    if 'base_link' in name:
        return 'base_link'
    else:
        return name.replace(' ', '_').replace(':', '_').replace('(', '').replace(')', '')


def getTemplate(templateName: str) -> str:
    LINK = """
    <link name = "%s">
        <visual>
            <origin xyz = "%f %f %f" rpy = "0 0 0"/>
            <geometry>
                <mesh filename = "package://%s/%s" scale = "0.01 0.01 0.01"/>
            </geometry>
        </visual>
        <collision>
            <origin xyz = "%f %f %f" rpy = "0 0 0"/>
            <geometry>
                <mesh filename = "package://%s/%s" scale = "0.01 0.01 0.01"/>
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
        <origin xyz = "%f %f %f" rpy = "0 0 0"/>
        <parent link = "%s"/>
        <child link = "%s"/>
        <axis xyz = "%f %f %f"/>
    </joint>
    """
    FIXED = """
    <joint name = "%s" type = "fixed">
        <origin xyz = "%f %f %f" rpy = "0 0 0"/>
        <parent link = "%s"/>
        <child link = "%s"/>
    </joint>
    """
    REVOLUTE = """
    <joint name = "%s" type = "revolute">
        <origin xyz = "%f %f %f" rpy = "0 0 0"/>
        <parent link = "%s"/>
        <child link = "%s"/>
        <axis xyz = "%f %f %f"/>
        <limit lower = "%f" upper = "%f"/>
    </joint>
    """
    PRISMATIC = """
    <joint name = "%s" type = "prismatic">
        <origin xyz = "%f %f %f" rpy = "0 0 0"/>
        <parent link = "%s"/>
        <child link = "%s"/>
        <axis xyz = "%f %f %f"/>
        <limit lower = "%f" upper = "%f"/>
    </joint>
    """
    return locals()[templateName.upper()]


def fillLinkTemplate(link: adsk.fusion.Occurrence, robotName: str) -> str:
    link_origin = link.getPhysicalProperties().centerOfMass
    inertiaSuccess, xx, yy, zz, xy, yz, xz = link.getPhysicalProperties().getXYZMomentsOfInertia()
    if not inertiaSuccess:
        raise ValueError('Failed to calculate moments of inertia')
    cm_to_m = 0.01
    kgcm2_to_kgm2 = 1e-6
    parsed_name = formatName(link.name)
    mesh_name = 'meshes/' + parsed_name + '.stl'
    linkTemplate = getTemplate('link')
    return linkTemplate % (parsed_name,
                           link_origin.x * cm_to_m,
                           link_origin.y * cm_to_m,
                           link_origin.z * cm_to_m,
                           robotName,
                           mesh_name,
                           link_origin.x * cm_to_m,
                           link_origin.y * cm_to_m,
                           link_origin.z * cm_to_m,
                           robotName,
                           mesh_name,
                           link_origin.x * cm_to_m,
                           link_origin.y * cm_to_m,
                           link_origin.z * cm_to_m,
                           link.getPhysicalProperties().mass,
                           xx * kgcm2_to_kgm2,
                           xy * kgcm2_to_kgm2,
                           xz * kgcm2_to_kgm2,
                           yy * kgcm2_to_kgm2,
                           yz * kgcm2_to_kgm2,
                           zz * kgcm2_to_kgm2,)


def fillJointTemplate(joint: adsk.fusion.Joint, jointType: int, asBuilt: bool) -> str:
    jointDict = {0: 'fixed', 1: 'revolute', 2: 'prismatic', 3: 'continuous'}
    jointTypeStr = jointDict[jointType]
    jointTemplate = getTemplate(jointTypeStr)
    childLink = formatName(joint.occurrenceOne.name)
    if childLink == 'base_link':
        raise ValueError(
            'base_link cannot be a child link. Reverse the selection of the joint.')
    parentLink = formatName(joint.occurrenceTwo.name)
    cm_to_m = 0.01
    if asBuilt:
        joint_origin = joint.geometry
    else:
        joint_origin = joint.geometryOrOriginOne
    if jointTypeStr == 'continuous':
        joint_axis = joint.jointMotion.rotationAxisVector
        return jointTemplate % (joint.name,
                                joint_origin.origin.x * cm_to_m,
                                joint_origin.origin.y * cm_to_m,
                                joint_origin.origin.z * cm_to_m,
                                parentLink,
                                childLink,
                                joint_axis.x,
                                joint_axis.y,
                                joint_axis.z)
    elif jointTypeStr == 'fixed':
        return jointTemplate % (joint.name,
                                joint_origin.origin.x * cm_to_m,
                                joint_origin.origin.y * cm_to_m,
                                joint_origin.origin.z * cm_to_m,
                                parentLink,
                                childLink)
    elif jointTypeStr == 'revolute':
        joint_axis = joint.jointMotion.rotationAxisVector
        joint_limits = joint.jointMotion.rotationLimits
        return jointTemplate % (joint.name,
                                joint_origin.origin.x * cm_to_m,
                                joint_origin.origin.y * cm_to_m,
                                joint_origin.origin.z * cm_to_m,
                                parentLink,
                                childLink,
                                joint_axis.x,
                                joint_axis.y,
                                joint_axis.z,
                                joint_limits.minimumValue,
                                joint_limits.maximumValue)
    elif jointTypeStr == 'prismatic':
        joint_axis = joint.jointMotion.slideDirectionVector
        joint_limits = joint.jointMotion.slideLimits
        return jointTemplate % (joint.name,
                                joint_origin.origin.x * cm_to_m,
                                joint_origin.origin.y * cm_to_m,
                                joint_origin.origin.z * cm_to_m,
                                parentLink,
                                childLink,
                                joint_axis.x,
                                joint_axis.y,
                                joint_axis.z,
                                joint_limits.minimumValue,
                                joint_limits.maximumValue)
    else:
        raise ValueError('Invalid joint type')
