from onshape_api.client import Client
from tkinter import filedialog
import traceback
import os
import numpy as np


class OnShapeURDF:
    def __init__(self):
        self.client = Client()
        os.system('clear')  # Clear the terminal
        self.onshapeURL = input(
            "Enter the URL of the OnShape assembly document: ")
        self.documentID, self.workspaceID, self.elementID = self.extractID(
            self.onshapeURL)
        self.assembly = self.client.get_assembly(
            self.documentID, self.workspaceID, self.elementID)
        os.system('clear')  # Clear the terminal

    def createURDF(self):
        """Create the URDF file for the robot."""
        xmlHeader = """<?xml version = "1.0" ?>\n"""
        robotHeader = """<robot name = "%s">\n"""
        robotFooter = """\n</robot>"""
        try:
            robotName, folderPath = self.getFolder()
            urdfFile = open(os.path.join(
                folderPath, robotName, 'urdf', robotName + '.urdf'), 'w')
            urdfFile.write(xmlHeader)
            urdfFile.write(robotHeader % robotName)
            partNameList = self.extractLinks(folderPath, robotName)
            joints = self.extractJoints(partNameList)
            urdfFile.write(robotFooter)
        except:
            print('Failed:\n{}'.format(traceback.format_exc()))

    @staticmethod
    def getFolder():
        """Get the folder path to save the URDF and mesh files."""
        robotName = input("\nEnter the name of the robot: ")
        folderPath = filedialog.askdirectory(
            title='Select the folder to save the URDF files')
        try:
            os.mkdir(os.path.join(folderPath, robotName))
            os.mkdir(os.path.join(folderPath, robotName, 'urdf'))
            os.mkdir(os.path.join(folderPath,
                     robotName, 'meshes'))
        except:
            returnValue = input(
                'Folder already exists. Try to overwrite it? Enter Y or N')
            if returnValue.upper() == 'Y':
                pass
            elif returnValue.upper() == 'N':
                exit()
            else:
                print('Invalid input. Exiting...')
                exit()
        return robotName, folderPath

    def extractID(self, url: str) -> str:
        """Extract the documentID, workspaceID, and elementID from the URL."""
        documentID = url.split('documents/')[1].split('/')[0]
        workspaceID = url.split('w/')[1].split('/')[0]
        elementID = url.split('e/')[1]
        return documentID, workspaceID, elementID

    def extractLinks(self, folderPath: str, robotName: str) -> list[str]:
        """Extract the links from the assembly and save the meshes as STL files."""
        partNameList = []
        for instance in self.assembly["rootAssembly"]["instances"]:
            if instance["type"] == "Part":
                partName = self.formatName(instance["name"])
                partNameList.append(partName)
                stl = self.client.part_studio_stl_m(self.documentID,
                                                    instance["documentMicroversion"],
                                                    instance["elementId"], instance["partId"])
                with open(os.path.join(folderPath, robotName, 'meshes', partName + '.stl'), 'wb') as f:
                    f.write(stl)

            elif instance["type"] == "Assembly":
                print("Sub-assembly found, not supported yet")
            else:
                print("Unknown instance type: " + instance["type"])
        return partNameList

    def extractJoints(self, partNameList: list[str]) -> list[dict]:
        """Extract the joints from the assembly."""
        joints = []
        for feature in self.assembly["rootAssembly"]["features"]:
            if feature["featureType"] == "mate":
                matedCS = feature["featureData"]["matedEntities"][1]["matedCS"]
                joint = {
                    "type": feature["featureData"]["mateType"],
                    "name": self.formatName(feature["featureData"]["name"]),
                    "parent": feature["featureData"]["matedEntities"][0]["matedOccurrence"],
                    "child": feature["featureData"]["matedEntities"][1]["matedOccurrence"],
                    "rotationAxis": self.getRotationAxis(matedCS),
                    "origin": matedCS["origin"],
                }
                joints.append(joint)
        return joints

    def getRotationAxis(self, matedCS: dict) -> np.array:
        """Get the rotation axis of the joint."""
        rMatrix = np.array(
            [matedCS["xAxis"], matedCS["yAxis"], matedCS["zAxis"]])
        rMatrixT = rMatrix.T
        rotation_axis = np.dot(rMatrixT, [0, 0, 1])
        return rotation_axis

    def formatName(self, name: str) -> str:
        """Format the name of the link or joint."""
        if 'base_link' in name:
            return 'base_link'
        else:
            return name.translate(str.maketrans(' :()<>', '______'))

    def fillLinkTemplate(self, linkName: str, origin: np.array,
                         meshPath: str, mass: float, inertia: np.array) -> str:
        """Fill the link template with the given values."""
        return self.getTemplate('link') % (linkName, origin[0], origin[1], origin[2], meshPath,
                                           mass, inertia[0], inertia[1], inertia[2], inertia[3],
                                           inertia[4], inertia[5], inertia[6])

    def fillJointTemplate(self, jointName: str, jointType: str, origin: np.array,
                          parentLink: str, childLink: str, axis: np.array, limits: np.array) -> str:
        """Fill the joint template with the given values."""
        return self.getTemplate(jointType) % (jointName, origin[0], origin[1], origin[2],
                                              parentLink, childLink, axis[0], axis[1], axis[2],
                                              limits[0], limits[1])

    def getTemplate(templateName: str) -> str:
        """Get the URDF XML template based on the joint or link name."""
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


if __name__ == "__main__":
    onshapeURDF = OnShapeURDF()
    onshapeURDF.createURDF()
