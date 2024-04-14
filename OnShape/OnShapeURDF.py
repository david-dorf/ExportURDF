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
        xmlHeader = """<?xml version = "1.0" ?>\n"""
        robotHeader = """<robot name = "%s">\n"""
        robotFooter = """\n</robot>"""
        try:
            robotName, folderPath = self.getRobotDetails()
            urdfFile = open(os.path.join(
                folderPath, robotName, 'urdf', robotName + '.urdf'), 'w')
            urdfFile.write(xmlHeader)
            urdfFile.write(robotHeader % robotName)
            partIDs, partNames = self.extractLinks()
            # TODO: Fix the function to export meshes
            self.exportMeshes(folderPath, robotName, partIDs, partNames)
            joints = self.extractJoints()
            urdfFile.write(robotFooter)
        except:
            print('Failed:\n{}'.format(traceback.format_exc()))

    @staticmethod
    def getRobotDetails():
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
        documentID = url.split('documents/')[1].split('/')[0]
        workspaceID = url.split('w/')[1].split('/')[0]
        elementID = url.split('e/')[1]
        return documentID, workspaceID, elementID

    def extractLinks(self) -> list:
        partIDs = []
        partNames = []

        for instance in self.assembly["rootAssembly"]["instances"]:
            if instance["type"] == "Part":
                partIDs.append(instance["partId"])
                partNames.append(self.formatName(instance["name"]))
            elif instance["type"] == "Assembly":
                # TODO: Implement sub-assembly support
                print("Sub-assembly found, not supported yet")
            else:
                print("Unknown instance type: " + instance["type"])

        return partIDs, partNames

    def extractJoints(self) -> list:
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
        rMatrix = np.array(
            [matedCS["xAxis"], matedCS["yAxis"], matedCS["zAxis"]])
        rMatrixT = rMatrix.T
        rotation_axis = np.dot(rMatrixT, [0, 0, 1])
        return rotation_axis

    def exportMeshes(self, folderPath: str, robotName: str, partIDs: list, partNames: list):
        for partID in partIDs:
            print(partID)
            mesh = self.client.part_studio_stl_m(
                self.documentID, self.workspaceID, self.elementID, partID)
            with open(os.path.join(folderPath, robotName, 'meshes',
                                   self.formatName(partNames[partIDs.index(partID)]) + '.stl'), 'wb') as f:
                f.write(mesh)

    def formatName(self, name: str) -> str:
        if 'base_link' in name:
            return 'base_link'
        else:
            return name.translate(str.maketrans(' :()<>', '______'))

    def fillTemplate(self):
        # self.getTemplate()
        # TODO: Implement the function to fill the URDF template using the extracted data
        pass

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


if __name__ == "__main__":
    onshapeURDF = OnShapeURDF()
    onshapeURDF.createURDF()
