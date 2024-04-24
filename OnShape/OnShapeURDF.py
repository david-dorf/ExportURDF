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
        self.urdfFile = None
        self.robotName = None
        self.folderPath = None
        os.system('clear')  # Clear the terminal

    def createURDF(self):
        """Create the URDF file for the robot."""
        xmlHeader = """<?xml version = "1.0" ?>\n"""
        robotHeader = """<robot name = "%s">\n"""
        robotFooter = """\n</robot>"""
        try:
            self.robotName, self.folderPath = self.getFolder()
            self.urdfFile = open(os.path.join(
                self.folderPath, self.robotName, 'urdf', self.robotName + '.urdf'), 'w')
            self.urdfFile.write(xmlHeader)
            self.urdfFile.write(robotHeader % self.robotName)
            linkData = self.extractLinks(self.folderPath, self.robotName)
            jointData = self.extractJoints()
            self.fillLinkTemplate(linkData)
            self.fillJointTemplate(jointData, linkData)
            self.urdfFile.write(robotFooter)
            print('URDF file created successfully.')
        except:
            print('Failed:\n{}'.format(traceback.format_exc()))

    @ staticmethod
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

    def extractLinks(self, folderPath: str, robotName: str) -> list[dict]:
        """Extract the link information from the assembly and save the meshes as STL files."""
        linkDataSet = []
        for instance in self.assembly["rootAssembly"]["instances"]:
            linkData = {}
            if instance["type"] == "Part":
                partName = self.formatName(instance["name"])
                stl = self.client.part_studio_stl_m(self.documentID,
                                                    instance["documentMicroversion"],
                                                    instance["elementId"], instance["partId"])
                with open(os.path.join(folderPath, robotName,
                                       'meshes', partName + '.stl'), 'wb') as f:
                    f.write(stl)
                massProperties = self.client.part_mass_properties(
                    self.documentID, instance["documentMicroversion"],
                    instance["elementId"], instance["partId"])
                os.system('clear')  # Clear the terminal
                mass = massProperties["bodies"][instance["partId"]]["mass"]
                inertia = massProperties["bodies"][instance["partId"]]["inertia"]
                centroid = massProperties["bodies"][instance["partId"]]["centroid"]
                linkData = {
                    "name": partName,
                    "id": instance["id"],
                    "origin": centroid,
                    "meshPath": os.path.join('meshes', partName + '.stl'),
                    "mass": mass,
                    "inertia": inertia
                }
                linkDataSet.append(linkData)
            elif instance["type"] == "Assembly":
                print("Sub-assembly found, not supported yet")
            else:
                print("Unknown instance type: " + instance["type"])
        return linkDataSet

    def extractJoints(self) -> list[dict]:
        """Extract the joint information from the assembly."""
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
                    "limit": None  # TODO: Add limit for revolute and prismatic joints
                }
                joints.append(joint)
        return joints

    def mapPartNames(self, joints: list[dict], links: list[dict]) -> dict:
        """Map the parents and children of the joints to their link names."""
        linkMap = {}
        for joint in joints:
            parent = joint["parent"][0]
            child = joint["child"][0]
            for link in links:
                if parent == link["id"]:
                    linkMap[parent] = link["name"]
                if child == link["id"]:
                    linkMap[child] = link["name"]
        return linkMap

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
            return name.translate(str.maketrans(' :()<>;', '_______'))

    def fillLinkTemplate(self, linkDataSet: list[dict]):
        """Fill the link template with the given values."""
        for linkData in linkDataSet:
            linkName = linkData["name"]
            origin = linkData["origin"]
            meshPath = linkData["meshPath"]
            mass = linkData["mass"][0]
            inertia = linkData["inertia"]
            self.urdfFile.write(self.getTemplate('link') % (linkName, origin[0], origin[1],
                                                            origin[2], self.robotName, meshPath,
                                                            origin[0], origin[1], origin[2],
                                                            self.robotName, meshPath, origin[0],
                                                            origin[1], origin[2], mass, inertia[0],
                                                            inertia[1], inertia[2], inertia[3],
                                                            inertia[4], inertia[5]))

    def fillJointTemplate(self, jointData: list[dict], linkData: list[dict]):
        """Fill the joint template with the given values."""
        partMap = self.mapPartNames(jointData, linkData)
        for joint in jointData:
            jointName = joint["name"]
            jointType = joint["type"].lower()
            origin = joint["origin"]
            # OnShape gives the parent and child in the opposite order, so we need to swap them
            parent = partMap[joint["child"][0]]
            child = partMap[joint["parent"][0]]
            if child == "base_link":
                parent, child = child, parent  # Always make base_link the parent
                print("Base link was assigned as child, was this intentional?")
            rotationAxis = joint["rotationAxis"]
            if jointType == "fastened":
                self.urdfFile.write(self.getTemplate(jointType) % (jointName, origin[0],
                                                                   origin[1], origin[2],
                                                                   parent, child))
            elif jointType == "revolute":
                limit = joint["limit"]
                if limit:
                    self.urdfFile.write(self.getTemplate(jointType) %
                                        (jointName, origin[0], origin[1], origin[2], parent, child,
                                         rotationAxis[0], rotationAxis[1], rotationAxis[2], limit[0],
                                         limit[1]))
                else:
                    self.urdfFile.write(self.getTemplate("continuous") % (jointName, origin[0],
                                                                          origin[1], origin[2],
                                                                          parent, child,
                                                                          rotationAxis[0],
                                                                          rotationAxis[1],
                                                                          rotationAxis[2]))
            elif jointType == "slider":
                limit = joint["limit"]
                self.urdfFile.write(self.getTemplate(jointType) % (jointName, origin[0], origin[1],
                                                                   origin[2], parent, child,
                                                                   rotationAxis[0], rotationAxis[1],
                                                                   rotationAxis[2], limit[0], limit[1]))
            else:
                raise ValueError("Unknown joint type: " + jointType)

    def getTemplate(self, templateName: str) -> str:
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
        FASTENED = """
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
        SLIDER = """
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
