from onshape_api.client import Client
from tkinter import filedialog
import traceback
import os


class OnShapeURDF:
    def __init__(self):
        self.client = Client()
        self.xmlHeader = """<?xml version = "1.0" ?>\n"""
        self.robotHeader = """<robot name = "%s">\n"""
        self.robotFooter = """\n</robot>"""
        self.onshapeURL = input(
            "Enter the URL of the OnShape assembly document: ")
        self.documentID, self.workspaceID, self.elementID = self.extractID(
            self.onshapeURL)
        self.assembly = self.client.get_assembly(
            self.documentID, self.workspaceID, self.elementID)
        self.robotName = input("Enter the name of the robot: ")
        self.robotLinks = self.assembly['rootAssembly']['occurrences']
        self.robotJoints = self.assembly['rootAssembly']['features'][0]['featureData']['matedEntities']
        print(self.robotName)
        print(self.robotLinks)
        print(self.robotJoints)
        return

    def createURDF(self):
        try:
            folderPath = filedialog.askdirectory()
            try:
                os.mkdir(os.path.join(folderPath, self.robotName))
                os.mkdir(os.path.join(folderPath, self.robotName, 'urdf'))
                os.mkdir(os.path.join(folderPath, self.robotName, 'meshes'))
            except:
                returnValue = input(
                    'Folder already exists. Try to overwrite it? Enter Y or N')
                if returnValue.upper() == 'Y':
                    pass
                elif returnValue.upper() == 'N':
                    return
                else:
                    print('Invalid input. Exiting...')
                    return
            urdfFile = open(os.path.join(
                folderPath, self.robotName, 'urdf', self.robotName + '.urdf'), 'w')
            urdfFile.write(self.xmlHeader)
            urdfFile.write(self.robotHeader % self.robotName)
            self.createURDFLinks()
            self.createURDFJoints()
            urdfFile.write(self.robotFooter)
        except:
            print('Failed:\n{}'.format(traceback.format_exc()))

    def extractID(self, url: str) -> str:
        documentID = url.split('documents/')[1].split('/')[0]
        workspaceID = url.split('w/')[1].split('/')[0]
        elementID = url.split('e/')[1]
        return documentID, workspaceID, elementID

    def createURDFLinks(self):
        pass

    def createURDFJoints(self):
        pass

    def formatName(self, name: str) -> str:
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


if __name__ == "__main__":
    onshapeURDF = OnShapeURDF()
    onshapeURDF.createURDF()
