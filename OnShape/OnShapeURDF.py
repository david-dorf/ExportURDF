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

    def createURDF(self):
        try:
            robotName = self.formatName(input("Enter the name of the robot: "))
            folderPath = filedialog.askdirectory()
            try:
                os.mkdir(os.path.join(folderPath, robotName))
                os.mkdir(os.path.join(folderPath, robotName, 'urdf'))
                os.mkdir(os.path.join(folderPath, robotName, 'meshes'))
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
                folderPath, robotName, 'urdf', robotName + '.urdf'), 'w')
            urdfFile.write(self.xmlHeader)
            urdfFile.write(self.robotHeader % robotName)
            self.createURDFLinks()
            self.createURDFJoints()
            urdfFile.write(self.robotFooter)
        except:
            print('Failed:\n{}'.format(traceback.format_exc()))

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
