# Fusion360 URDF Converter Script

This repository contains a Fusion360 URDF converter script that converts Fusion360 models into URDF files. The script is written in Python and uses the Fusion360 API to extract the necessary information from the CAD model and generate the URDF files.

## Getting Started
Before using the script, make sure your Fusion360 design is set up correctly. The script requires the following:
1. Each link should be a component in the Fusion360 design.
2. At least one link should be designated as the base link. The base link should only have a fixed joint connected to it.
3. The joints should be created using the Fusion360 joint tool. The script currently supports the following joint types:
   - Revolute
   - Fixed
   - Slider
   - The above, but with limits and as-built joints
4. When making a joint, the parent link should be selected first, followed by the child link.

In a Fusion360 environment, you can run the script by following these steps:
1. Open the Fusion360 application.
2. Use `Shift + S` to open the Scripts and Add-ins dialog.
3. Click the green `+` icon to add a new script.
4. Copy the `Fusion360` folder into the script folder. [TODO: Create a setup program to do this automatically for users]
5. Run the `FusionURDF` script by clicking the `Run` button.

## Roadmap
1. Write unit tests for the script to ensure code quality and maintainability.
2. Test the script with various Fusion360 models to identify and fix any issues. 
   Edge cases like closed chains may not be handled correctly yet.
3. Add additional features such as materials, rigid groups, and more joint types.

## Dependencies
The script requires the following dependencies:
Python 3.6+
Fusion360 API

## Contributing
Contributions are welcome! Currently the script does not create a base_link, and does not support rigid groups or as-built joints. There are still likely to be lingering issues with the script, so any help is appreciated.
