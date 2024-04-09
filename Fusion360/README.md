# Fusion360 URDF Converter Script

This repository contains a Fusion360 URDF converter script that converts Fusion360 models into URDF files. The script is written in Python and uses the Fusion360 API to extract the necessary information from the CAD model and generate the URDF files.

## Getting Started
In a Fusion360 environment, you can run the script by following these steps:

1. Open the Fusion360 application.
2. Use `Shift + S` to open the Scripts and Add-ins dialog.
3. Click the green `+` icon to add a new script.
4. Copy the `Fusion360` folder into the script folder. [TODO: Create a setup program to do this automatically for users]
5. Run the `FusionURDF` script by clicking the `Run` button.

## Roadmap
1. Add support for rigid groups and as-built joints.
2. Add additional features such as materials integration and SDF tools integration.
3. Fix any lingering issues with the script.

## Dependencies
The script requires the following dependencies:
Python 3.6+
Fusion360 API

## Contributing
Contributions are welcome! Currently the script does not create a base_link, and does not support rigid groups or as-built joints. There are still likely to be lingering issues with the script, so any help is appreciated.
