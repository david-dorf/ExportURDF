![exporturdfbanner](https://github.com/daviddorf2023/ExportURDF/assets/113081373/35c860a4-1283-4824-84e4-e0c137349353)
# ExportURDF
ExportURDF is a unified URDF converter library for CAD programs, primarily targeting Fusion360, OnShape, and Solidworks. The library aims to provide central, maintainable location for scripts to convert CAD models to URDF files, which are commonly used in robotics and simulation.

## Supported CAD Programs

Currently, the library supports Fusion360 and OnShape is actively in the works. However, work is still fresh to add support for SolidWorks and other CAD programs.

## Roadmap

1. Fusion360 support
2. OnShape support
3. SolidWorks support
4. Unit tests
5. Linting
6. Tutorials/Docs

Other potential features:
- Support for other CAD platforms
- Materials integration
- SDF tools integration
- ROS 2 package generator from URDF

## Current TODOs
1. Tests for roll, pitch, yaw rotation of parts.
2. Subassembly support.
3. Rigid groups in Fusion360.

## Getting Started

To get started with the library, follow these steps:

1. Clone the repository: `git clone https://github.com/daviddorf2023/ExportURDF.git`
2. Follow the instructions in the respective directories for Fusion360 and OnShape to install the required dependencies and run the converter scripts.

## Contributing

Contributions are welcome!

## License

This project is licensed under the [MIT License](LICENSE).
