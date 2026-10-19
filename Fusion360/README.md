# Fusion360 URDF Converter Script

This repository contains a Fusion360 URDF converter script that converts Fusion360 models into URDF files. The script is written in Python and uses the Fusion360 API to extract the necessary information from the CAD model and generate the URDF files.

## Getting Started
Before using the script, make sure your Fusion360 design is set up correctly:
1. Each link should be a top-level component (occurrence) in the design. Subassemblies are exported as a single link.
2. Name the base component `base_link`. If none are named that, the grounded component is used as the base.
3. Connect the components with Fusion360 joints or as-built joints. The joints may form a tree (several children per parent). Selection order does not matter: each joint is oriented away from `base_link` automatically.
4. To export several components as one link, put them in a **rigid group**. Their meshes appear as separate visuals in the same link, and their mass properties are combined.
5. Hidden components, suppressed joints and suppressed rigid groups are left out.

Supported joint types:

| Fusion360 joint | URDF |
| --- | --- |
| Rigid | `fixed` |
| Revolute | `revolute` if it has limits, otherwise `continuous` |
| Slider | `prismatic` (+/-1 m if it has no limits) |
| Cylindrical, Pin-slot | `prismatic` + `revolute`/`continuous` through a massless helper link |
| Planar | `planar` |
| Ball | `fixed`, with a warning |

Joint limits are measured from the pose the design is in when you export. Any skipped joints, loops or disconnected components are listed in a warning when the export finishes. The design is not modified.

In a Fusion360 environment, you can run the script by following these steps:
1. Open the Fusion360 application.
2. Use `Shift + S` to open the Scripts and Add-ins dialog.
3. Click the green `+` icon to open the Fusion360 script folder.
4. Copy the `Fusion360` folder from inside this repository into the Fusion360 script folder.
5. Run the `FusionURDF` script by clicking the `Run` button.

## Roadmap
1. Write unit tests for the script to ensure code quality and maintainability.
2. Test the script with various Fusion360 models to identify and fix any issues.
   Closed kinematic chains cannot be represented in URDF; the joint that closes a loop is skipped.
3. Export Fusion360 physical materials, and joint effort/velocity limits (currently fixed defaults).
4. Export joints inside subassemblies as separate links.

## Dependencies
The script requires the following dependencies:

Python 3.6+

Fusion360 API
