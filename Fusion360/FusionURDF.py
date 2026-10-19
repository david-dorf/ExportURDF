"""Export a Fusion 360 assembly to URDF."""

import math
import os
import re
import traceback
from collections import deque
from xml.sax.saxutils import quoteattr

import adsk.core
import adsk.fusion

# Fusion API internal units are cm, kg and radians.
CM_TO_M = 0.01
KGCM2_TO_KGM2 = 1e-4
# Fusion writes STL files in millimetres.
MESH_SCALE = 0.001
MESH_URI = "../meshes/{}"

# URDF requires effort and velocity on every limited joint; Fusion has neither.
DEFAULT_EFFORT = 100.0
DEFAULT_VELOCITY = 1.0
# Used when a slider has no limit set on one side (metres).
DEFAULT_SLIDE_LIMIT = 1.0
# Used when a revolute joint has a limit set on one side only (radians).
DEFAULT_ROTATION_LIMIT = math.pi
# Mass and inertia given to links that have no bodies, so simulators accept them.
MIN_MASS = 1e-6
MIN_INERTIA = 1e-9

JointTypes = adsk.fusion.JointTypes


class Link:
    def __init__(self, name: str, occurrences: list):
        self.name = name
        self.occurrences = occurrences
        # World position of the link frame in cm.
        self.frame = (0.0, 0.0, 0.0)


class Edge:
    """A Fusion joint between two links."""

    def __init__(self, joint, name: str, child: Link, parent: Link, origin: tuple):
        self.joint = joint
        self.name = name
        # occurrenceOne is the moving side of a Fusion joint, so it is the
        # natural child. The tree search may still attach it the other way.
        self.child = child
        self.parent = parent
        self.origin = origin


class FusionURDF:
    def __init__(self):
        self.ui = None
        self.rootComp = None
        self.exporter = None
        self.warnings = []
        self.usedNames = set()

    def process_URDF(self):
        try:
            app = adsk.core.Application.get()
            self.ui = app.userInterface
            design = adsk.fusion.Design.cast(app.activeProduct)
            if not design:
                self.ui.messageBox("No active Fusion design. Exiting...")
                return
            self.exporter = design.exportManager
            self.rootComp = design.rootComponent

            links = self.buildLinks()
            if not links:
                self.ui.messageBox("No visible components found. Exiting...")
                return
            baseLink = self.findBaseLink(links)
            if baseLink is None:
                self.ui.messageBox(
                    "No component named base_link and no grounded component found. "
                    "Please name or ground the base component. Exiting..."
                )
                return

            edges = self.buildEdges(links, baseLink)
            if not edges and len(links) > 1:
                self.ui.messageBox("No joints found. Exiting...")
                return

            folderPath = self.chooseFolder()
            if folderPath is None:
                return
            robotName = self.formatName(self.rootComp.name)
            robotPath = os.path.join(folderPath, robotName)
            if os.path.exists(robotPath):
                result = self.ui.messageBox(
                    f"Folder {robotPath} already exists. Overwrite its contents?",
                    "Warning",
                    adsk.core.MessageBoxButtonTypes.YesNoButtonType,
                    adsk.core.MessageBoxIconTypes.WarningIconType,
                )
                if result != adsk.core.DialogResults.DialogYes:
                    return
            os.makedirs(os.path.join(robotPath, "urdf"), exist_ok=True)
            os.makedirs(os.path.join(robotPath, "meshes"), exist_ok=True)

            jointXml, extraLinkXml, treeLinks = self.buildTree(baseLink, edges)
            for link in links:
                if link not in treeLinks:
                    self.warnings.append(
                        f"Link {link.name} is not connected to base_link by any joint and was "
                        "left out."
                    )

            linkXml = [self.linkXml(link, robotPath) for link in treeLinks]

            urdfPath = os.path.join(robotPath, "urdf", robotName + ".urdf")
            with open(urdfPath, "w") as urdfFile:
                urdfFile.write('<?xml version="1.0"?>\n')
                urdfFile.write(f"<robot name={quoteattr(robotName)}>\n")
                urdfFile.write("".join(linkXml + extraLinkXml + jointXml))
                urdfFile.write("</robot>\n")

            message = "Exported URDF to " + urdfPath
            if self.warnings:
                message += "\n\nWarnings:\n- " + "\n- ".join(self.warnings)
            self.ui.messageBox(message)

        except:
            if self.ui:
                self.ui.messageBox(f"Failed:\n{traceback.format_exc()}")

    def buildLinks(self) -> list:
        """Create one link per top-level occurrence, merging rigid groups."""
        occurrences = {
            occ.name: occ for occ in self.rootComp.occurrences if occ.isLightBulbOn
        }

        # Union-find over occurrence names so overlapping rigid groups merge.
        groupOf = {name: name for name in occurrences}

        def find(name):
            while groupOf[name] != name:
                groupOf[name] = groupOf[groupOf[name]]
                name = groupOf[name]
            return name

        groupNames = {}
        for group in self.rootComp.rigidGroups:
            if group.isSuppressed:
                continue
            members = {self.topLevelName(occ) for occ in group.occurrences}
            members = [name for name in members if name in occurrences]
            for name in members[1:]:
                groupOf[find(name)] = find(members[0])
            if len(members) > 1:
                groupNames.setdefault(find(members[0]), group.name)

        merged = {}
        for name in occurrences:
            merged.setdefault(find(name), []).append(occurrences[name])

        # Rigid group names may have been recorded against a root that was
        # later merged into another group.
        rootGroupNames = {}
        for root, groupName in groupNames.items():
            rootGroupNames.setdefault(find(root), groupName)

        links = []
        for root, members in merged.items():
            if any(self.isBaseOccurrence(occ) for occ in members):
                name = "base_link"
            elif len(members) > 1:
                name = rootGroupNames.get(root, members[0].name)
            else:
                name = members[0].name
            links.append(Link(self.uniqueName(name), members))
        return links

    @staticmethod
    def isBaseOccurrence(occ) -> bool:
        return "base_link" in occ.name or "base_link" in occ.component.name

    def findBaseLink(self, links: list):
        for link in links:
            if link.name == "base_link":
                return link
        for link in links:
            if any(occ.isGrounded for occ in link.occurrences):
                self.warnings.append(
                    f"No component named base_link; using grounded link {link.name}."
                )
                return link
        return None

    def linkXml(self, link: Link, robotPath: str) -> str:
        fx, fy, fz = link.frame
        visualOrigin = self.originXml(-fx, -fy, -fz)

        parts = [f"  <link name={quoteattr(link.name)}>\n"]
        meshOccurrences = [occ for occ in link.occurrences if self.hasBodies(occ)]
        for occ in meshOccurrences:
            baseName = (
                link.name
                if len(meshOccurrences) == 1
                else f"{link.name}_{self.formatName(occ.name)}"
            )
            meshName = baseName.lower() + ".stl"
            if not self.exportMesh(occ, os.path.join(robotPath, "meshes", meshName)):
                continue
            geometry = (
                "      <geometry>\n"
                '        <mesh filename={} scale="{s} {s} {s}"/>\n'
                "      </geometry>\n"
            ).format(quoteattr(MESH_URI.format(meshName)), s=self.num(MESH_SCALE))
            r, g, b, a = self.occurrenceColor(occ)
            parts.append(
                "    <visual>\n"
                + visualOrigin
                + geometry
                + "      <material name={}>\n"
                '        <color rgba="{} {} {} {}"/>\n'
                "      </material>\n"
                "    </visual>\n".format(
                    quoteattr(baseName.lower() + "_material"),
                    self.num(r),
                    self.num(g),
                    self.num(b),
                    self.num(a),
                )
            )
            parts.append(
                "    <collision>\n" + visualOrigin + geometry + "    </collision>\n"
            )

        mass, com, inertia = self.massProperties(link.occurrences)
        parts.append(
            self.inertialXml(mass, [c - f for c, f in zip(com, link.frame)], inertia)
        )
        parts.append("  </link>\n")
        return "".join(parts)

    @staticmethod
    def hasBodies(occ) -> bool:
        return occ.bRepBodies.count > 0 or occ.childOccurrences.count > 0

    def exportMesh(self, occ, meshPath: str) -> bool:
        try:
            options = self.exporter.createSTLExportOptions(occ, meshPath)
            options.sendToPrintUtility = False
            options.isBinaryFormat = True
            options.meshRefinement = (
                adsk.fusion.MeshRefinementSettings.MeshRefinementLow
            )
            self.exporter.execute(options)
            return True
        except:
            self.warnings.append(f"Could not export a mesh for {occ.name}.")
            return False

    @staticmethod
    def massProperties(occurrences: list):
        """Combined mass (kg), centre of mass (cm, world) and inertia about the
        centre of mass (kg*cm^2, world axes) as [xx, yy, zz, xy, yz, xz]."""
        accuracy = adsk.fusion.CalculationAccuracy.HighCalculationAccuracy
        mass = 0.0
        moment = [0.0, 0.0, 0.0]
        # Inertia about the world origin is additive across occurrences.
        inertia = [0.0] * 6
        for occ in occurrences:
            props = occ.getPhysicalProperties(accuracy)
            if props.mass <= 0:
                continue
            mass += props.mass
            com = props.centerOfMass
            moment = [
                moment[0] + props.mass * com.x,
                moment[1] + props.mass * com.y,
                moment[2] + props.mass * com.z,
            ]
            _, xx, yy, zz, xy, yz, xz = props.getXYZMomentsOfInertia()
            inertia = [a + b for a, b in zip(inertia, [xx, yy, zz, xy, yz, xz])]
        if mass <= 0:
            return 0.0, (0.0, 0.0, 0.0), [0.0] * 6
        x, y, z = (m / mass for m in moment)
        # Parallel axis theorem, from the world origin to the centre of mass.
        shift = [y * y + z * z, x * x + z * z, x * x + y * y, -x * y, -y * z, -x * z]
        inertia = [i - mass * s for i, s in zip(inertia, shift)]
        return mass, (x, y, z), inertia

    def inertialXml(self, mass: float, offset, inertia) -> str:
        if mass <= 0:
            mass, offset = MIN_MASS, (0.0, 0.0, 0.0)
            inertia = [MIN_INERTIA, MIN_INERTIA, MIN_INERTIA, 0.0, 0.0, 0.0]
        else:
            inertia = [i * KGCM2_TO_KGM2 for i in inertia]
        xx, yy, zz, xy, yz, xz = inertia
        return (
            "    <inertial>\n" + self.originXml(*offset) + '      <mass value="{}"/>\n'
            '      <inertia ixx="{}" ixy="{}" ixz="{}" iyy="{}" iyz="{}" izz="{}"/>\n'
            "    </inertial>\n"
        ).format(
            self.num(mass),
            self.num(xx),
            self.num(xy),
            self.num(xz),
            self.num(yy),
            self.num(yz),
            self.num(zz),
        )

    @staticmethod
    def occurrenceColor(occ) -> tuple:
        """Best-effort RGBA (0-1) from the first body's appearance."""
        try:
            bodies = occ.bRepBodies
            appearance = bodies.item(0).appearance if bodies.count else None
            if appearance:
                for prop in appearance.appearanceProperties:
                    colorProp = adsk.core.ColorProperty.cast(prop)
                    if colorProp and colorProp.value:
                        color = colorProp.value
                        return (
                            color.red / 255,
                            color.green / 255,
                            color.blue / 255,
                            color.opacity / 255,
                        )
        except:
            pass
        return 0.7, 0.7, 0.7, 1.0

    def buildEdges(self, links: list, baseLink: Link) -> list:
        linkOf = {occ.name: link for link in links for occ in link.occurrences}

        def linkFor(occ):
            # A joint to the root component itself is a joint to the world,
            # which the URDF represents with base_link.
            if occ is None:
                return baseLink
            return linkOf.get(self.topLevelName(occ))

        candidates = [(joint, False) for joint in self.rootComp.joints]
        candidates += [(joint, True) for joint in self.rootComp.asBuiltJoints]

        edges = []
        for joint, asBuilt in candidates:
            if joint.isSuppressed:
                continue
            child = linkFor(joint.occurrenceOne)
            parent = linkFor(joint.occurrenceTwo)
            if child is None or parent is None:
                self.warnings.append(
                    f"Joint {joint.name} connects a hidden component and was skipped."
                )
                continue
            if child is parent:
                self.warnings.append(
                    f"Joint {joint.name} is inside link {child.name} (rigid group) and was skipped."
                )
                continue
            edges.append(
                Edge(
                    joint,
                    self.uniqueName(joint.name),
                    child,
                    parent,
                    self.jointOrigin(joint, asBuilt),
                )
            )
        return edges

    @staticmethod
    def jointOrigin(joint, asBuilt: bool) -> tuple:
        geometry = joint.geometry if asBuilt else joint.geometryOrOriginOne
        if geometry is None:
            # Rigid as-built joints have no geometry; the child's origin will do.
            translation = joint.occurrenceOne.transform2.translation
            return translation.x, translation.y, translation.z
        point = getattr(geometry, "origin", None)
        if point is None:
            # A JointOrigin wraps its JointGeometry.
            point = geometry.geometry.origin
        return point.x, point.y, point.z

    def buildTree(self, baseLink: Link, edges: list):
        """Breadth-first search from base_link, assigning parent/child roles
        and link frames. Returns joint XML, XML for helper links, and the
        links reached, in tree order."""
        adjacent = {}
        for edge in edges:
            adjacent.setdefault(id(edge.child), []).append(edge)
            adjacent.setdefault(id(edge.parent), []).append(edge)

        jointXml, extraLinkXml = [], []
        visited = [baseLink]
        used = set()
        queue = deque([baseLink])
        while queue:
            parent = queue.popleft()
            for edge in adjacent.get(id(parent), []):
                if id(edge) in used:
                    continue
                used.add(id(edge))
                reversed_ = edge.child is parent
                child = edge.parent if reversed_ else edge.child
                if child in visited:
                    self.warnings.append(
                        f"Joint {edge.name} closes a kinematic loop, which URDF cannot represent, "
                        "and was skipped."
                    )
                    continue
                if (
                    reversed_
                    and edge.joint.jointMotion.jointType != JointTypes.RigidJointType
                ):
                    self.warnings.append(
                        f"Joint {edge.name} was selected child-first; its direction was reversed "
                        "so that it points away from base_link."
                    )
                child.frame = edge.origin
                visited.append(child)
                queue.append(child)
                joints, extraLinks = self.jointXml(edge, parent, child, reversed_)
                jointXml += joints
                extraLinkXml += extraLinks
        return jointXml, extraLinkXml, visited

    def jointXml(self, edge: Edge, parent: Link, child: Link, reversed_: bool):
        """URDF joint elements for one Fusion joint. Joints with two degrees of
        freedom are split in two through a massless helper link."""
        motion = edge.joint.jointMotion
        jointType = motion.jointType
        origin = [(c - p) for c, p in zip(child.frame, parent.frame)]
        # Reversing parent and child reverses the direction of motion. Negating
        # the axis compensates, so the joint value and limits stay the same.
        sign = -1.0 if reversed_ else 1.0

        if jointType == JointTypes.RigidJointType:
            return [
                self.jointElement(edge.name, "fixed", parent.name, child.name, origin)
            ], []

        if jointType == JointTypes.RevoluteJointType:
            return [
                self.rotationElement(
                    edge.name, motion, parent.name, child.name, origin, sign
                )
            ], []

        if jointType == JointTypes.SliderJointType:
            return [
                self.slideElement(
                    edge.name,
                    motion,
                    motion.slideDirectionVector,
                    parent.name,
                    child.name,
                    origin,
                    sign,
                )
            ], []

        if jointType in (JointTypes.CylindricalJointType, JointTypes.PinSlotJointType):
            if jointType == JointTypes.CylindricalJointType:
                direction = motion.rotationAxisVector
            else:
                direction = motion.slideDirectionVector
            helper = self.uniqueName(edge.name + "_slide")
            return (
                [
                    self.slideElement(
                        edge.name + "_slide",
                        motion,
                        direction,
                        parent.name,
                        helper,
                        origin,
                        sign,
                    ),
                    self.rotationElement(
                        edge.name, motion, helper, child.name, (0.0, 0.0, 0.0), sign
                    ),
                ],
                [self.helperLinkXml(helper)],
            )

        if jointType == JointTypes.PlanarJointType:
            return [
                self.jointElement(
                    edge.name,
                    "planar",
                    parent.name,
                    child.name,
                    origin,
                    axis=self.direction(motion.normalDirectionVector, 1.0),
                )
            ], []

        self.warnings.append(
            f"Joint {edge.name} is a ball joint, which URDF cannot represent; it was exported "
            "as fixed."
        )
        return [
            self.jointElement(edge.name, "fixed", parent.name, child.name, origin)
        ], []

    def rotationElement(self, name, motion, parentName, childName, origin, sign) -> str:
        axis = self.direction(motion.rotationAxisVector, sign)
        limits = motion.rotationLimits
        if not (limits.isMinimumValueEnabled or limits.isMaximumValueEnabled):
            return self.jointElement(
                name, "continuous", parentName, childName, origin, axis=axis
            )
        limit = self.limitRange(
            limits, motion.rotationValue, DEFAULT_ROTATION_LIMIT, 1.0
        )
        return self.jointElement(
            name, "revolute", parentName, childName, origin, axis=axis, limit=limit
        )

    def slideElement(
        self, name, motion, direction, parentName, childName, origin, sign
    ) -> str:
        limits = motion.slideLimits
        if not (limits.isMinimumValueEnabled or limits.isMaximumValueEnabled):
            self.warnings.append(
                f"Joint {name} has no slide limits; using +/-{DEFAULT_SLIDE_LIMIT} m."
            )
        limit = self.limitRange(
            limits, motion.slideValue, DEFAULT_SLIDE_LIMIT / CM_TO_M, CM_TO_M
        )
        return self.jointElement(
            name,
            "prismatic",
            parentName,
            childName,
            origin,
            axis=self.direction(direction, sign),
            limit=limit,
        )

    @staticmethod
    def limitRange(limits, current, default, scale) -> tuple:
        """URDF lower/upper limits. URDF measures from the exported pose, while
        Fusion measures from the joint's own zero, so subtract the current value."""
        lower = (
            limits.minimumValue if limits.isMinimumValueEnabled else current - default
        )
        upper = (
            limits.maximumValue if limits.isMaximumValueEnabled else current + default
        )
        return (lower - current) * scale, (upper - current) * scale

    def jointElement(
        self, name, jointType, parentName, childName, origin, axis=None, limit=None
    ) -> str:
        xml = (
            f'  <joint name={quoteattr(name)} type="{jointType}">\n'
            + self.originXml(*origin, indent="    ")
            + f"    <parent link={quoteattr(parentName)}/>\n"
            + f"    <child link={quoteattr(childName)}/>\n"
        )
        if axis is not None:
            xml += '    <axis xyz="{} {} {}"/>\n'.format(*map(self.num, axis))
        if limit is not None:
            xml += f'    <limit lower="{self.num(limit[0])}" upper="{self.num(limit[1])}" effort="{self.num(DEFAULT_EFFORT)}" velocity="{self.num(DEFAULT_VELOCITY)}"/>\n'
        return xml + "  </joint>\n"

    def helperLinkXml(self, name: str) -> str:
        return (
            f"  <link name={quoteattr(name)}>\n"
            + self.inertialXml(0.0, (0.0, 0.0, 0.0), None)
            + "  </link>\n"
        )

    @staticmethod
    def direction(vector, sign: float) -> tuple:
        length = math.sqrt(vector.x**2 + vector.y**2 + vector.z**2) or 1.0
        return tuple(sign * v / length for v in (vector.x, vector.y, vector.z))

    def chooseFolder(self):
        folderOpener = self.ui.createFolderDialog()
        folderOpener.title = "Select folder to save URDF"
        if folderOpener.showDialog() != adsk.core.DialogResults.DialogOK:
            return None
        return folderOpener.folder

    @staticmethod
    def topLevelName(occ) -> str:
        """Name of the top-level occurrence that contains occ."""
        return occ.fullPathName.split("+")[0]

    def originXml(self, x, y, z, indent="      ") -> str:
        """Origin element from a position in cm."""
        return (
            indent
            + f'<origin xyz="{self.num(x * CM_TO_M)} {self.num(y * CM_TO_M)} {self.num(z * CM_TO_M)}" rpy="0 0 0"/>\n'
        )

    @staticmethod
    def num(value: float) -> str:
        value = 0.0 if abs(value) < 1e-12 else value
        return f"{value:.9g}"

    @staticmethod
    def formatName(name: str) -> str:
        if "base_link" in name:
            return "base_link"
        return re.sub(r"[^A-Za-z0-9_]", "_", name)

    def uniqueName(self, name: str) -> str:
        """Formatted name that no other link or joint uses yet."""
        base = self.formatName(name)
        candidate, n = base, 1
        while candidate.lower() in self.usedNames:
            n += 1
            candidate = f"{base}_{n}"
        self.usedNames.add(candidate.lower())
        return candidate


def run(context):
    fusion_urdf_loader = FusionURDF()
    fusion_urdf_loader.process_URDF()
