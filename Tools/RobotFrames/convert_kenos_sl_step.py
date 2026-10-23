#!/usr/bin/env python3
"""Tessellate the owner-supplied KSL assembly into the ROS tool's Unity frame.

Requires Python 3.12 and cadquery-ocp==8.0.1.1.0 with its declared dependencies.
Packages may be unpacked into Temp/KenosSlCad/packages; no system install is used.

Example (from the Unity project root):
  python Tools/RobotFrames/convert_kenos_sl_step.py --input <owner-supplied.stp>

The .bytes output is authoritative: KSL1, little-endian int32 vertex/index counts,
float32 XYZ vertices in Unity RUF metres, and int32 triangle indices. OBJ is an
inspection copy because Unity model importers can apply their own axis conversion.
The mesh is local to kenos_sl_base. It does not alter ROS joints, TCP or mount.
"""

from __future__ import annotations

import argparse
import hashlib
import importlib.metadata
import json
import math
from pathlib import Path
import struct
import sys


SOURCE_SHA256 = "cfb722d77f26901b8299399c5c8beb66e5cf946b61a0637f6d34615044553731"
PROJECT = Path(__file__).resolve().parents[2]
DEFAULT_PACKAGES = PROJECT / "Temp/KenosSlCad/packages"
DEFAULT_OUTPUT = PROJECT / "Assets/Resources/RobotModels/KenosSLVisual"
DEFAULT_OBJ = PROJECT / "Assets/Models/KenosSL/KenosSLVisual.obj"


def root_mm_to_unity(point):
    """STEP root -> KSL mount-centred ROS FLU -> Unity RUF, in metres."""
    x, y, z = point
    return (-x / 1000.0, (31.75 - z) / 1000.0, (y + 34.0) / 1000.0)


def bounds(points):
    return {
        "min": [min(p[i] for p in points) for i in range(3)],
        "max": [max(p[i] for p in points) for i in range(3)],
    }


def cross(a, b):
    return (a[1] * b[2] - a[2] * b[1],
            a[2] * b[0] - a[0] * b[2],
            a[0] * b[1] - a[1] * b[0])


def check_reference_frame():
    for source, expected in [((0, -34, 31.75), (0, 0, 0)),
                             ((0, 0, -58), (0, .08975, .034))]:
        actual = root_mm_to_unity(source)
        if any(abs(a - b) > 1e-12 for a, b in zip(actual, expected)):
            raise ValueError("The CAD mounting/contact frame conversion changed")


def load_mesh(source, packages, linear_deflection, angular_deflection):
    if packages.is_dir():
        sys.path.insert(0, str(packages))
    import OCP
    from OCP.BRep import BRep_Tool
    from OCP.BRepBndLib import BRepBndLib
    from OCP.BRepCheck import BRepCheck_Analyzer
    from OCP.BRepMesh import BRepMesh_IncrementalMesh
    from OCP.Bnd import Bnd_Box
    from OCP.IFSelect import IFSelect_RetDone
    from OCP.STEPControl import STEPControl_Reader
    from OCP.TopAbs import TopAbs_FACE, TopAbs_REVERSED, TopAbs_SOLID
    from OCP.TopExp import TopExp_Explorer
    from OCP.TopLoc import TopLoc_Location
    from OCP.TopoDS import TopoDS

    reader = STEPControl_Reader()
    if reader.ReadFile(str(source)) != IFSelect_RetDone:
        raise ValueError("Open CASCADE could not read the STEP file")
    if reader.NbRootsForTransfer() != 1 or reader.TransferRoots() != 1:
        raise ValueError("Expected the complete single-root STEP assembly")
    shape = reader.OneShape()
    if shape.IsNull() or not BRepCheck_Analyzer(shape).IsValid():
        raise ValueError("The STEP assembly is empty or is not a valid BRep")

    # Serial meshing keeps face/triangle order reproducible between reruns.
    mesher = BRepMesh_IncrementalMesh(shape, linear_deflection, False,
                                    angular_deflection, False)
    if not mesher.IsDone():
        raise ValueError("Open CASCADE tessellation failed")

    source_bounds = Bnd_Box()
    BRepBndLib.AddOptimal_s(shape, source_bounds, False, False)
    source_limits = {"min": list(source_bounds.CornerMin().Coord()),
                     "max": list(source_bounds.CornerMax().Coord())}
    vertices, indices, solids = [], [], []
    face_count, omitted_degenerate = 0, 0
    solid_explorer = TopExp_Explorer(shape, TopAbs_SOLID)
    while solid_explorer.More():
        solid = solid_explorer.Current()
        solid_first_vertex, solid_first_index = len(vertices), len(indices)
        face_explorer = TopExp_Explorer(solid, TopAbs_FACE)
        while face_explorer.More():
            face = TopoDS.Face(face_explorer.Current())
            location = TopLoc_Location()
            triangulation = BRep_Tool.Triangulation_s(face, location)
            if triangulation is None or triangulation.NbTriangles() == 0:
                raise ValueError("A CAD face has no triangulation")
            offset = len(vertices)
            transform = location.Transformation()
            # Keep face seams: RecalculateNormals should preserve sharp CAD edges.
            for node in range(1, triangulation.NbNodes() + 1):
                point = triangulation.Node(node).Transformed(transform).Coord()
                vertices.append(struct.unpack("<3f", struct.pack("<3f", *root_mm_to_unity(point))))
            for triangle in range(1, triangulation.NbTriangles() + 1):
                a, b, c = triangulation.Triangle(triangle).Get()
                if face.Orientation() == TopAbs_REVERSED:
                    b, c = c, b
                # ROS FLU -> Unity RUF reflects handedness: reverse winding once.
                triangle_indices = (offset + a - 1, offset + c - 1, offset + b - 1)
                pa, pb, pc = (vertices[j] for j in triangle_indices)
                normal = cross(tuple(pb[j] - pa[j] for j in range(3)),
                               tuple(pc[j] - pa[j] for j in range(3)))
                # CAD poles can have zero-area triangles. Float32 may also merge
                # sub-micrometre edges; they have no visible or physical area.
                if sum(x * x for x in normal) < 1e-24:
                    omitted_degenerate += 1
                else:
                    indices.extend(triangle_indices)
            face_count += 1
            face_explorer.Next()
        solid_vertices = vertices[solid_first_vertex:]
        names = ("holder", "manifold", "plug_1", "plug_2", "plug_3", "plug_4", "foam", "ejector", "air_connection")
        solids.append({"index": len(solids), "name": names[len(solids)], "boundsUnityMetres": bounds(solid_vertices),
                       "firstVertex": solid_first_vertex,
                       "vertexCount": len(vertices) - solid_first_vertex,
                       "firstIndex": solid_first_index,
                       "indexCount": len(indices) - solid_first_index})
        solid_explorer.Next()
    if len(solids) != 9 or face_count != 794:
        raise ValueError(f"Unexpected assembly topology: {len(solids)} solids/{face_count} faces")
    foam = solids[6]["boundsUnityMetres"]
    for i, (expected_min, expected_max) in enumerate(((-.06, .06), (.06575, .08975), (-.086, .154))):
        if abs(foam["min"][i] - expected_min) > 1e-7 or abs(foam["max"][i] - expected_max) > 1e-7:
            raise ValueError("The source foam envelope no longer matches its mounting/contact frame")
    return vertices, indices, solids, source_limits, str(OCP.__version__), face_count, omitted_degenerate


def validate_mesh(vertices, indices):
    if not vertices or not indices or len(indices) % 3:
        raise ValueError("Mesh is empty or has incomplete triangles")
    if len(indices) // 3 > 100000:
        raise ValueError("Detailed mesh exceeds the 100,000 triangle runtime budget")
    if any(not math.isfinite(c) for vertex in vertices for c in vertex):
        raise ValueError("Mesh contains a non-finite vertex")
    if min(indices) < 0 or max(indices) >= len(vertices):
        raise ValueError("Mesh contains an invalid vertex index")
    volume = 0.0
    degenerate = 0
    for i in range(0, len(indices), 3):
        a, b, c = (vertices[indices[i + j]] for j in range(3))
        ab, ac = tuple(b[j] - a[j] for j in range(3)), tuple(c[j] - a[j] for j in range(3))
        normal = cross(ab, ac)
        if sum(x * x for x in normal) < 1e-24:
            degenerate += 1
        bc = cross(b, c)
        volume += sum(a[j] * bc[j] for j in range(3)) / 6.0
    if degenerate:
        raise ValueError(f"Mesh contains {degenerate} degenerate triangles")
    if volume <= 0:
        raise ValueError("Mesh winding has a non-positive signed volume")
    limits = bounds(vertices)
    expected_min, expected_max = (-.065, -.002, -.091), (.065, .08975, .159)
    # The two extreme holder dimensions/contact plane independently detect mm/m
    # mistakes, missing assembly locations, and accidental importer-style mirrors.
    for i in (0, 2):
        if abs(limits["min"][i] - expected_min[i]) > .0005 or abs(limits["max"][i] - expected_max[i]) > .0005:
            raise ValueError("Mesh envelope does not match the mounted KSL assembly")
    if abs(limits["max"][1] - expected_max[1]) > .00001:
        raise ValueError("Mesh foam contact surface does not match its reference frame")
    return volume, limits


def write_outputs(output, obj_path, vertices, indices):
    output.parent.mkdir(parents=True, exist_ok=True)
    obj_path.parent.mkdir(parents=True, exist_ok=True)
    binary = bytearray(struct.pack("<4sii", b"KSL1", len(vertices), len(indices)))
    for vertex in vertices:
        binary.extend(struct.pack("<3f", *vertex))
    for index in indices:
        binary.extend(struct.pack("<i", index))
    output.with_suffix(".bytes").write_bytes(binary)
    with obj_path.open("w", encoding="utf-8", newline="\n") as stream:
        stream.write("# KSL assembly; kenos_sl_base local Unity RUF metres\n")
        stream.write("# Runtime uses KSL1 .bytes to avoid model importer axis conversion.\n")
        stream.write("o KenosSLVisual\n")
        for vertex in vertices:
            stream.write("v " + " ".join(f"{c:.9g}" for c in vertex) + "\n")
        for i in range(0, len(indices), 3):
            stream.write("f " + " ".join(str(j + 1) for j in indices[i:i + 3]) + "\n")
    # Verify the actual binary written, including float32 envelope and length.
    decoded = output.with_suffix(".bytes").read_bytes()
    magic, nvertices, nindices = struct.unpack_from("<4sii", decoded)
    if (magic, nvertices, nindices) != (b"KSL1", len(vertices), len(indices)):
        raise ValueError("Binary mesh header failed round-trip validation")
    if len(decoded) != 12 + nvertices * 12 + nindices * 4:
        raise ValueError("Binary mesh byte length is invalid")
    packed_vertices = list(struct.iter_unpack("<3f", decoded[12:12 + nvertices * 12]))
    packed_indices = [i[0] for i in struct.iter_unpack("<i", decoded[12 + nvertices * 12:])]
    validate_mesh(packed_vertices, packed_indices)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", type=Path, required=True)
    parser.add_argument("--ocp-path", type=Path, default=DEFAULT_PACKAGES)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--obj-output", type=Path, default=DEFAULT_OBJ)
    parser.add_argument("--linear-deflection-mm", type=float, default=.3)
    parser.add_argument("--angular-deflection-radians", type=float, default=.3)
    args = parser.parse_args()
    if not (0 < args.linear_deflection_mm <= 1) or not (0 < args.angular_deflection_radians <= 1):
        parser.error("CAD deflections must be positive and no greater than 1 mm/radian")
    source = args.input.resolve()
    source_hash = hashlib.sha256(source.read_bytes()).hexdigest()
    if source_hash != SOURCE_SHA256:
        raise ValueError("Source hash differs from the owner-supplied KSL STEP; revalidate its mounting frame before conversion")
    check_reference_frame()
    vertices, indices, solids, source_bounds, version, faces, omitted = load_mesh(
        source, args.ocp_path, args.linear_deflection_mm, args.angular_deflection_radians)
    volume, limits = validate_mesh(vertices, indices)
    write_outputs(args.output, args.obj_output, vertices, indices)
    provenance = {
        "schemaVersion": 1,
        "sourceFile": source.name,
        "sourceSha256": source_hash,
        "sourceUnits": "millimetres",
        "sourceCopyright": "Piab AB 2026; owner-supplied CAD",
        "converter": "Tools/RobotFrames/convert_kenos_sl_step.py",
        "cadKernel": "cadquery-ocp " + version,
        "dependencies": {name: importlib.metadata.version(name) for name in
                         ("cadquery-ocp", "cadquery-ocp-proxy", "vtk")},
        "linearDeflectionMm": args.linear_deflection_mm,
        "angularDeflectionRadians": args.angular_deflection_radians,
        "sourceBrepValid": True,
        "sourceFaceCount": faces,
        "omittedZeroAreaTriangles": omitted,
        "sourceBoundsMm": source_bounds,
        "sourceMountCentreMm": [0, -34, 31.75],
        "sourceFoamContactCentreMm": [0, 0, -58],
        "targetLink": "kenos_sl_base",
        "coordinateConvention": "Unity RUF metres: (-STEP X, 31.75-STEP Z, STEP Y+34)/1000; triangle winding reversed",
        "mountCentreUnityMetres": list(root_mm_to_unity((0, -34, 31.75))),
        "foamContactCentreUnityMetres": list(root_mm_to_unity((0, 0, -58))),
        "vertexCount": len(vertices),
        "indexCount": len(indices),
        "triangleCount": len(indices) // 3,
        "signedVolumeCubicMetres": volume,
        "boundsUnityMetres": limits,
        "binaryFormat": "KSL1 + little-endian int32 vertexCount,indexCount + float32 XYZ[vertexCount] + int32[indexCount]",
        "binarySha256": hashlib.sha256(args.output.with_suffix(".bytes").read_bytes()).hexdigest(),
        "objPath": args.obj_output.relative_to(PROJECT).as_posix() if args.obj_output.is_relative_to(PROJECT) else str(args.obj_output),
        "objSha256": hashlib.sha256(args.obj_output.read_bytes()).hexdigest(),
        "solids": solids,
        "notes": ["Detailed visuals only; ROS joint, TCP, mount and collision geometry remain authoritative.",
                  "DXX blank has no solid geometry in the source STEP.",
                  "The OnRobot mounting changer is not included in the supplied CAD.",
                  "STEP reader reports one unresolved reference; all 9 valid assembly solids and 794 faces transfer and tessellate."]
    }
    args.output.with_name(args.output.name + "Provenance").with_suffix(".json").write_text(
        json.dumps(provenance, indent=2) + "\n", encoding="utf-8")
    print(json.dumps({"output": str(args.output), "vertices": len(vertices),
                      "triangles": len(indices) // 3, "boundsUnityMetres": limits,
                      "binarySha256": provenance["binarySha256"]}, indent=2))


if __name__ == "__main__":
    main()
