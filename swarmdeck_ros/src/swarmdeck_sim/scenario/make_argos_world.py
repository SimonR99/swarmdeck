#!/usr/bin/env python3
"""Build the ARGoS backend's world geometry as glTF.

Two products, from one mesh builder:

1. `indoor.gltf` and `indoor_collision.gltf` (+ their `.bin`): the seeded
   indoor floor plan. The first is the photorealism `<prop>` the cameras and
   the photorealistic lidar raytrace; the second is the Jolt `<mesh>` the
   robots collide with. They carry the same transform, and MUST: if those ever
   disagree, robots collide with a building that is not where it is drawn.

   They differ in exactly one thing, and only because the physics engine
   supplies it instead: the collision mesh has no floor slab. See
   `build_indoor_world` for what a second ground surface did to rotation.

2. `props/*.glb`: one model per class in `adapters/perception/catalog.py`,
   the objects the detector is asked to find. These are static and committed;
   regenerate with `--props`.

The floor plan, the furniture and the target placement all come from
`generate_world.py`, which the Gazebo backend also reads, so a run of one
backend is comparable with a run of the other. Nothing here is random: the
same seed produces a byte-identical file, which `tests/integration/`
asserts (NFR-5).

Deliberately dependency-free. The mesh builder below is a few hundred lines of
struct packing rather than a trimesh import, because this runs inside the ROS
image, where every added dependency is paid for on every build, and because a
hand-rolled writer emits bytes in an order that cannot drift between library
versions.

    python3 make_argos_world.py --seed 20260801 -o ../worlds/indoor.gltf
    python3 make_argos_world.py --props ../../../../argos/assets/props
"""

from __future__ import annotations

import argparse
import json
import math
import struct
import sys
from pathlib import Path
from typing import Sequence

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

# The floor plan is shared with the Gazebo backend; see the module docstring.
from generate_world import FURNITURE, LAYOUT, WALL_H  # noqa: E402

Vec3 = tuple[float, float, float]


# --------------------------------------------------------------------------
# glTF writer
# --------------------------------------------------------------------------

class Material:
    """One PBR material. Equality is by value so parts can share slots."""

    __slots__ = ("name", "color", "roughness", "metallic")

    def __init__(self, name: str, color: Sequence[float],
                 roughness: float = 0.8, metallic: float = 0.0):
        self.name = name
        self.color = tuple(color)
        self.roughness = roughness
        self.metallic = metallic

    def as_gltf(self) -> dict:
        return {
            "name": self.name,
            "pbrMetallicRoughness": {
                "baseColorFactor": list(self.color) + [1.0],
                "metallicFactor": self.metallic,
                "roughnessFactor": self.roughness,
            },
        }


class GLTFBuilder:
    """Triangle-soup glTF 2.0 writer with one primitive per material.

    Geometry is authored in the ARGoS frame (z up, metres, origin on the
    floor) and rotated onto glTF's y-up convention on export, so every model
    written here is referenced with orientation="0,0,90" exactly like the
    photorealism plugin's own assets. Doing the rotation once, here, is what
    keeps the `<mesh>` and the `<prop>` in the experiment file agreeing.
    """

    def __init__(self) -> None:
        # material name -> (positions, normals, indices)
        self._groups: dict[str, tuple[list[float], list[float], list[int]]] = {}
        self._materials: dict[str, Material] = {}

    # -- primitives --------------------------------------------------------

    def _group(self, mat: Material):
        if mat.name not in self._groups:
            self._groups[mat.name] = ([], [], [])
            self._materials[mat.name] = mat
        return self._groups[mat.name]

    def add_tri(self, mat: Material, p0: Vec3, p1: Vec3, p2: Vec3,
                normal: Vec3 | None = None) -> None:
        pos, nrm, idx = self._group(mat)
        if normal is None:
            ux, uy, uz = (p1[i] - p0[i] for i in range(3))
            vx, vy, vz = (p2[i] - p0[i] for i in range(3))
            nx, ny, nz = (uy * vz - uz * vy, uz * vx - ux * vz, ux * vy - uy * vx)
            length = math.sqrt(nx * nx + ny * ny + nz * nz) or 1.0
            normal = (nx / length, ny / length, nz / length)
        start = len(pos) // 3
        for p in (p0, p1, p2):
            pos.extend(p)
            nrm.extend(normal)
        idx.extend([start, start + 1, start + 2])

    def add_quad(self, mat: Material, p0: Vec3, p1: Vec3, p2: Vec3, p3: Vec3,
                 normal: Vec3 | None = None) -> None:
        self.add_tri(mat, p0, p1, p2, normal)
        self.add_tri(mat, p0, p2, p3, normal)

    def add_box(self, mat: Material, centre: Vec3, size: Vec3,
                yaw: float = 0.0) -> None:
        """Axis-aligned box, optionally rotated about z by `yaw` radians."""
        hx, hy, hz = (s / 2.0 for s in size)
        cos_y, sin_y = math.cos(yaw), math.sin(yaw)

        def place(x: float, y: float, z: float) -> Vec3:
            return (centre[0] + x * cos_y - y * sin_y,
                    centre[1] + x * sin_y + y * cos_y,
                    centre[2] + z)

        def normal(x: float, y: float, z: float) -> Vec3:
            return (x * cos_y - y * sin_y, x * sin_y + y * cos_y, z)

        corners = {
            (sx, sy, sz): place(sx * hx, sy * hy, sz * hz)
            for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)
        }
        faces = (
            (( 1,  0,  0), (( 1, -1, -1), ( 1,  1, -1), ( 1,  1,  1), ( 1, -1,  1))),
            ((-1,  0,  0), ((-1,  1, -1), (-1, -1, -1), (-1, -1,  1), (-1,  1,  1))),
            (( 0,  1,  0), (( 1,  1, -1), (-1,  1, -1), (-1,  1,  1), ( 1,  1,  1))),
            (( 0, -1,  0), ((-1, -1, -1), ( 1, -1, -1), ( 1, -1,  1), (-1, -1,  1))),
            (( 0,  0,  1), ((-1, -1,  1), ( 1, -1,  1), ( 1,  1,  1), (-1,  1,  1))),
            (( 0,  0, -1), ((-1,  1, -1), ( 1,  1, -1), ( 1, -1, -1), (-1, -1, -1))),
        )
        for n, quad in faces:
            self.add_quad(mat, *(corners[c] for c in quad), normal(*n))

    def add_cylinder(self, mat: Material, centre: Vec3, radius: float,
                     height: float, sections: int = 24) -> None:
        """Z-axis cylinder, `centre` at its mid-height."""
        cx, cy, cz = centre
        z0, z1 = cz - height / 2.0, cz + height / 2.0
        ring = [(math.cos(2.0 * math.pi * i / sections),
                 math.sin(2.0 * math.pi * i / sections)) for i in range(sections)]
        for i in range(sections):
            ax, ay = ring[i]
            bx, by = ring[(i + 1) % sections]
            pa0 = (cx + ax * radius, cy + ay * radius, z0)
            pb0 = (cx + bx * radius, cy + by * radius, z0)
            pa1 = (cx + ax * radius, cy + ay * radius, z1)
            pb1 = (cx + bx * radius, cy + by * radius, z1)
            # Side, with per-vertex-ish normals approximated per face.
            mid = ((ax + bx) / 2.0, (ay + by) / 2.0)
            length = math.hypot(*mid) or 1.0
            self.add_quad(mat, pa0, pb0, pb1, pa1,
                          (mid[0] / length, mid[1] / length, 0.0))
            self.add_tri(mat, (cx, cy, z1), pa1, pb1, (0.0, 0.0, 1.0))
            self.add_tri(mat, (cx, cy, z0), pb0, pa0, (0.0, 0.0, -1.0))

    def add_sphere(self, mat: Material, centre: Vec3, radius: float,
                   segments: int = 20, rings: int = 12,
                   scale: Vec3 = (1.0, 1.0, 1.0)) -> None:
        """UV sphere, optionally scaled per axis into an ellipsoid."""
        cx, cy, cz = centre

        def point(lat: float, lon: float) -> Vec3:
            return (cx + radius * scale[0] * math.sin(lat) * math.cos(lon),
                    cy + radius * scale[1] * math.sin(lat) * math.sin(lon),
                    cz + radius * scale[2] * math.cos(lat))

        for r in range(rings):
            lat0 = math.pi * r / rings
            lat1 = math.pi * (r + 1) / rings
            for s in range(segments):
                lon0 = 2.0 * math.pi * s / segments
                lon1 = 2.0 * math.pi * (s + 1) / segments
                p00, p01 = point(lat0, lon0), point(lat0, lon1)
                p10, p11 = point(lat1, lon0), point(lat1, lon1)
                if r == 0:
                    self.add_tri(mat, p00, p11, p10)
                elif r == rings - 1:
                    self.add_tri(mat, p00, p01, p10)
                else:
                    self.add_quad(mat, p00, p01, p11, p10)

    # -- export ------------------------------------------------------------

    def _pack(self) -> tuple[bytes, dict]:
        """Interleave nothing; lay out position, normal and index blocks per
        material, then describe them. Returns (binary, gltf-json-without-buffer).
        """
        blob = bytearray()
        materials: list[dict] = []
        accessors: list[dict] = []
        views: list[dict] = []
        primitives: list[dict] = []

        for name in sorted(self._groups):
            pos, nrm, idx = self._groups[name]
            if not idx:
                continue
            mat_index = len(materials)
            materials.append(self._materials[name].as_gltf())

            # ARGoS z-up -> glTF y-up: (x, y, z) becomes (x, z, -y).
            flat_pos: list[float] = []
            flat_nrm: list[float] = []
            for i in range(0, len(pos), 3):
                flat_pos.extend((pos[i], pos[i + 2], -pos[i + 1]))
                flat_nrm.extend((nrm[i], nrm[i + 2], -nrm[i + 1]))

            mins = [min(flat_pos[i::3]) for i in range(3)]
            maxs = [max(flat_pos[i::3]) for i in range(3)]

            def emit(payload: bytes, target: int, align: int = 4) -> int:
                while len(blob) % align:
                    blob.append(0)
                offset = len(blob)
                blob.extend(payload)
                views.append({"buffer": 0, "byteOffset": offset,
                              "byteLength": len(payload), "target": target})
                return len(views) - 1

            pos_view = emit(struct.pack(f"<{len(flat_pos)}f", *flat_pos), 34962)
            nrm_view = emit(struct.pack(f"<{len(flat_nrm)}f", *flat_nrm), 34962)
            idx_view = emit(struct.pack(f"<{len(idx)}I", *idx), 34963)

            count = len(flat_pos) // 3
            accessors.append({"bufferView": pos_view, "componentType": 5126,
                              "count": count, "type": "VEC3",
                              "min": mins, "max": maxs})
            accessors.append({"bufferView": nrm_view, "componentType": 5126,
                              "count": count, "type": "VEC3"})
            accessors.append({"bufferView": idx_view, "componentType": 5125,
                              "count": len(idx), "type": "SCALAR"})
            base = len(accessors) - 3
            primitives.append({
                "attributes": {"POSITION": base, "NORMAL": base + 1},
                "indices": base + 2,
                "material": mat_index,
                "mode": 4,
            })

        gltf = {
            "asset": {"version": "2.0", "generator": "SwarmDeck make_argos_world"},
            "scene": 0,
            "scenes": [{"nodes": [0]}],
            "nodes": [{"name": "world", "mesh": 0}],
            "meshes": [{"name": "world", "primitives": primitives}],
            "materials": materials,
            "accessors": accessors,
            "bufferViews": views,
        }
        return bytes(blob), gltf

    def export_gltf(self, path: Path) -> None:
        blob, gltf = self._pack()
        bin_name = path.with_suffix(".bin").name
        gltf["buffers"] = [{"uri": bin_name, "byteLength": len(blob)}]
        path.parent.mkdir(parents=True, exist_ok=True)
        path.with_suffix(".bin").write_bytes(blob)
        # sort_keys and a fixed separator so the same inputs give the same bytes.
        path.write_text(json.dumps(gltf, indent=2, sort_keys=True) + "\n")

    def export_glb(self, path: Path) -> None:
        blob, gltf = self._pack()
        gltf["buffers"] = [{"byteLength": len(blob)}]
        json_chunk = json.dumps(gltf, sort_keys=True,
                                separators=(",", ":")).encode("utf-8")
        json_chunk += b" " * (-len(json_chunk) % 4)
        bin_chunk = blob + b"\x00" * (-len(blob) % 4)
        total = 12 + 8 + len(json_chunk) + 8 + len(bin_chunk)
        path.parent.mkdir(parents=True, exist_ok=True)
        with path.open("wb") as handle:
            handle.write(struct.pack("<III", 0x46546C67, 2, total))
            handle.write(struct.pack("<II", len(json_chunk), 0x4E4F534A))
            handle.write(json_chunk)
            handle.write(struct.pack("<II", len(bin_chunk), 0x004E4942))
            handle.write(bin_chunk)

    @property
    def triangle_count(self) -> int:
        return sum(len(idx) for _, _, idx in self._groups.values()) // 3


# --------------------------------------------------------------------------
# Materials
# --------------------------------------------------------------------------

FLOOR = Material("floor", (0.34, 0.34, 0.36), roughness=0.92)
WALL = Material("wall", (0.80, 0.80, 0.78), roughness=0.88)
WOOD_TOP = Material("wood_top", (0.55, 0.30, 0.12), roughness=0.55)
WOOD_LEG = Material("wood_leg", (0.42, 0.22, 0.09), roughness=0.65)
SEAT = Material("seat", (0.16, 0.36, 0.54), roughness=0.70)
CHAIR_LEG = Material("chair_leg", (0.22, 0.24, 0.27), roughness=0.45, metallic=0.5)
FRAME = Material("frame", (0.12, 0.07, 0.03), roughness=0.55)
POT = Material("pot", (0.58, 0.20, 0.09), roughness=0.80)
LEAVES = Material("leaves", (0.08, 0.48, 0.20), roughness=0.85)


# --------------------------------------------------------------------------
# The indoor world
# --------------------------------------------------------------------------

# The floor slab is a little larger than the building so the walls stand on
# something at every seam, and thin because it is collision geometry too:
# a robot spawned at z=0 rests on its top face.
FLOOR_HALF = 13.0
FLOOR_THICKNESS = 0.10


def build_table(mesh: GLTFBuilder, x: float, y: float, yaw: float = 0.0) -> None:
    mesh.add_box(WOOD_TOP, (x, y, 0.76), (1.3, 0.85, 0.10), yaw)
    for lx, ly in ((-0.55, -0.35), (-0.55, 0.35), (0.55, -0.35), (0.55, 0.35)):
        ox = x + lx * math.cos(yaw) - ly * math.sin(yaw)
        oy = y + lx * math.sin(yaw) + ly * math.cos(yaw)
        mesh.add_box(WOOD_LEG, (ox, oy, 0.36), (0.08, 0.08, 0.72), yaw)


def build_chair(mesh: GLTFBuilder, x: float, y: float, yaw: float) -> None:
    mesh.add_box(SEAT, (x, y, 0.45), (0.48, 0.48, 0.09), yaw)
    bx = x - 0.20 * math.cos(yaw)
    by = y - 0.20 * math.sin(yaw)
    mesh.add_box(SEAT, (bx, by, 0.78), (0.08, 0.48, 0.62), yaw)
    mesh.add_box(CHAIR_LEG, (x, y, 0.21), (0.38, 0.38, 0.42), yaw)


def build_painting(mesh: GLTFBuilder, x: float, y: float, z: float,
                   yaw: float, color: str) -> None:
    mesh.add_box(FRAME, (x, y, z), (1.15, 0.055, 0.78), yaw)
    canvas = Material(f"canvas_{color.replace(' ', '_').replace('.', '')}",
                      tuple(float(c) for c in color.split()), roughness=0.75)
    cx = x - 0.032 * -math.sin(yaw)
    cy = y - 0.032 * math.cos(yaw)
    mesh.add_box(canvas, (cx, cy, z), (0.98, 0.018, 0.61), yaw)


def build_plant(mesh: GLTFBuilder, x: float, y: float) -> None:
    mesh.add_cylinder(POT, (x, y, 0.20), 0.20, 0.40)
    mesh.add_sphere(LEAVES, (x, y, 0.62), 0.32)


_FURNITURE_BUILDERS = {
    "table": build_table,
    "chair": build_chair,
    "painting": build_painting,
    "plant": build_plant,
}


def build_indoor_world(floor: bool = True) -> GLTFBuilder:
    """Walls and furniture, in the ARGoS frame with z up, optionally floored.

    `floor=False` is what the COLLISION mesh is built with, and the reason is
    worth stating because the two meshes otherwise being identical is a rule
    this deliberately breaks.

    The Jolt engine already provides the ground as a `<floor height="0">`
    plane. Cooking a floor slab into the collision mesh as well puts a second
    surface at exactly the same height, so every robot rests on both at once
    and the solver has to reconcile two coplanar contacts under it. Measured
    against 0.4 rad/s commanded: rotation collapsed to 0-35% of the commanded
    rate, varying with arena position, while translation was unaffected. A
    robot that drives but will not turn, and nothing logs a thing.

    Removing the slab from the collision mesh restored 90-100% for all three
    platforms at three different arena positions.

    The slab stays in the VISUAL mesh, because something has to be drawn.
    """
    mesh = GLTFBuilder()
    if floor:
        mesh.add_box(FLOOR, (0.0, 0.0, -FLOOR_THICKNESS / 2.0),
                     (FLOOR_HALF * 2.0, FLOOR_HALF * 2.0, FLOOR_THICKNESS))
    for x0, y0, x1, y1 in LAYOUT:
        mesh.add_box(WALL,
                     ((x0 + x1) / 2.0, (y0 + y1) / 2.0, WALL_H / 2.0),
                     (abs(x1 - x0), abs(y1 - y0), WALL_H))
    for kind, _index, args in FURNITURE:
        _FURNITURE_BUILDERS[kind](mesh, *args)
    return mesh


def collision_path(world: Path) -> Path:
    """Where the collision mesh sits, given the visual one."""
    return world.with_name(f"{world.stem}_collision{world.suffix}")


# --------------------------------------------------------------------------
# Detection targets
# --------------------------------------------------------------------------

DUCK_BODY = Material("duck_body", (1.00, 0.82, 0.03), roughness=0.35)
DUCK_BEAK = Material("duck_beak", (1.00, 0.38, 0.02), roughness=0.40)
DUCK_EYE = Material("duck_eye", (0.02, 0.02, 0.03), roughness=0.20)
BLOCK_WOOD = Material("block_wood", (0.76, 0.58, 0.33), roughness=0.60)
CONE_ORANGE = Material("cone_orange", (0.95, 0.35, 0.05), roughness=0.55)
SPOOL_FILAMENT = Material("spool_filament", (0.06, 0.06, 0.07), roughness=0.45)
SPOOL_RIM = Material("spool_rim", (0.20, 0.21, 0.24), roughness=0.35, metallic=0.3)
NOODLE_FOAM = Material("noodle_foam", (0.20, 0.45, 0.85), roughness=0.90)


def make_rubber_duck() -> GLTFBuilder:
    """The same duck the Gazebo world built from SDF primitives, as geometry.

    Kept dimensionally identical (0.17 m body, 0.33 m overall) so a detection
    range measured on one backend means the same thing on the other.
    """
    mesh = GLTFBuilder()
    mesh.add_sphere(DUCK_BODY, (0.0, 0.0, 0.16), 0.17, scale=(1.15, 1.0, 0.95))
    mesh.add_sphere(DUCK_BODY, (0.105, 0.0, 0.345), 0.115)
    # A tail the Gazebo duck did not have. It lengthens the silhouette by
    # about 9 cm, which is the one dimension where the two backends' ducks
    # differ; it is here because recognisability is the entire reason for
    # switching to a rendered scene, and a tailless sphere reads as a ball.
    mesh.add_sphere(DUCK_BODY, (-0.160, 0.0, 0.245), 0.085,
                    scale=(1.2, 0.75, 0.85))
    mesh.add_box(DUCK_BEAK, (0.215, 0.0, 0.325), (0.10, 0.105, 0.045))
    for side in (1.0, -1.0):
        mesh.add_sphere(DUCK_EYE, (0.18, side * 0.078, 0.378), 0.018)
    return mesh


def make_wooden_block() -> GLTFBuilder:
    mesh = GLTFBuilder()
    mesh.add_box(BLOCK_WOOD, (0.0, 0.0, 0.045), (0.09, 0.09, 0.09))
    return mesh


def make_disc_cone() -> GLTFBuilder:
    """A flat sports saucer: a shallow dome on a brim, not a highway cone.

    The catalog's own note explains why the shape matters: every prompt with
    the word "cone" in it scores at or below 0.10 on this object, and the
    prompts that find it describe an orange plastic saucer.
    """
    mesh = GLTFBuilder()
    # Brim, then a dome whose lower half is buried in it. The dome centre is
    # its own z half-extent (0.062 * 0.55) above the floor so that nothing
    # dips below z=0: a prop that sinks through the floor is invisible to a
    # lidar looking at it edge-on and pokes out of the underside of the world.
    mesh.add_cylinder(CONE_ORANGE, (0.0, 0.0, 0.004), 0.095, 0.008, sections=32)
    mesh.add_sphere(CONE_ORANGE, (0.0, 0.0, 0.0341), 0.062,
                    scale=(1.0, 1.0, 0.55), rings=8)
    return mesh


def make_filament_spool() -> GLTFBuilder:
    mesh = GLTFBuilder()
    # Lying flat on its lower rim, which is how a spool sits on a bench. The
    # stack is measured from the floor up (lower rim 0.000..0.006) rather than
    # from an arbitrary centre, or the whole prop floats.
    mesh.add_cylinder(SPOOL_FILAMENT, (0.0, 0.0, 0.033), 0.088, 0.055,
                      sections=32)
    for z in (0.003, 0.063):
        mesh.add_cylinder(SPOOL_RIM, (0.0, 0.0, z), 0.100, 0.006, sections=32)
    mesh.add_cylinder(SPOOL_RIM, (0.0, 0.0, 0.033), 0.028, 0.062, sections=20)
    return mesh


def make_pool_noodle() -> GLTFBuilder:
    """A foam tube lying on its side, which is how one is ever found indoors."""
    mesh = GLTFBuilder()
    sections = 20
    length, radius = 1.30, 0.035
    for i in range(sections):
        a0 = 2.0 * math.pi * i / sections
        a1 = 2.0 * math.pi * (i + 1) / sections
        y0, z0 = math.cos(a0) * radius, math.sin(a0) * radius + radius
        y1, z1 = math.cos(a1) * radius, math.sin(a1) * radius + radius
        mesh.add_quad(NOODLE_FOAM,
                      (-length / 2.0, y0, z0), (length / 2.0, y0, z0),
                      (length / 2.0, y1, z1), (-length / 2.0, y1, z1))
        for end, normal in ((-length / 2.0, (-1.0, 0.0, 0.0)),
                            (length / 2.0, (1.0, 0.0, 0.0))):
            mesh.add_tri(NOODLE_FOAM, (end, 0.0, radius), (end, y0, z0),
                         (end, y1, z1), normal)
    return mesh


# Ordered to match adapters/perception/catalog.py, which is what decides both
# the protocol class name and the prompts the detector is given.
PROPS = {
    "rubber_duck": make_rubber_duck,
    "wooden_block": make_wooden_block,
    "disc_cone": make_disc_cone,
    "filament_spool": make_filament_spool,
    "pool_noodle": make_pool_noodle,
}


def target_classes(count: int) -> list[str]:
    """Which class sits at each seeded target position.

    Round robin from the catalog, duck first: `place_targets` puts target 0 in
    robot_0's opening field of view, and a rubber duck is the class with the
    most measured evidence behind its prompts, so it is the one that makes the
    camera -> detector -> adapter -> UI path testable on the first frame.
    """
    names = list(PROPS)
    return [names[i % len(names)] for i in range(count)]


def generate_props(outdir: Path) -> list[Path]:
    written = []
    for name, factory in PROPS.items():
        path = outdir / f"{name}.glb"
        mesh = factory()
        mesh.export_glb(path)
        written.append(path)
        print(f"[props] {path.name}: {mesh.triangle_count} triangles, "
              f"{path.stat().st_size / 1024:.1f} KiB")
    return written


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--seed", type=int, default=20260801,
                    help="Floor plan seed; the same seed gives the same bytes.")
    ap.add_argument("-o", "--output",
                    default=str(HERE.parent / "worlds" / "indoor.gltf"),
                    help="Output .gltf path; the .bin lands beside it.")
    ap.add_argument("--props", metavar="DIR", default=None,
                    help="Regenerate the committed detection-target models "
                         "into DIR instead of building the world.")
    args = ap.parse_args()

    if args.props:
        generate_props(Path(args.props))
        return 0

    out = Path(args.output)
    visual = build_indoor_world(floor=True)
    visual.export_gltf(out)
    collision = build_indoor_world(floor=False)
    collision_out = collision_path(out)
    collision.export_gltf(collision_out)
    print(f"[world] seed={args.seed} walls={len(LAYOUT)} "
          f"furniture={len(FURNITURE)}")
    print(f"[world]   visual    {visual.triangle_count} triangles -> {out}")
    print(f"[world]   collision {collision.triangle_count} triangles -> "
          f"{collision_out}  (no floor slab; see build_indoor_world)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
