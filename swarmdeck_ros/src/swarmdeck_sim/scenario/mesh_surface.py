"""Sample a world's GLB triangles for placement; no runtime physics dependency.

Only embedded, static glTF 2.0 triangle meshes are supported. Unsupported
geometry fails explicitly instead of silently placing colliders at z=0.
"""

from pathlib import Path
import json
import math
import struct

import numpy as np


class GlbGeometry:
    def __init__(self, path):
        data = Path(path).read_bytes()
        if struct.unpack_from("<4sII", data) != (b"glTF", 2, len(data)):
            raise ValueError(f"{path}: expected a glTF 2.0 GLB")
        offset = 12
        self.binary = None
        self.doc = None
        while offset < len(data):
            size, kind = struct.unpack_from("<II", data, offset)
            chunk = data[offset + 8 : offset + 8 + size]
            if kind == 0x4E4F534A:
                self.doc = json.loads(chunk)
            elif kind == 0x004E4942:
                self.binary = chunk
            offset += 8 + size
        if self.doc is None or self.binary is None:
            raise ValueError(f"{path}: GLB needs JSON and embedded geometry")

    def accessor(self, index):
        a = self.doc["accessors"][index]
        if a.get("sparse") or "bufferView" not in a:
            raise ValueError("Sparse/missing geometry accessors are unsupported")
        view = self.doc["bufferViews"][a["bufferView"]]
        if view.get("buffer", 0) != 0:
            raise ValueError("Only embedded GLB geometry is supported")
        types = {5121: "u1", 5123: "<u2", 5125: "<u4", 5126: "<f4"}
        dtype = np.dtype(types[a["componentType"]])
        width = {"SCALAR": 1, "VEC3": 3}[a["type"]]
        return np.ndarray(
            (a["count"], width),
            dtype=dtype,
            buffer=self.binary,
            offset=view.get("byteOffset", 0) + a.get("byteOffset", 0),
            strides=(view.get("byteStride", width * dtype.itemsize), dtype.itemsize),
        )

    def triangles(self, *, material_prefix=None):
        nodes = self.doc.get("nodes", [])
        parents = {
            child: i
            for i, node in enumerate(nodes)
            for child in node.get("children", [])
        }
        cache = {}

        def world(i):
            if i in cache:
                return cache[i]
            node = nodes[i]
            if "matrix" in node:
                local = np.array(node["matrix"], dtype=float).reshape(4, 4).T
            else:
                x, y, z, w = node.get("rotation", [0, 0, 0, 1])
                local = np.eye(4)
                local[:3, :3] = np.array(
                    [
                        [
                            1 - 2 * (y * y + z * z),
                            2 * (x * y - z * w),
                            2 * (x * z + y * w),
                        ],
                        [
                            2 * (x * y + z * w),
                            1 - 2 * (x * x + z * z),
                            2 * (y * z - x * w),
                        ],
                        [
                            2 * (x * z - y * w),
                            2 * (y * z + x * w),
                            1 - 2 * (x * x + y * y),
                        ],
                    ]
                ) @ np.diag(node.get("scale", [1, 1, 1]))
                local[:3, 3] = node.get("translation", [0, 0, 0])
            cache[i] = world(parents[i]) @ local if i in parents else local
            return cache[i]

        for i, node in enumerate(nodes):
            if "mesh" not in node:
                continue
            if "skin" in node:
                raise ValueError("Skinned placement geometry is unsupported")
            for p in self.doc["meshes"][node["mesh"]]["primitives"]:
                material = (
                    self.doc.get("materials", [])[p["material"]].get("name", "")
                    if "material" in p
                    else ""
                )
                if material_prefix and not material.lower().startswith(material_prefix):
                    continue
                if p.get("mode", 4) != 4:
                    raise ValueError("Placement geometry must use triangles")
                vertices = self.accessor(p["attributes"]["POSITION"])
                t = world(i)
                vertices = vertices @ t[:3, :3].T + t[:3, 3]
                # Same explicit +90-degree X rotation as the visual and y_up=false collider.
                vertices = vertices[:, [0, 2, 1]] * [1, -1, 1]
                indices = (
                    self.accessor(p["indices"]).ravel()
                    if "indices" in p
                    else np.arange(len(vertices))
                )
                yield vertices[indices.reshape(-1, 3)]


class MeshSurface:
    """The ground of a mesh world, for standing detection targets on it.

    `material_prefix` keeps only the triangles whose glTF material name starts
    with it (Bistro's "pavement"); None keeps everything, which is what a
    material-less collision mesh needs. `z_offset` is the world's z translation
    in the experiment, so heights come out in ARGoS coordinates. `below` caps
    the surfaces considered ground: in a tunnel the highest triangle over a
    point is the ceiling.
    """

    def __init__(self, path, *, material_prefix=None, z_offset=0.0, below=None):
        triangles = list(GlbGeometry(path).triangles(material_prefix=material_prefix))
        if not triangles:
            what = f"'{material_prefix}' material" if material_prefix else "geometry"
            raise ValueError(f"{path}: no ground {what} for object placement")
        self.triangles = np.concatenate(triangles)
        self.triangles[:, :, 2] += z_offset
        self.below = below
        a, b, c = self.triangles.transpose(1, 0, 2)
        self.den = (b[:, 1] - c[:, 1]) * (a[:, 0] - c[:, 0]) + (c[:, 0] - b[:, 0]) * (
            a[:, 1] - c[:, 1]
        )
        self.lo = self.triangles.min(axis=1)
        self.hi = self.triangles.max(axis=1)

    def _restricted(self, x_min, x_max, y_min, y_max, z_min=None, z_max=None):
        """A view over the triangles whose xy box meets the given box, so a
        footprint's samples do not each scan a million-triangle world, and,
        given a height band, only those reaching into it."""
        keep = (
            (self.hi[:, 0] >= x_min)
            & (self.lo[:, 0] <= x_max)
            & (self.hi[:, 1] >= y_min)
            & (self.lo[:, 1] <= y_max)
        )
        if z_min is not None:
            keep &= self.hi[:, 2] >= z_min
        if z_max is not None:
            keep &= self.lo[:, 2] <= z_max
        sub = MeshSurface.__new__(MeshSurface)
        sub.triangles = self.triangles[keep]
        sub.den = self.den[keep]
        sub.lo = self.lo[keep]
        sub.hi = self.hi[keep]
        sub.below = self.below if z_max is None else z_max
        return sub

    def height(self, x, y):
        keep = (
            (abs(self.den) > 1e-10)
            & (self.lo[:, 0] <= x)
            & (self.hi[:, 0] >= x)
            & (self.lo[:, 1] <= y)
            & (self.hi[:, 1] >= y)
        )
        a, b, c = self.triangles[keep].transpose(1, 0, 2)
        d = self.den[keep]
        u = (
            (b[:, 1] - c[:, 1]) * (x - c[:, 0]) + (c[:, 0] - b[:, 0]) * (y - c[:, 1])
        ) / d
        v = (
            (c[:, 1] - a[:, 1]) * (x - c[:, 0]) + (a[:, 0] - c[:, 0]) * (y - c[:, 1])
        ) / d
        heights = u * a[:, 2] + v * b[:, 2] + (1 - u - v) * c[:, 2]
        hits = (u >= -1e-7) & (v >= -1e-7) & (u + v <= 1 + 1e-7)
        if self.below is not None:
            hits &= heights <= self.below
        if not hits.any():
            raise ValueError(f"No ground below target at ({x:g}, {y:g})")
        return float(heights[hits].max())

    def place(self, model, x, y, yaw, near_z=None):
        """The z that stands `model` on the ground at (x, y), rotated by yaw.

        `near_z` is the floor height when it is known (walkable.py): only
        surfaces from half a metre below it to 0.3 m above count, so a target
        on a lower level of a multi-level world stands on that level's floor,
        not on the ceiling of the level below or the floor of the one above.
        """
        vertices = np.concatenate(list(GlbGeometry(model).triangles())).reshape(-1, 3)
        lo, hi = vertices.min(axis=0), vertices.max(axis=0)
        # Sample the rotated footprint at <=5 cm spacing, including its edges.
        xs = np.linspace(lo[0], hi[0], max(2, math.ceil((hi[0] - lo[0]) / 0.05) + 1))
        ys = np.linspace(lo[1], hi[1], max(2, math.ceil((hi[1] - lo[1]) / 0.05) + 1))
        co, si = math.cos(yaw), math.sin(yaw)
        reach = math.hypot(max(abs(lo[0]), abs(hi[0])), max(abs(lo[1]), abs(hi[1])))
        band = (None, None) if near_z is None else (near_z - 0.5, near_z + 0.3)
        local = self._restricted(x - reach, x + reach, y - reach, y + reach, *band)
        ground = max(
            local.height(x + co * a - si * b, y + si * a + co * b)
            for a in xs
            for b in ys
        )
        return ground - float(lo[2]) + 0.005
