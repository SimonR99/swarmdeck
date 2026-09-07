#!/usr/bin/env python3
"""Original, texture-free robot visuals for ARGoS. Metres, +X forward, +Z up.

Run from any directory; generated GLBs and descriptors live in argos/assets/robots.
Geometry is merged by material to keep draw calls bounded. Physical bodies and
sensor calibration stay in the ARGoS entity plugins; these are visual approximations.
"""

from pathlib import Path
import math

from make_argos_world import GLTFBuilder, Material

RUBBER = Material("rubber", (0.035, 0.04, 0.045), 0.95)
DARK = Material("graphite", (0.09, 0.105, 0.12), 0.65)
METAL = Material("alloy", (0.45, 0.49, 0.52), 0.35, 0.65)
ORANGE = Material("orange_shell", (0.86, 0.28, 0.045), 0.48)
YELLOW = Material("yellow_shell", (0.95, 0.69, 0.025), 0.45)
GLASS = Material("sensor_glass", (0.025, 0.06, 0.085), 0.16, 0.25)
LIGHT = Material("lamp", (0.85, 0.93, 0.96), 0.25)


class RobotMesh(GLTFBuilder):
    def cylinder_between(self, mat, a, b, radius, sections=16):
        """Capped cylinder along an arbitrary segment, without external libraries."""
        axis = tuple(b[i] - a[i] for i in range(3))
        length = math.sqrt(sum(v * v for v in axis))
        axis = tuple(v / length for v in axis)
        helper = (0, 0, 1) if abs(axis[2]) < 0.9 else (1, 0, 0)

        def cross(u, v):
            return (
                u[1] * v[2] - u[2] * v[1],
                u[2] * v[0] - u[0] * v[2],
                u[0] * v[1] - u[1] * v[0],
            )

        u = cross(axis, helper)
        norm = math.sqrt(sum(v * v for v in u))
        u = tuple(v / norm for v in u)
        v = cross(axis, u)
        ring = [
            tuple(
                radius
                * (
                    u[j] * math.cos(i * 2 * math.pi / sections)
                    + v[j] * math.sin(i * 2 * math.pi / sections)
                )
                for j in range(3)
            )
            for i in range(sections)
        ]
        for i in range(sections):
            r, s = ring[i], ring[(i + 1) % sections]
            ar, br = tuple(a[j] + r[j] for j in range(3)), tuple(
                b[j] + r[j] for j in range(3)
            )
            az, bz = tuple(a[j] + s[j] for j in range(3)), tuple(
                b[j] + s[j] for j in range(3)
            )
            self.add_quad(mat, ar, az, bz, br)
            self.add_tri(mat, a, az, ar)
            self.add_tri(mat, b, br, bz)

    def shell(self, mat, length, width, bottom, top, bevel=0.06):
        """Eight-sided hull with bevelled corners and a slightly inset top."""
        x, y = length / 2, width / 2
        outline = [
            (x - bevel, -y),
            (x, -y + bevel),
            (x, y - bevel),
            (x - bevel, y),
            (-x + bevel, y),
            (-x, y - bevel),
            (-x, -y + bevel),
            (-x + bevel, -y),
        ]
        lower = [(a, b, bottom) for a, b in outline]
        upper = [(a * 0.92, b * 0.92, top) for a, b in outline]
        for i in range(8):
            j = (i + 1) % 8
            self.add_quad(mat, lower[i], lower[j], upper[j], upper[i])
            self.add_tri(mat, (0, 0, top), upper[i], upper[j])
            self.add_tri(mat, (0, 0, bottom), lower[j], lower[i])

    def sensors(self, lidar, camera, deck):
        x, y, z = lidar
        self.add_cylinder(
            METAL, (x, y, (deck + z - 0.04) / 2), 0.018, z - 0.04 - deck, 12
        )
        self.add_cylinder(DARK, (x, y, z - 0.025), 0.051, 0.025, 20)
        self.add_cylinder(GLASS, (x, y, z), 0.05, 0.025, 20)
        self.add_cylinder(METAL, (x, y, z + 0.025), 0.05, 0.014, 20)
        x, y, z = camera
        # Housing ends behind the optical origin to avoid self-occlusion.
        self.add_box(DARK, (x - 0.025, y, z), (0.04, 0.1, 0.038))
        for side in (-1, 1):
            self.cylinder_between(
                GLASS,
                (x - 0.008, y + side * 0.029, z),
                (x - 0.002, y + side * 0.029, z),
                0.012,
                12,
            )


def bunker():
    m = RobotMesh()
    m.shell(ORANGE, 0.88, 0.47, 0.17, 0.355)
    m.add_box(DARK, (0, 0, 0.37), (0.94, 0.70, 0.02))
    for side in (-1, 1):
        y = side * 0.31
        # Closed capsule track profile: straight contact belt, rounded ends.
        contour = []
        for end, start in ((1, -math.pi / 2), (-1, math.pi / 2)):
            contour.extend(
                (
                    end * 0.4115 + 0.1 * math.cos(start + i * math.pi / 10),
                    0.1 + 0.1 * math.sin(start + i * math.pi / 10),
                )
                for i in range(11)
            )
        for i, (x, z) in enumerate(contour):
            nx, nz = contour[(i + 1) % len(contour)]
            m.add_quad(
                RUBBER,
                (x, y - 0.079, z),
                (nx, y - 0.079, nz),
                (nx, y + 0.079, nz),
                (x, y + 0.079, z),
            )
        for x in (-0.4115, -0.205, 0, 0.205, 0.4115):
            m.cylinder_between(
                DARK, (x, y - 0.077, 0.1), (x, y + 0.077, 0.1), 0.084, 16
            )
            m.cylinder_between(
                METAL, (x, y + side * 0.077, 0.1), (x, y + side * 0.08, 0.1), 0.045, 12
            )
        for i in range(19):
            x = -0.405 + i * 0.045
            for z in (0.008, 0.195):
                m.add_box(DARK, (x, y, z), (0.018, 0.163, 0.012))
        for x in (-0.46, 0.46):
            m.add_box(DARK, (x, 0, 0.24), (0.045, 0.48, 0.05))
        m.add_box(LIGHT, (0.478, side * 0.21, 0.305), (0.01, 0.075, 0.028))
        for i in range(5):
            m.add_box(
                DARK, (-0.20 + i * 0.06, side * 0.218, 0.30), (0.028, 0.015, 0.065)
            )
        m.cylinder_between(
            METAL, (-0.3, side * 0.27, 0.385), (0.3, side * 0.27, 0.385), 0.012, 8
        )
    m.sensors((-0.15, 0, 0.72), (0.515, 0, 0.30), 0.38)
    return m


def scout_mini():
    m = RobotMesh()
    m.shell(ORANGE, 0.52, 0.39, 0.095, 0.222, 0.04)
    m.add_box(DARK, (0, 0, 0.2335), (0.56, 0.50, 0.02))
    for x in (-0.203, 0.203):
        for side in (-1, 1):
            y = side * 0.225
            m.cylinder_between(
                RUBBER, (x, y - 0.035, 0.0875), (x, y + 0.035, 0.0875), 0.0875, 20
            )
            m.cylinder_between(
                METAL,
                (x, y + side * 0.035, 0.0875),
                (x, y + side * 0.038, 0.0875),
                0.048,
                12,
            )
            for i in range(12):
                angle = i * math.tau / 12
                m.cylinder_between(
                    DARK,
                    (
                        x + 0.032 * math.cos(angle),
                        y + side * 0.039,
                        0.0875 + 0.032 * math.sin(angle),
                    ),
                    (
                        x + 0.032 * math.cos(angle),
                        y + side * 0.042,
                        0.0875 + 0.032 * math.sin(angle),
                    ),
                    0.006,
                    6,
                )
    for x in (-0.285, 0.285):
        m.add_box(DARK, (x, 0, 0.145), (0.03, 0.38, 0.055))
    for side in (-1, 1):
        m.add_box(LIGHT, (0.302, side * 0.14, 0.165), (0.008, 0.052, 0.022))
        m.add_box(METAL, (0, side * 0.145, 0.247), (0.27, 0.018, 0.008))
    m.sensors((-0.08, 0, 0.4525), (0.322, 0, 0.2125), 0.2435)
    return m


def spot():
    m = RobotMesh()
    m.shell(YELLOW, 0.96, 0.30, 0.46, 0.63, 0.08)
    m.add_box(DARK, (0, 0, 0.455), (0.76, 0.26, 0.045))
    m.add_box(DARK, (0.47, 0, 0.545), (0.025, 0.24, 0.11))
    m.add_box(DARK, (0, 0, 0.639), (0.48, 0.22, 0.02))
    for x in (-0.33, 0.33):
        for side in (-1, 1):
            hip = (x, side * 0.205, 0.50)
            knee = (x + 0.115, side * 0.245, 0.285)
            foot = (x - 0.015, side * 0.26, 0.025)
            m.cylinder_between(DARK, (x, side * 0.14, 0.50), hip, 0.065, 16)
            m.cylinder_between(YELLOW, hip, knee, 0.043, 12)
            m.cylinder_between(
                DARK,
                (knee[0], knee[1] - 0.038, knee[2]),
                (knee[0], knee[1] + 0.038, knee[2]),
                0.05,
                14,
            )
            m.cylinder_between(DARK, knee, foot, 0.022, 12)
            m.add_cylinder(RUBBER, foot, 0.032, 0.045, 12)
    for side in (-1, 1):
        m.cylinder_between(
            GLASS, (0.484, side * 0.072, 0.55), (0.49, side * 0.072, 0.55), 0.026, 16
        )
        m.add_box(DARK, (-0.10, side * 0.146, 0.55), (0.14, 0.012, 0.055))
    m.sensors((-0.18, 0, 0.97), (0.598, 0, 0.52), 0.65)
    return m


def generate(output):
    output.mkdir(parents=True, exist_ok=True)
    for name, build, segmentation in [
        ("bunker", bunker, 10),
        ("scout_mini", scout_mini, 8),
        ("spot", spot, 9),
    ]:
        mesh = build()
        mesh.export_glb(output / (name + ".glb"))
        (output / (name + ".visual.xml")).write_text(
            f'<visual>\n  <model path="{name}.glb" scale="1" position="0,0,0" orientation="0,0,90" />\n  <segmentation class="{segmentation}" />\n</visual>\n'
        )
        triangles = sum(len(group[2]) // 3 for group in mesh._groups.values())
        print(f"{name}: {triangles} triangles, {len(mesh._groups)} material batches")


if __name__ == "__main__":
    generate(Path(__file__).resolve().parents[4] / "argos/assets/robots")
