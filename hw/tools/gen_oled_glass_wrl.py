#!/usr/bin/env python3
"""Coloured VRML model of the 0.91" OLED glass for footprint J4 (KiCad 3D viewer).

The STEP twin (OLED_0.91_glass_on_J4.step, uncoloured, made in FreeCAD) is what STEP export uses
(kicad-cli pcb export step --subst-models swaps .wrl for a .step of the same name).

Geometry from hw/SPEC.md (vendor drawing, case coordinates): glass 30.0 x 11.5 x 1.3 at
x -13.292..16.708, y 16.25..27.75; visible area 24.384 x 7.584 at 1.1/1.1 from the left/top glass
edges; active area 22.384 x 5.584 at 2.1/2.1. J4 origin = case (14.45, 20.25).

Usage: gen_oled_glass_wrl.py OUT.wrl
"""
import sys

J4 = (14.45, 20.25)
INCH10 = 2.54  # KiCad VRML unit = 0.1 inch

# (name, x0, x1, y0, y1, z0, z1, rgb, transparency) in case-coordinate mm
BOXES = [
    ("glass", -13.292, 16.708, 16.25, 27.75, 0.0, 1.28, (0.55, 0.62, 0.66), 0.35),
    # visible area as a black frame around the active area (4 strips), active area dark blue
    ("visible_l", -12.192, -11.192, 19.066, 26.65, 1.28, 1.30, (0.02, 0.02, 0.02), 0.0),
    ("visible_r", 11.192, 12.192, 19.066, 26.65, 1.28, 1.30, (0.02, 0.02, 0.02), 0.0),
    ("visible_b", -11.192, 11.192, 19.066, 20.066, 1.28, 1.30, (0.02, 0.02, 0.02), 0.0),
    ("visible_t", -11.192, 11.192, 25.65, 26.65, 1.28, 1.30, (0.02, 0.02, 0.02), 0.0),
    ("active", -11.192, 11.192, 20.066, 25.65, 1.28, 1.30, (0.05, 0.15, 0.45), 0.0),
]


def node(name, x0, x1, y0, y1, z0, z1, rgb, tr):
    cx = ((x0 + x1) / 2 - J4[0]) / INCH10
    cy = ((y0 + y1) / 2 - J4[1]) / INCH10
    cz = ((z0 + z1) / 2) / INCH10
    sx, sy, sz = ((x1 - x0) / INCH10, (y1 - y0) / INCH10, (z1 - z0) / INCH10)
    r, g, b = rgb
    return (f"# {name}\nTransform {{\n  translation {cx:.5f} {cy:.5f} {cz:.5f}\n  children Shape {{\n"
            f"    appearance Appearance {{ material Material {{ diffuseColor {r} {g} {b} "
            f"specularColor 0.3 0.3 0.3 shininess 0.4 transparency {tr} }} }}\n"
            f"    geometry Box {{ size {sx:.5f} {sy:.5f} {sz:.5f} }}\n  }}\n}}\n")


with open(sys.argv[1], "w") as f:
    f.write("#VRML V2.0 utf8\n# 0.91in OLED glass on J4 -- hw/tools/gen_oled_glass_wrl.py\n")
    for b in BOXES:
        f.write(node(*b))
print("written", sys.argv[1])
