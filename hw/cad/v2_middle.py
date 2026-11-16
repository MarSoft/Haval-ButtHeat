"""ButtHeat v2 front case, MIDDLE part (SLA), around the display and the car knob.

Run headless (MCP execute_code_headless / freecadcmd); writes hw/cad/out/Middle.brep.
Load into the GUI with hw/cad/v2_load.py.

Shape (case coordinates, z 0 = car console surface, board top z 3.5):
  - x -SEAM..SEAM; front edge flush with the board's knob-arch edge, resting on the board top;
  - top z 6.5 over the glass (glass top 4.8, 0.2 air, 1.5 wall);
  - bevel plane in front of the glass: from (y 16.05, z 6.5) down to z 5.0 at the seams;
  - back face on the 15 deg display frame (0.2 tape gap); the skin rests on the rear plate's lip
    (lip must be <= z TOP - WALL in this zone);
  - display window = visible area + 0.2, 45 deg flare outwards;
  - spigots: the inner half of the wall (0.75) continues SPIGOT mm past each seam into the side parts.
"""
import math
import FreeCAD as App
import Part

OUT = '/home/mars/my/buttheat/hw/cad/out/'
P = App.Vector

SEAM = 20.0
SPIGOT = 3.0
WALL = 1.5
BOARD_TOP = 3.5
TOP = 6.5
GLASS_Y0 = 16.05          # glass bottom edge (16.25) - 0.2
BEVEL_Z_AT_SEAM = 5.0
FRAME_Y0, FRAME_TAN, TAPE_GAP = 29.75, math.tan(math.radians(15)), 0.2
LIP_FRONT = 28.3
# visible area of the glass (vendor), + margin
WIN = (-12.192 - 0.2, 12.192 + 0.2, 19.066 - 0.2, 26.65 + 0.2)

# board knob-arch edge (KiCad Edge.Cuts, case coordinates), x ascending
ARCH = [(-22.51, 8.33), (-18.93, 8.71), (-16.94, 10.34), (-12.47, 12.88), (-7.63, 14.61), (-2.57, 15.495),
        (2.57, 15.495), (7.63, 14.61), (12.47, 12.88), (16.94, 10.34), (18.93, 8.71), (22.51, 8.33)]


def arch_y(x):
    for (x0, y0), (x1, y1) in zip(ARCH, ARCH[1:]):
        if x0 <= x <= x1:
            return y0 + (y1 - y0) * (x - x0) / (x1 - x0)
    raise ValueError(x)


BEVEL_TAN = (TOP - BEVEL_Z_AT_SEAM) / (GLASS_Y0 - arch_y(SEAM))


def footprint(inset, x0, x1, y_back):
    """Plan-view region between the (inset) arch edge and y_back, x0..x1."""
    pts = [P(x, y, 0) for x, y in ARCH]
    pts = [P(-40, ARCH[0][1], 0)] + pts + [P(40, ARCH[-1][1], 0), P(40, 45, 0), P(-40, 45, 0)]
    face = Part.Face(Part.makePolygon(pts + [pts[0]]))
    if inset:
        face = face.makeOffset2D(-inset, join=2)
    clip = Part.Face(Part.makePolygon([P(x0, -5, 0), P(x1, -5, 0), P(x1, y_back, 0), P(x0, y_back, 0), P(x0, -5, 0)]))
    return face.common(clip)


def envelope(t, x0, x1, z_bottom, y_back):
    """Outer envelope shrunk by t: top TOP-t, bevel plane and frame plane moved inwards by t."""
    f = footprint(t, x0, x1, y_back)
    s = f.extrude(P(0, 0, (TOP - t) - z_bottom))
    s.translate(P(0, 0, z_bottom))
    big = 200
    # bevel: remove everything above the plane through (y GLASS_Y0, z TOP-t) sloping down towards -y
    z_at = lambda y: (TOP - t) - (GLASS_Y0 - y) * BEVEL_TAN
    cutter = Part.Face(Part.makePolygon([P(0, GLASS_Y0, z_at(GLASS_Y0)), P(0, -20, z_at(-20)), P(0, -20, 30),
                                         P(0, GLASS_Y0, 30), P(0, GLASS_Y0, z_at(GLASS_Y0))])).extrude(P(big, 0, 0))
    cutter.translate(P(-big / 2, 0, 0))
    s = s.cut(cutter)
    # frame: remove y > frame plane (tape gap, then t inwards)
    yf = lambda z: FRAME_Y0 - TAPE_GAP - t / math.cos(math.radians(15)) + z * FRAME_TAN
    fr = Part.Face(Part.makePolygon([P(0, yf(-5), -5), P(0, yf(30), 30), P(0, 60, 30), P(0, 60, -5), P(0, yf(-5), -5)])).extrude(P(big, 0, 0))
    fr.translate(P(-big / 2, 0, 0))
    return s.cut(fr)


FRONT_ROUND = 1.0
# Rounded outer envelope: shrink by r, grow back by r with arc joins (rounds every convex edge, works on
# the faceted arch where fillets fail), then cut the bottom flat at the board top and the ends flat at
# the seams so only the outside edges (top-front along the knob, top-back along the frame) stay round.
_core = envelope(FRONT_ROUND, -SEAM - 5, SEAM + 5, BOARD_TOP - 3, 45)
_round = _core.makeOffsetShape(FRONT_ROUND, 0.01, join=0)
outer = _round.common(Part.makeBox(2 * SEAM, 60, 20, P(-SEAM, -10, BOARD_TOP)))
print('rounded outer envelope valid', outer.isValid())
cavity = envelope(WALL, -SEAM - SPIGOT - 1, SEAM + SPIGOT + 1, BOARD_TOP - 1, LIP_FRONT)
part = outer.cut(cavity)
for sgn in (-1, 1):
    xa, xb = (SEAM, SEAM + SPIGOT) if sgn > 0 else (-SEAM - SPIGOT, -SEAM)
    sleeve = envelope(WALL / 2, xa, xb, BOARD_TOP, LIP_FRONT + WALL / 2).cut(cavity)
    part = part.fuse(sleeve)
# part-specific clearances (case coordinates): x0, x1, y0, y1, z top of the cut
KEEPOUT = [(-13.5, 16.9, 16.05, 27.95, TOP - WALL),    # glass + 0.2: the front wall stops above it
           (20.4, 25.8, 24.6, 27.8, TOP - 0.3),        # D2 (SMA, top 5.71) under the right spigot
           (-SEAM - SPIGOT - 1, SEAM + SPIGOT + 1, LIP_FRONT, 40, TOP - WALL)]   # the rear plate's lip (z <= 5.0)
for x0, x1, y0, y1, zt in KEEPOUT:
    part = part.cut(Part.makeBox(x1 - x0, y1 - y0, zt - (BOARD_TOP - 0.5), P(x0, y0, BOARD_TOP - 0.5)))
# display window with 45 deg flare
x0, x1, y0, y1 = WIN
zb, zt = TOP - WALL - 0.01, TOP + 0.01
d = zt - zb
w_lo = Part.makePolygon([P(x0, y0, zb), P(x1, y0, zb), P(x1, y1, zb), P(x0, y1, zb), P(x0, y0, zb)])
w_hi = Part.makePolygon([P(x0 - d, y0 - d, zt), P(x1 + d, y0 - d, zt), P(x1 + d, y1 + d, zt), P(x0 - d, y1 + d, zt), P(x0 - d, y0 - d, zt)])
part = part.cut(Part.makeLoft([w_lo, w_hi], True, True))
part = part.removeSplitter()

import os
os.makedirs(OUT, exist_ok=True)
part.exportBrep(OUT + 'Middle.brep')
print('Middle', part.isValid(), round(part.Volume, 1), part.BoundBox, 'bevel tan', round(BEVEL_TAN, 4))
