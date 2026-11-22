"""ButtHeat v2: cable clamp saddle (SLA) for the three-bar clamp in the left part (v2_cable.py).

Run headless; writes hw/cad/out/Saddle.brep in the assembled position, screwed home (case coordinates).
An arc-shaped base plate under the cable (R SADDLE_R around the vent centre, phi SADDLE_PHI, z SADDLE_BASE)
with the middle bar rising between the two case bars; the M2x4 self-tapper goes through the screw lobe right
at that bar (outer side of the curve) into the case's screw column, and the lobe's top bottoms out on the
column at the maximum bend (hard stop); the lobe is joined to the body by a full-height web (rigid ring).
A foot rib along the inner (wall-side) edge reaches down to 0.05 over the rear plate top, which carries that
edge (the screw pulls the outer side up, the cable pushes the middle down). No pin: the screw and the arc wall
(0.05 off the inner edge) keep the saddle from turning.
Tightening the screw pushes the cable into a zig-zag around the bars; it works over a range of cable
diameters (no reprint for a slightly different cable).

Assembly: front part upside down, board not fitted; lay the cable under the case bars and into the exit
channel, push the saddle straight up into place, screw it until it stops; the foot only bears once the
rear plate is on.
"""
import math
import FreeCAD as App
import Part

OUT = '/home/mars/my/buttheat/hw/cad/out/'
P = App.Vector
exec(open('/home/mars/my/buttheat/hw/cad/v2_cable.py').read())
cz = Part.makeCylinder


def sector(r0, r1, phi0, phi1, z0, z1):
    a0, a1 = min(phi0, phi1), max(phi0, phi1)
    ring = cz(r1, z1 - z0, P(ARC_C[0], ARC_C[1], z0), P(0, 0, 1), a1 - a0).cut(
        cz(r0, z1 - z0 + 2, P(ARC_C[0], ARC_C[1], z0 - 1), P(0, 0, 1), 360))
    ring.rotate(P(ARC_C[0], ARC_C[1], 0), P(0, 0, 1), a0)
    return ring


z0, z1 = SADDLE_BASE
r0, r1 = SADDLE_R
s = sector(r0, r1, SADDLE_PHI[0], SADDLE_PHI[1], z0, z1)
# middle bar, fully rounded tip at SADDLE_BAR_TIP
dphi = math.degrees(BAR_T / CABLE_R)
zc = SADDLE_BAR_TIP - BAR_TIP_R
rb1 = r1 - 0.3                             # bar ends short of the case's screw column (R 25.75)
s = s.fuse(sector(r0, rb1, SADDLE_BAR_PHI - dphi / 2, SADDLE_BAR_PHI + dphi / 2, z1 - 0.1, zc - 0.05))   # not tangent to the tip
a = math.radians(SADDLE_BAR_PHI)
ra, rb = r0 + 0.05, rb1 - 0.05            # tip ends inside the slab's curved faces (tangency crashes OCCT)
s = s.fuse(cz(BAR_TIP_R, rb - ra, P(ARC_C[0] + ra * math.cos(a), ARC_C[1] + ra * math.sin(a), zc), P(math.cos(a), math.sin(a), 0)))
# screw lobe up to the hard stop, joined solidly to the body (no side gaps): a web flaring from the lobe to
# the base over the lobe's full height, kept outside the cable (R >= RING_WEB_R0); the base under it, and the
# middle bar's outer end tied to the lobe below the cable
x, y = polar(SCREW['phi'], SCREW['r'])
s = s.fuse(cz(SCREW['lobe_r'], STOP_Z - z0, P(x, y, z0)))
def _hull(pts):
    """2D convex hull (monotone chain), counter-clockwise."""
    pts = sorted(set(pts))
    def cross(o, a, b):
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])
    lo, up = [], []
    for p_ in pts:
        while len(lo) >= 2 and cross(lo[-2], lo[-1], p_) <= 0: lo.pop()
        lo.append(p_)
    for p_ in reversed(pts):
        while len(up) >= 2 and cross(up[-2], up[-1], p_) <= 0: up.pop()
        up.append(p_)
    return lo[:-1] + up[:-1]


# ring + web: ONE plan outline (convex hull of the screw ring and the body's outer edge over +-RING_WEB_W),
# extruded from the base bottom to the hard stop; the cable's space (R < RING_WEB_R0 above the base) cut out.
# (The previous trapezoid web only grazed the ring, leaving slots on both sides.)
dw = math.degrees(RING_WEB_W / RING_WEB_R0)
pts = [polar(SCREW['phi'] + dw * k / 8, RING_WEB_R0 - 0.35) for k in range(-8, 9)]
pts += [(x + SCREW['lobe_r'] * math.cos(2 * math.pi * k / 48), y + SCREW['lobe_r'] * math.sin(2 * math.pi * k / 48)) for k in range(48)]
hull = _hull([(round(u, 6), round(v, 6)) for u, v in pts])
ring = Part.Face(Part.makePolygon([P(u, v, z0) for u, v in hull] + [P(hull[0][0], hull[0][1], z0)])).extrude(P(0, 0, STOP_Z - z0))
ring = ring.cut(sector(r0 - 2, RING_WEB_R0, SCREW['phi'] - 30, SCREW['phi'] + 30, z1, STOP_Z + 1))
s = s.fuse(ring)
s = s.fuse(sector(rb1 - 0.1, RING_WEB_R0 + 0.1, SADDLE_BAR_PHI - dphi / 2, SADDLE_BAR_PHI + dphi / 2, z1 - 0.1,
                  STOP_Z - 0.1))                                     # bar end -> web, under the cable
# foot: a rib along the inner (wall-side) edge down onto the rear plate, which takes that edge's load (the
# screw pulls the outer side up, the cable pushes down in the middle)
fr0, fr1, fz = SADDLE_FOOT
s = s.fuse(sector(fr0, fr1, SADDLE_PHI[0] + 0.3, SADDLE_PHI[1] - 0.3, fz, z0 + 0.1))
x, y = polar(SCREW['phi'], SCREW['r'])
s = s.cut(cz(SCREW['clr_d'] / 2, 10, P(x, y, z0 - 1)))
# (removeSplitter returns an empty shape here: skipped)

import os
os.makedirs(OUT, exist_ok=True)
s.exportBrep(OUT + 'Saddle.brep')
print('Saddle', s.isValid(), round(s.Volume, 1), s.optimalBoundingBox(), 'bar tip', round(SADDLE_BAR_TIP, 2),
      'case bar tips', round(CASE_BAR_TIP, 2), 'zig', ZIG)
