"""ButtHeat v2: replaceable cable clamp insert (SLA), two halves split top/bottom at the cable axis.

Run headless; writes hw/cad/out/ClampTop.brep and ClampBottom.brep in the assembled position (case
coordinates, inside the left part's seat). Geometry and the cable diameter come from v2_cable.py
(CABLE_D = V2Vars.cable_d): re-run for another cable, reprint only the insert.

Assembly: lay the cable into the left part's slot from below, put both halves around it (the bottom half's
pegs into the top half), slide them into the seat from the J1 side until the outer end hits the seat's end,
then fix the bottom half's tab with an M2x4 self-tapping screw from below (2.5 mm into the boss). Assembled on the cable, the halves
stand 0.1 taller than the seat, so the seat squeezes them onto the cable; grip rings bite into the jacket.
"""
import math
import FreeCAD as App
import Part

OUT = '/home/mars/my/buttheat/hw/cad/out/'
P = App.Vector
exec(open('/home/mars/my/buttheat/hw/cad/v2_cable.py').read())

sx0, sx1, sy, sz = SEAT['x0'], SEAT['x1'], SEAT['y'], SEAT['z']
xa, xb = sx0 + INSERT_GAP, sx1 - INSERT_GAP          # outer end (pull side) .. J1 end
R = INSERT_OD / 2
rb = (CABLE_D - BORE_SQUEEZE) / 2                    # bore radius
rr = rb - GRIP_H                                      # grip ring inner radius
LEAD = 0.4                                           # lead-in chamfer on the outer end (slides in first)
ENTRY = 0.3                                          # bore entry chamfers (no sharp edge on the cable)

# profile in the (x, r) half-plane, revolved about the axis: outer surface with the lead-in, bore with
# GRIP_N grip rings spread over the length
L = xb - xa
pitch = L / (GRIP_N + 1)
inner = [(xb, rb + ENTRY), (xb - ENTRY, rb)]
for i in range(GRIP_N, 0, -1):
    xc = xa + i * pitch
    inner += [(xc + GRIP_W / 2, rb), (xc + GRIP_W / 2, rr), (xc - GRIP_W / 2, rr), (xc - GRIP_W / 2, rb)]
inner += [(xa + ENTRY, rb), (xa, rb + ENTRY)]
prof = [(xa, R - LEAD), (xa + LEAD, R), (xb, R)] + inner
pts = [P(x, 0, r) for x, r in prof]
body = Part.Face(Part.makePolygon(pts + [pts[0]])).revolve(P(0, 0, 0), P(1, 0, 0), 360)
body.translate(P(0, sy, sz))
assert body.isValid(), 'insert body invalid'

g = SPLIT_GAP / 2
# split: at the axis on the +y side; raised by SPLIT_STEP on the -y (tab) side, stepping where the bore edge is
# at that height (so neither half gets a sliver next to the bore)
ys = sy - math.sqrt(rb ** 2 - SPLIT_STEP ** 2)
zl, zh = sz + SPLIT_STEP, sz
def region(pts_yz):
    f = Part.Face(Part.makePolygon([P(xa - 1, y, z) for y, z in pts_yz] + [P(xa - 1, *pts_yz[0])]))
    return f.extrude(P(L + 2, 0, 0))
low = region([(sy - 10, sz - 10), (sy + 10, sz - 10), (sy + 10, zh - g), (ys, zh - g), (ys, zl - g), (sy - 10, zl - g)])
upp = region([(sy - 10, zl + g), (ys, zl + g), (ys, zh + g), (sy + 10, zh + g), (sy + 10, sz + 10), (sy - 10, sz + 10)])
top = body.common(upp)
bot = body.common(low)

# pegs on the bottom half's two lands (between bore and outside), holes in the top half
xm = (xa + xb) / 2
for yp, zf in ((sy - (rb + R) / 2 - 0.1, zl - g), (sy + (rb + R) / 2, zh - g)):
    bot = bot.fuse(Part.makeCylinder(PEG_D / 2, PEG_H + 0.3, P(xm, yp, zf - 0.3)))
    top = top.cut(Part.makeCylinder(PEG_HOLE_D / 2, PEG_H + 2 * g + 0.2 + 0.1, P(xm, yp, zf + 2 * g - 0.1)))

# tab at the J1 end (towards -y) under the left part's M2 boss, joined to the bottom half's raised -y wall
# by a web as thick as the tab
T = TAB
bot = bot.fuse(Part.makeBox(T['x1'] - T['x0'], T['y1'] - T['y0'], T['z1'] - T['z0'], P(T['x0'], T['y0'], T['z0'])))
web = Part.makeBox(xb - T['x0'], (ys - 0.6) - (T['y1'] - 0.3), T['z1'] - T['z0'], P(T['x0'], T['y1'] - 0.3, T['z0']))
bot = bot.fuse(web.cut(Part.makeCylinder(rb + 0.01, L + 2, P(xa - 1, sy, sz), P(1, 0, 0))))
bot = bot.cut(Part.makeCylinder(M2['clr_d'] / 2, 5, P(M2['x'], M2['y'], T['z0'] - 1)))

top, bot = top.removeSplitter(), bot.removeSplitter()
import os
os.makedirs(OUT, exist_ok=True)
for name, sh in (('ClampTop', top), ('ClampBottom', bot)):
    sh.exportBrep(OUT + name + '.brep')
    print(name, sh.isValid(), round(sh.Volume, 1), 'cable d', CABLE_D, 'bore', round(2 * rb, 2), 'rings', round(2 * rr, 2))
