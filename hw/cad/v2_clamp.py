"""ButtHeat v2: replaceable cable clamp insert (SLA), two halves split top/bottom at the cable axis.

Run headless; writes hw/cad/out/ClampTop.brep and ClampBottom.brep in the assembled position (case
coordinates, inside the left part's seat). Geometry and the cable diameter come from v2_cable.py
(CABLE_D = V2Vars.cable_d): re-run for another cable, reprint only the insert.

Assembly: lay the cable into the left part's slot from below, put both halves around it (the bottom half's
pegs into the top half), slide them into the seat from the J1 side until the outer end hits the seat's end,
then fix the bottom half's tab with an M2 self-tapping screw from below. Assembled on the cable, the halves
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
top = body.common(Part.makeBox(L + 2, 20, 10, P(xa - 1, sy - 10, sz + g)))
bot = body.common(Part.makeBox(L + 2, 20, 10, P(xa - 1, sy - 10, sz - g - 10)))

# pegs on the bottom half's two lands (between bore and outside), holes in the top half
xm, rp = (xa + xb) / 2, (rb + R) / 2
for s in (-1, 1):
    bot = bot.fuse(Part.makeCylinder(PEG_D / 2, PEG_H + g + 0.3, P(xm, sy + s * rp, sz - g - 0.3)))
    top = top.cut(Part.makeCylinder(PEG_HOLE_D / 2, PEG_H + 0.2 + 0.1, P(xm, sy + s * rp, sz + g - 0.1)))

# tab at the J1 end (towards -y) under the left part's M2 boss, joined to the bottom half by a web that
# stays below the split face
T = TAB
bot = bot.fuse(Part.makeBox(T['x1'] - T['x0'], T['y1'] - T['y0'], T['z1'] - T['z0'], P(T['x0'], T['y0'], T['z0'])))
web_y1 = sy - rb - 0.5
web = Part.makeBox(xb - T['x0'], web_y1 - (T['y1'] - 0.2), (sz - g - 0.3) - T['z0'], P(T['x0'], T['y1'] - 0.2, T['z0']))
bot = bot.fuse(web.cut(Part.makeCylinder(rb + 0.01, L + 2, P(xa - 1, sy, sz), P(1, 0, 0))))
bot = bot.cut(Part.makeCylinder(M2['clr_d'] / 2, 5, P(M2['x'], M2['y'], T['z0'] - 1)))

top, bot = top.removeSplitter(), bot.removeSplitter()
import os
os.makedirs(OUT, exist_ok=True)
for name, sh in (('ClampTop', top), ('ClampBottom', bot)):
    sh.exportBrep(OUT + name + '.brep')
    print(name, sh.isValid(), round(sh.Volume, 1), 'cable d', CABLE_D, 'bore', round(2 * rb, 2), 'rings', round(2 * rr, 2))
