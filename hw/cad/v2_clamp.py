"""ButtHeat v2: cable clamp saddle (SLA), presses the jacket up into the left part's groove boss (v2_cable.py).

Run headless; writes hw/cad/out/Saddle.brep in the assembled position (case coordinates). An arc-shaped bar
under the cable on the ear's arc (R SADDLE_R around the vent centre, phi CLAMP_PHI), z SADDLE_Z, with the
groove's lower half (S-wave, CABLE_D - SQUEEZE) on top; two lobes on the outer side of the curve: one for
the M2x4 self-tapper from below into the boss, one carrying a pin that goes up into the boss (no rotation).
Screwed home, it stops SQUEEZE (0.5) below the boss: the cable is held by the squeeze and the S-wave.
For another cable: set V2Vars.cable_d, re-run v2_left.py and this, reprint the saddle (and the left part,
whose groove half follows the diameter too).

Assembly: front part upside down, board not fitted; lay the cable into the groove and the exit channel, put
the saddle on (pin into its hole), screw it.
"""
import math
import FreeCAD as App
import Part

OUT = '/home/mars/my/buttheat/hw/cad/out/'
P = App.Vector
exec(open('/home/mars/my/buttheat/hw/cad/v2_cable.py').read())

c0, c1 = CLAMP_PHI
z0, z1 = SADDLE_Z
r0, r1 = SADDLE_R
a0, a1 = min(c0, c1), max(c0, c1)
bar = Part.makeCylinder(r1, z1 - z0, P(ARC_C[0], ARC_C[1], z0), P(0, 0, 1), a1 - a0).cut(
    Part.makeCylinder(r0, z1 - z0 + 2, P(ARC_C[0], ARC_C[1], z0 - 1), P(0, 0, 1), 360))
bar.rotate(P(ARC_C[0], ARC_C[1], 0), P(0, 0, 1), a0)
lz0 = SCREW['lobe_z0']
for f in (SCREW, PIN):
    x, y = polar(f['phi'], f['r'])
    bar = bar.fuse(Part.makeCylinder(f['lobe_r'], z1 - lz0, P(x, y, lz0)))
x, y = polar(PIN['phi'], PIN['r'])
bar = bar.fuse(Part.makeCylinder(PIN['d'] / 2, (BOSS_Z0 - z1) + PIN['len'] + 0.3, P(x, y, z1 - 0.3)))
bar = bar.cut(cable_tube(CABLE_D - SQUEEZE, c0 + 0.5, c1 - 0.5))   # groove's lower half (runs out past the ends)
x, y = polar(SCREW['phi'], SCREW['r'])
bar = bar.cut(Part.makeCylinder(SCREW['clr_d'] / 2, 10, P(x, y, z0 - 1)))
bar = bar.removeSplitter()

import os
os.makedirs(OUT, exist_ok=True)
bar.exportBrep(OUT + 'Saddle.brep')
print('Saddle', bar.isValid(), round(bar.Volume, 1), bar.BoundBox, 'cable d', CABLE_D, 'groove', CABLE_D - SQUEEZE)
