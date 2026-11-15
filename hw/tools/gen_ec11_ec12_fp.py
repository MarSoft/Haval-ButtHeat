#!/usr/bin/env python3
"""Universal EC11/EC12 rotary encoder footprint (with push switch, vertical shaft).

Derived from KiCad's RotaryEncoder_Alps_EC12E-Switch_Vertical_H20mm. EC11E-Switch has identical
A/B/C and S1/S2 holes (S labels swapped -- irrelevant, the switch has no polarity) and the same
shaft axis; only the mounting-lug slots differ: EC12E 2.5 x 2.0, EC11E 2.8 x 1.5. The union slot
2.8 x 2.0 (pad 3.2 x 2.4, 0.2 mm ring; 3.4 x 2.6 hit 16 existing tracks) accepts both the thick EC12 tabs and the wide EC11 tabs.

Usage: gen_ec11_ec12_fp.py STOCK_EC12E.kicad_mod OUT.kicad_mod
"""
import re
import sys

NAME = "RotaryEncoder_EC11_EC12-Switch_Vertical_H20mm"
src = open(sys.argv[1]).read()

dst, n = re.subn(r'(\(pad "MP" thru_hole rect\s*\(at [-\d.]+ [-\d.]+\)\s*)\(size 3 2\.5\)(\s*)\(drill oval 2\.5 2\)',
                 r'\1(size 3.2 2.4)\2(drill oval 2.8 2)', src)
assert n == 2, n
dst = dst.replace('(footprint "RotaryEncoder_Alps_EC12E-Switch_Vertical_H20mm"', f'(footprint "{NAME}"', 1)
dst = re.sub(r'\(descr "([^"]*)"\)', lambda q: '(descr "EC11 / EC12 rotary encoder with switch, vertical shaft; '
             'EC12E-Switch land pattern with 2.8 x 2.0 mounting slots accepting EC11E tabs too '
             f'(hw/tools/gen_ec11_ec12_fp.py). Base: {q.group(1)}")', dst, count=1)
dst = re.sub(r'\(property "Value" "[^"]*"', f'(property "Value" "{NAME}"', dst, count=1)

# Both encoder bodies, overlaid, so the case is designed around the union of EC11 and EC12:
# - EC12 composite (own measurements: body 12.5 x 13.4 x 6, slim bushing D7 to z 11, EC11 shaft
#   tip at z 20), already in footprint coordinates;
# - EC11B with switch (GrabCAD, pins along -Y in its own frame): shaft to (7.5, -2.5), turned +90 (KiCad model Z rotation is clockwise-positive; -90 put the 3-pin row on the switch side).
MODELS = [("${KIPRJMOD}/3d/EC12_switch_H20_composite.step", (0, 0, 0), (0, 0, 0)),
          ("${KIPRJMOD}/3d/EC11B-SW-Vert-15F-green.step", (7.5, -2.5, 0), (0, 0, 90))]


def model(path, off, rot):
    return (f'\t(model "{path}"\n\t\t(offset\n\t\t\t(xyz {off[0]} {off[1]} {off[2]})\n\t\t)\n'
            f'\t\t(scale\n\t\t\t(xyz 1 1 1)\n\t\t)\n\t\t(rotate\n\t\t\t(xyz {rot[0]} {rot[1]} {rot[2]})\n\t\t)\n\t)\n')


dst, n = re.subn(r'\t\(model "[^"]*"\s*\(offset\s*\(xyz [^)]*\)\s*\)\s*\(scale\s*\(xyz [^)]*\)\s*\)\s*'
                 r'\(rotate\s*\(xyz [^)]*\)\s*\)\s*\)\n', lambda _: ''.join(model(*m) for m in MODELS), dst)
assert n == 1, n
open(sys.argv[2], "w").write(dst)
print("written", sys.argv[2])
