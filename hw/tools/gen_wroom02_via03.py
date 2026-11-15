#!/usr/bin/env python3
"""Derive ESP32-C3-WROOM-02 footprint with 0.3 mm thermal vias (JLC standard capability).

The stock RF_Module:ESP32-C3-WROOM-02 puts 13 vias of 0.2 mm drill (0.55 mm pitch) into the
2.9 x 2.9 mm GND pad. Here they are replaced by a 3 x 3 grid of 0.3 mm drill / 0.6 mm pad vias at
1.1 mm pitch (span 2.2 + 0.6 = 2.8 mm, inside the pad). The vias also allow soldering the centre
pad from the back side by hand.

Usage: gen_wroom02_via03.py STOCK.kicad_mod OUT.kicad_mod
"""
import re
import sys
import uuid

src = open(sys.argv[1]).read()
NAME = "ESP32-C3-WROOM-02_Via0.3"

# locate the centre pad (the SMD rect of pad 19) to centre the via grid on it
m = re.search(r'\(pad "19" smd rect\s*\(at ([-\d.]+) ([-\d.]+)\)', src)
cx, cy = float(m.group(1)), float(m.group(2))


def blocks(text):
    """Yield (start, end) of every top-level (pad ...) block."""
    for pm in re.finditer(r'\n\t\(pad ', text):
        st = pm.start() + 2
        d = 0
        for j in range(st, len(text)):
            if text[j] == '(':
                d += 1
            elif text[j] == ')':
                d -= 1
                if d == 0:
                    yield st, j + 1
                    break


# drop the stock thru-hole vias of pad 19
out, last = [], 0
for st, en in blocks(src):
    blk = src[st:en]
    if blk.startswith('(pad "19" thru_hole'):
        out.append(src[last:st].rstrip("\t"))
        last = en
out.append(src[last:])
dst = "".join(out)

vias = []
for i in (-1, 0, 1):
    for j in (-1, 0, 1):
        vias.append(
            f'\t(pad "19" thru_hole circle\n\t\t(at {cx + 1.1 * i:.4f} {cy + 1.1 * j:.4f})\n'
            f'\t\t(size 0.6 0.6)\n\t\t(drill 0.3)\n\t\t(property pad_prop_heatsink)\n'
            f'\t\t(layers "*.Cu" "*.Mask")\n\t\t(remove_unused_layers no)\n\t\t(zone_connect 2)\n'
            f'\t\t(uuid "{uuid.uuid4()}")\n\t)\n')
# relax the antenna keep-out: keep the copper pour away from the antenna (a ground plane under it
# kills the radio), but allow tracks, vias and pads, so J1 (pins under the antenna end) can be routed
dst, n_keep = re.subn(r'\(keepout\s*\(tracks [a-z_]+\)\s*\(vias [a-z_]+\)\s*\(pads [a-z_]+\)\s*'
                      r'\(copperpour [a-z_]+\)\s*\(footprints [a-z_]+\)\s*\)',
                      '(keepout\n\t\t\t(tracks allowed)\n\t\t\t(vias allowed)\n\t\t\t(pads allowed)\n'
                      '\t\t\t(copperpour not_allowed)\n\t\t\t(footprints allowed)\n\t\t)', dst)
assert n_keep == 1, n_keep
k = dst.index("\t(embedded_fonts")
dst = dst[:k] + "".join(vias) + dst[k:]
dst = dst.replace('(footprint "ESP32-C3-WROOM-02"', f'(footprint "{NAME}"', 1)
dst = re.sub(r'\(descr "([^"]*)"\)', lambda q: f'(descr "{q.group(1)} -- 3x3 0.3mm thermal vias '
             f'(hw/tools/gen_wroom02_via03.py)")', dst, count=1)
open(sys.argv[2], "w").write(dst)
print(f"centre pad at ({cx}, {cy}); {len(vias)} vias written")
