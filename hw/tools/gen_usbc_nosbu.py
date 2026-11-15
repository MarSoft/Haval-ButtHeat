#!/usr/bin/env python3
"""Derive a USB-C receptacle footprint without the SBU pads (A8, B8).

SBU1/SBU2 are unused (USB 2.0 only); dropping their pads leaves a wider gap next to the CC pads,
which makes hand soldering the 0.3 mm signal pads easier. The connector's SBU pins then rest on
solder mask.

Usage: gen_usbc_nosbu.py STOCK.kicad_mod OUT.kicad_mod NEW_NAME
"""
import re
import sys

src = open(sys.argv[1]).read()
name = sys.argv[3]
DROP = {"A8", "B8"}

out, last, dropped = [], 0, []
for pm in re.finditer(r'\n\t\(pad "([^"]*)"', src):
    if pm.group(1) not in DROP:
        continue
    st = pm.start() + 1
    d = 0
    for j in range(st + 1, len(src)):
        if src[j] == '(':
            d += 1
        elif src[j] == ')':
            d -= 1
            if d == 0:
                break
    out.append(src[last:st])
    last = j + 2          # also eat the newline after the block
    dropped.append(pm.group(1))
out.append(src[last:])
dst = "".join(out)
old = re.search(r'\(footprint "([^"]+)"', dst).group(1)
dst = dst.replace(f'(footprint "{old}"', f'(footprint "{name}"', 1)
dst = re.sub(r'\(descr "([^"]*)"\)', lambda q: f'(descr "{q.group(1)} -- SBU pads A8/B8 removed '
             f'(hw/tools/gen_usbc_nosbu.py)")', dst, count=1)
open(sys.argv[2], "w").write(dst)
print("dropped:", dropped)
