#!/usr/bin/env python3
"""Universal 6x6 mm tactile switch footprint: THT (SW_PUSH_6mm) + SMD gull-wing (PTS645Sx43SMTR92).

One symbol per button instead of a THT/SMD pair. Origin at the switch body centre (= the SMD footprint's
origin). The THT holes (stock SW_PUSH_6mm: pin 1 at (0, 0), body centre at (3.25, 2.25)) land at
(+-3.25, -2.25) for pin 1 and (+-3.25, 2.25) for pin 2, next to the SMD pads at (+-3.98, -+2.25) with the
same pin numbers, exactly as the former overlaid SW1/SW2 pair on the board. Silkscreen redrawn to clear
both pad sets; courtyard = union; both 3D models.

Usage: gen_sw_6x6_fp.py STOCK_PTS645.kicad_mod STOCK_SW_PUSH_6mm.kicad_mod OUT.kicad_mod
"""
import re
import sys
import uuid

NAME = "SW_Tactile_6x6_THT_SMD"
smd, tht, out = sys.argv[1:4]
src = open(smd).read()
tsrc = open(tht).read()


def blocks(text, head):
    """Top-level (one tab) s-expression blocks starting with head, e.g. '(fp_line'."""
    res, i = [], 0
    while True:
        i = text.find('\n\t' + head, i)
        if i < 0:
            return res
        depth, j = 0, text.index("(", i)
        while True:
            c = text[j]
            depth += c == '('
            depth -= c == ')'
            j += 1
            if depth == 0:
                break
        res.append(text[i:j])
        i = j


# drop the SMD footprint's silkscreen and courtyard (redrawn below), keep its fab drawing
for b in blocks(src, '(fp_line') + blocks(src, '(fp_circle'):
    if '(layer "F.SilkS")' in b or '(layer "F.CrtYd")' in b:
        src = src.replace(b, '', 1)
src = src.replace('(footprint "SW_SPST_PTS645Sx43SMTR92"', f'(footprint "{NAME}"', 1)
src = re.sub(r'\(descr "[^"]*"\)', '(descr "Tactile switch 6x6 mm, universal: THT (SW_PUSH_6mm, 6.5 x 4.5 mm pins) or SMD '
             'gull-wing (C&K PTS645Sx43SMTR92); same pin numbers (hw/tools/gen_sw_6x6_fp.py)")', src, count=1)
src = re.sub(r'\(property "Value" "[^"]*"', f'(property "Value" "{NAME}"', src, count=1)
src = src.replace('(attr smd)', '(attr through_hole)', 1)

u = lambda: str(uuid.uuid4())


def line(a, b, layer, w):
    return (f'\n\t(fp_line\n\t\t(start {a[0]} {a[1]})\n\t\t(end {b[0]} {b[1]})\n\t\t(stroke\n\t\t\t(width {w})\n'
            f'\t\t\t(type solid)\n\t\t)\n\t\t(layer "{layer}")\n\t\t(uuid "{u()}")\n\t)')


# silkscreen: body outline +-3 cut back to clear the THT pads (D2 at (+-3.25, +-2.25)) and the SMD pads
# (x 3.2..4.76, y +-(1.6..2.9)) by >= 0.2 mm; actuator circle
g = []
for s in (-1, 1):
    g.append(line((-2.2, 3.0 * s), (2.2, 3.0 * s), 'F.SilkS', 0.12))      # top / bottom edges
    g.append(line((2.95 * s, -1.0), (2.95 * s, 1.0), 'F.SilkS', 0.12))    # side stubs between the pins
g.append(f'\n\t(fp_circle\n\t\t(center 0 0)\n\t\t(end 1.75 0)\n\t\t(stroke\n\t\t\t(width 0.12)\n\t\t\t(type solid)\n\t\t)\n'
         f'\t\t(fill no)\n\t\t(layer "F.SilkS")\n\t\t(uuid "{u()}")\n\t)')
# courtyard: union of both land patterns (THT pads to x 4.25 / y 3.25, SMD pads to x 4.76) + 0.25
cx, cy = 5.05, 3.55
pts = [(-cx, -cy), (cx, -cy), (cx, cy), (-cx, cy)]
g += [line(a, b, 'F.CrtYd', 0.05) for a, b in zip(pts, pts[1:] + pts[:1])]

# THT pads from the stock footprint, moved so the body centre is the origin
pads = []
for b in blocks(tsrc, '(pad '):
    m = re.search(r'\(at ([-\d.]+) ([-\d.]+)( [-\d.]+)?\)', b)
    x, y = float(m.group(1)) - 3.25, float(m.group(2)) - 2.25
    b = b.replace(m.group(0), f'(at {x:g} {y:g}{m.group(3) or ""})', 1)
    b = re.sub(r'\(uuid "[^"]*"\)', f'(uuid "{u()}")', b)
    pads.append(b)
assert len(pads) == 4, len(pads)

# 3D models: the SMD one stays; the THT one is placed at its pin 1 (-3.25, -2.25) (model offsets are Y-up)
tmodel = blocks(tsrc, '(model ')[0]
tmodel = re.sub(r'\(offset\s*\(xyz [^)]*\)\s*\)', '(offset\n\t\t\t(xyz -3.25 2.25 0)\n\t\t)', tmodel, count=1)

i = src.find('\n\t(fp_text user "${REFERENCE}"')
src = src[:i] + ''.join(g) + src[i:]
i = src.find('\n\t(embedded_fonts')
src = src[:i] + ''.join(pads) + src[i:]
i = src.rfind('\n)')
src = src[:i] + tmodel + src[i:]
open(out, 'w').write(src)
print('written', out)
