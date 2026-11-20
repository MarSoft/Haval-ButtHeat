#!/usr/bin/env python3
"""USB-C receptacle 16PF-015 (Rui Chi drawing 1012-16F****01; 16P, top-mount SMT, L 7.55 mm) for J2,
as ButtHeat:USB_C_16PF-015.

It uses the XKB U262-16XN-4BVC11 land pattern that J2's routing was drawn for: same signal pads, rear
shell tabs (1.0 x 2.1, slot 0.6 x 1.7) and locating pegs (NPTH 0.65); only the front shell tabs differ
(pad 1.0 x 1.8, slot 0.6 x 1.4, 4.15 ahead of the rear tabs). So this is derived from our XKB footprint
(SBU pads A8/B8 already removed):
  - front tabs replaced; their pads are cut flat CU_EDGE_CLR before the board edge (EDGE_Y, J2's origin
    to the board's front edge; re-generate if J2 moves), leaving ~0.13 mm of ring at the slot's front end
    (DRC rule "J2 shell tabs: ring cut at the board edge");
  - fab outline for the 7.55 mm body (front face 2.60 ahead of the front tab centres = local y 3.645,
    i.e. 1.47 mm over the board edge, inside the case's USB knock-out);
  - silkscreen lines beyond the board edge dropped; our board edge drawn on Dwgs.User;
  - the project's XKB 3D model (3d/U262-161N-4BVC11.stp; nearly the same body), shifted to the 16PF front face.

Usage: gen_usbc_16pf015.py XKB_noSBU.kicad_mod OUT.kicad_mod
"""
import math
import re
import sys
import uuid

EDGE_Y = 2.175            # board edge in footprint coordinates (J2 origin to the board's front edge)
CU_EDGE_CLR = 0.3         # copper-to-edge clearance kept by the front tab pads
FRONT_TAB_Y = 1.045       # rear tabs at -3.105 + 4.15
FRONT_FACE_Y = FRONT_TAB_Y + 2.60
BODY_LEN, BODY_HALF_W = 7.55, 4.47
XKB_FACE_Y = 3.555        # front face of KiCad's XKB 3D model in these coordinates

src = open(sys.argv[1]).read()
u = lambda: str(uuid.uuid4())


def blocks(text, head):
    res, i = [], 0
    while True:
        i = text.find('\n\t' + head, i)
        if i < 0:
            return res
        depth, j = 0, text.index('(', i)
        while True:
            depth += text[j] == '('
            depth -= text[j] == ')'
            j += 1
            if depth == 0:
                break
        res.append(text[i:j])
        i = j


# front shell tabs -> custom pads cut at the board edge
cut = round(EDGE_Y - CU_EDGE_CLR - FRONT_TAB_Y, 4)
r, c = 0.5, 0.4                            # 1.8 = 2*0.4 + 2*0.5
t1 = math.asin(min(1.0, (cut - c) / r))
pts = [(r * math.cos(t1 * i / 10), c + r * math.sin(t1 * i / 10)) for i in range(11)]
pts += [(r * math.cos(math.pi - t1 + t1 * i / 10), c + r * math.sin(math.pi - t1 + t1 * i / 10)) for i in range(11)]
pts += [(r * math.cos(math.pi + math.pi * i / 18), -c + r * math.sin(math.pi + math.pi * i / 18)) for i in range(19)]
poly = ' '.join(f'(xy {x:.4f} {y:.4f})' for x, y in pts)
n = 0
for b in blocks(src, '(pad "SH"'):
    m = re.search(r'\(at ([-\d.]+) ([-\d.]+)\)', b)
    if float(m.group(2)) < 0:
        continue                           # rear tab: keep
    x = float(m.group(1))
    src = src.replace(b, f'''
	(pad "SH" thru_hole custom
		(at {x:g} {FRONT_TAB_Y:g})
		(size 0.6 0.6)
		(drill oval 0.6 1.4)
		(property pad_prop_mechanical)
		(layers "*.Cu" "*.Mask")
		(remove_unused_layers no)
		(options
			(clearance outline)
			(anchor circle)
		)
		(primitives
			(gr_poly
				(pts {poly})
				(width 0)
				(fill yes)
			)
		)
		(uuid "{u()}")
	)''', 1)
    n += 1
assert n == 2, n

# graphics: drop silkscreen beyond the board edge, redraw the fab body
for b in blocks(src, '(fp_line'):
    ys = [float(v) for v in re.findall(r'\((?:start|end) [-\d.]+ ([-\d.]+)\)', b)]
    if '(layer "F.SilkS")' in b and min(ys) > EDGE_Y - 0.2:
        src = src.replace(b, '', 1)
    elif '(layer "F.SilkS")' in b and max(ys) > EDGE_Y:
        src = src.replace(b, re.sub(r'(\((?:start|end) [-\d.]+ )([-\d.]+)\)',
                                    lambda q: q.group(1) + (f'{EDGE_Y - 0.2:g}' if float(q.group(2)) > EDGE_Y else q.group(2)) + ')', b), 1)
    elif '(layer "F.Fab")' in b:
        src = src.replace(b, '', 1)


def line(a, b, layer, w):
    return (f'\n\t(fp_line\n\t\t(start {a[0]:g} {a[1]:g})\n\t\t(end {b[0]:g} {b[1]:g})\n\t\t(stroke\n\t\t\t(width {w})\n'
            f'\t\t\t(type solid)\n\t\t)\n\t\t(layer "{layer}")\n\t\t(uuid "{u()}")\n\t)')


y0, y1, w = round(FRONT_FACE_Y - BODY_LEN, 3), round(FRONT_FACE_Y, 3), BODY_HALF_W
fab = [(-w, y0), (w, y0), (w, y1), (-w, y1)]
g = ''.join(line(a, b, 'F.Fab', 0.1) for a, b in zip(fab, fab[1:] + fab[:1]))
g += line((-5.6, EDGE_Y), (5.6, EDGE_Y), 'Dwgs.User', 0.1)
i = src.find('\n\t(pad ')
src = src[:i] + g + src[i:]

old = re.search(r'\(footprint "([^"]+)"', src).group(1)
src = src.replace(f'(footprint "{old}"', '(footprint "USB_C_16PF-015"', 1)
src = re.sub(r'\(descr "[^"]*"\)', '(descr "USB Type-C receptacle 16P, top-mount SMT, L 7.55 mm, Rui Chi 16PF-015 '
             '(drawing 1012-16F****01); XKB U262 land pattern, SBU pads A8/B8 omitted, front shell-tab pads cut '
             f'{CU_EDGE_CLR} mm before the board edge (hw/tools/gen_usbc_16pf015.py)")', src, count=1)
src = re.sub(r'\(property "Value" "[^"]*"', '(property "Value" "USB_C_16PF-015"', src, count=1)
# 3D: the project's XKB U262 STEP (as J2 used it on the board: offset (0, -3.6, 1.5), rotate (-90, 0, 0)),
# moved to the 16PF front face (model offsets are Y-up); length differs by 0.2 mm at the rear only
mdl = blocks(src, '(model ')
for m in mdl:
    src = src.replace(m, '', 1)
dy = -3.6 - (FRONT_FACE_Y - XKB_FACE_Y)
i = src.rfind('\n)')
src = src[:i] + f'''
	(model "${{KIPRJMOD}}/3d/U262-161N-4BVC11.stp"
		(offset
			(xyz 0 {dy:.3f} 1.5)
		)
		(scale
			(xyz 1 1 1)
		)
		(rotate
			(xyz -90 0 0)
		)
	)''' + src[i:]
open(sys.argv[2], 'w').write(src)
print('front tab copper cut at', EDGE_Y - CU_EDGE_CLR, 'slot ends at', FRONT_TAB_Y + 0.7,
      '-> ring', round(EDGE_Y - CU_EDGE_CLR - FRONT_TAB_Y - 0.7, 3), '; front face', FRONT_FACE_Y)
