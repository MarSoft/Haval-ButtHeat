"""ButtHeat v2 front case, SIDE parts (SLA): from the seam (x -+20) to the ends, which follow the car's
air-vent housings (R20 arcs around (-+96, 8.95), as v1's cover). Left: the cable exit in the arc
wall; right: J2 USB-C knock-out.

Don't run directly: v2_left.py / v2_right.py set SIDE and exec this (headless), writing hw/cad/out/Left.brep
or Right.brep. Needs hw/cad/v2_middle.py (re-used for the seam geometry; also refreshes Middle.brep).

Both parts are built in LEFT coordinates (x < 0); the right part's features are given mirrored (x -> -x)
and the result is mirrored back at the end, so the shoulder, seam socket, ear, posts etc. are identical.

Case coordinates: z 0 = car console surface, rear plate z 0..2 (lip to z 6 within |x| <= 76), board top z 3.5,
board side edges x -+73.
"""
import math
import FreeCAD as App
import Part

OUT = '/home/mars/my/buttheat/hw/cad/out/'
P = App.Vector

mid = {}
exec(open('/home/mars/my/buttheat/hw/cad/v2_middle.py').read(), mid)   # also refreshes Middle.brep

X_MAIN, X_SEAM = -76.0, -20.0   # main body left end (y < 8.95); seam
EAR_C, EAR_R = (-96.0, 8.95), 20.0   # car vent housing: the ear above it follows this arc (plate ear too)
WALL = 1.5
PLATE_T = 2.0
TOP = 12.4                      # side parts' outer top (v1)
CEIL = TOP - WALL               # 10.9
FRONT_R, FRONT_V = 6.325, 6.075 # v1 front rounding: vertical to z 6.075, then R6.325 to the top
FRAME_Y0, TAN15, TAPE = 29.75, math.tan(math.radians(15)), 0.2
LIP_Y0, LIP_TOP = 28.3, 6.0     # rear plate lip (x >= -76): the part's back edge sits on it
# rear plate lip top along x (real coordinates; mirrors v2_rear_plate.plate_body): 45 deg end chamfers from
# z 0 at |x| 76, z 5.0 under the middle part (|x| <= 23, 45 deg flanks), z 3.3 under J1 (left only, flanks)
LIP_END, LIP_MID, LIP_J1 = 76.0, (23.0, 5.0), (-74.3, -65.2, 3.3)


def lip_z(x):
    z = min(LIP_TOP, max(0.0, LIP_END - abs(x)))
    m, mz = LIP_MID
    z = min(z, mz + max(0.0, abs(x) - m))
    if SIDE == 'left':
        j0, j1, jz = LIP_J1
        z = min(z, jz + max(0.0, j0 - x, x - j1))
    return z
FIT = 0.15                      # SLA sliding fit at the seam

# shoulder (v1 style): from the middle part's top at the seam up to full height. Ruled surface between
# the middle's seam cross-section (bevel 5.0 at the front edge -> 6.5 from y 16.05) and z SHOULDER_Z2
# at SHOULDER_X2: ~22 deg, passes z 12.4 at x ~ -34.5 (v1: 18 deg, -36.7; steeper here because the v2
# seam is 0.5 lower and the THT switch top must stay under the skin)
SHOULDER_X2, SHOULDER_Z2 = -36.0, 13.0
ROOF_DZ_IN = 1.6                # cavity roof: WALL measured normal to the ~19 deg slope

BUSHING_HOLE = 7.4
PLATE_POST_TOP, PLATE_POST_D = 6.0, 6.0              # rear plate spacer tubes (v2_rear_plate POST_TOP, POST_D)
FRONT_POST_D, FRONT_POST_Z0 = 7.6, 6.1               # front posts continue down to the plate posts
SCREW_CLR, CSK_D, HEAD_Z = 2.9, 5.0, 8.5             # M2.5x8 DIN 965 (head 4.7, 90 deg), head top at z 8.5
                                                     # (tip at z 0.5: >= 0.5 above the car panel; FDM insert
                                                     # 3.5 mm, SLA nut 1.9 mm of thread)
CBORE_D = 6.0                                        # access counterbore: 0.65 mm headroom around the head
RIM_CHAMFER = 0.5                                    # chamfer on the counterbore rims
FRONT_FILLET = 1.0                                   # shoulder slope / knob-arch front wall edge (as the middle part)
KNOCKOUT_D, KNOCKOUT_WEB = 4.0, 0.4
H_BOSS = dict(boss_d=5.0, pilot_d=1.7, top=7.6)     # H1/H2 fallback M2x4 self-tapping from the board's back
# J1: JST ZH side-entry S4B-ZR (2026-11-19; was the vertical B4B with a raised "bump" roof for the wires'
# U-turn): body x -73.55..-66.05, y 22.2..28.2, z 4.6..8.3 (1 mm spacer under it), plug entering from -y;
# the wires curl over U1 inside the normal ceiling (z 10.9)
J1_CLR = (-74.5, -65.0, 24.9, 29.1, 3.3, 9.0)        # x0, x1, y0, y1, z0, z1 (clearance at the back wall)
exec(open('/home/mars/my/buttheat/hw/cad/v2_cable.py').read())   # cable seat, exit paths, insert, M2 tab

_doc = (App.getDocument('ButtHeatModel') if 'ButtHeatModel' in App.listDocuments()
        else App.openDocument('/home/mars/tmp/3dp/buttheat/ButtHeatModel.FCStd'))
_v = lambda n: float(getattr(_doc.getObject('V2Vars'), n).Value)

# per-side features, in LEFT coordinates (right side: x mirrored). Board parts from the board STEP.
#   enc: encoder shaft axes (bushing holes); posts; sw: 6x6 tact switch (DNP by default), body box + a second
#   tier for the THT cover features (z 7.6); h: fallback M2 hole; usb: knock-out x0, x1, z0, z1;
#   keepouts: boxes x0, x1, y0, y1, z top cut from z 3 (encoder bodies next to front posts, D2 under the shoulder)
CFG = {
    'left': dict(
        enc=[(-43.0, 11.0)],                                         # ENC1
        posts=[(-_v('post1_left'), _v('post1_y')), (-_v('post3_left'), _v('post3_y'))],   # posts 1, 3 (V2Vars)
        sw=dict(x=-27.0, y=19.0, body=6.0, body_top=7.1, clr=0.2, top=(-29.6, -24.4, 16.3, 21.7, 7.8)),
        h=(-27.0, 13.0),                                             # H1
        usb=(-66.0, -54.5, 2.0, 9.0),                                # SuperMini USB-C (x -64.7..-55.8, z 3.6..7.7) + plug
        keepouts=[(-49.5, -36.5, 4.05, 17.95, 9.8)]),                # ENC1 body (post 3's front post)
    'right': dict(
        enc=[(-43.0, 11.0), (-65.0, 17.0)],                          # ENC2 (43, 11), ENC3 (65, 17)
        posts=[(-_v('post2_x'), _v('post2_y')), (-_v('post4_x'), _v('post4_y'))],        # posts 2, 4 (V2Vars; post 2
                                                                     # moved 2026-11-17 from (78.5, 24): its counterbore
                                                                     # was 0.1 from the ear's arc face)
        sw=dict(x=-27.0, y=19.0, body=6.0, body_top=7.1, clr=0.2, top=(-29.8, -24.2, 16.3, 21.7, 7.8)),
        h=(-27.0, 13.0),                                             # H2
        usb=(-74.0, -61.6, 2.0, 8.4),                                # J2 USB-C mouth x 63.35..72.25, z 3.39..6.6 + plug overmold
        keepouts=[(-49.95, -36.05, 4.5, 17.5, 9.8),                  # ENC2 body
                  (-71.5, -58.5, 10.05, 23.95, 9.8),                 # ENC3 body
                  (-25.8, -20.4, 24.65, 27.75, 5.91)]),              # D2 (SMA, top 5.71) under the shoulder
}[SIDE]
POSTS, SW = CFG['posts'], CFG['sw']


def frame_y(z, inset=0.0):
    return FRAME_Y0 - TAPE - inset / math.cos(math.radians(15)) + z * TAN15


def profile(t, z_bottom):
    """Cross-section (y, z) of the side part shrunk by t (outer: t=0), no lip notch (cut separately)."""
    top = TOP - t
    r = FRONT_R - t
    c = (FRONT_R, FRONT_V)
    am = math.radians(135)
    arc = Part.Arc(P(0, t, FRONT_V), P(0, c[0] + r * math.cos(am), c[1] + r * math.sin(am)), P(0, FRONT_R, top))
    poly = [P(0, FRONT_R, top), P(0, frame_y(top, t), top), P(0, frame_y(z_bottom, t), z_bottom), P(0, t, z_bottom)]
    edges = [Part.LineSegment(P(0, t, z_bottom), P(0, t, FRONT_V)).toShape(), arc.toShape()]
    edges += [Part.LineSegment(a, b).toShape() for a, b in zip(poly, poly[1:])]
    return Part.Face(Part.Wire(Part.__sortEdges__(edges)))


def arcc(c, r, a0, a1):
    pt = lambda a: P(c[0] + r * math.cos(a), c[1] + r * math.sin(a), 0)
    return Part.Arc(pt(a0), pt((a0 + a1) / 2), pt(a1)).toShape()


def plan(t, x1):
    """Plan-view outline shrunk by t: ear arc R20+t, main left end x -76+t (y < 8.95), front line far
    out (the profile limits y), knob arch R29.75-t, the seam-side line, x <= x1.
    Built explicitly: makeOffset2D gives offset curves that crash OCCT's booleans here."""
    kc, kR = (-40.16528, 29.75), 29.75 - t
    ka0, ka1 = -math.pi / 2, math.atan2(6.79578 - kc[1], -21.24 - kc[0])
    kp = lambda a: P(kc[0] + kR * math.cos(a), kc[1] + kR * math.sin(a), 0)
    er = EAR_R + t
    xm = EAR_C[0] + er
    ys = 8.60 + t * 0.567 + (0.5 if t else 0)
    xs = -20.0 - t * 0.824
    xl = EAR_C[0] + t                                 # ear tip: x -96 (as the plate ear)
    a_top = math.acos(t / er)
    e = [Part.LineSegment(P(xl, 60, 0), P(xl, EAR_C[1] + er * math.sin(a_top), 0)).toShape(),
         arcc(EAR_C, er, a_top, 0.0),
         Part.LineSegment(P(xm, EAR_C[1], 0), P(xm, -5, 0)).toShape(),
         Part.LineSegment(P(xm, -5, 0), P(kc[0], -5, 0)).toShape(),
         Part.LineSegment(P(kc[0], -5, 0), kp(ka0)).toShape(),
         Part.Arc(kp(ka0), kp((ka0 + ka1) / 2), kp(ka1)).toShape(),
         Part.LineSegment(kp(ka1), P(xs, ys, 0)).toShape(),
         Part.LineSegment(P(xs, ys, 0), P(-5, ys, 0)).toShape(),
         Part.LineSegment(P(-5, ys, 0), P(-5, 60, 0)).toShape(),
         Part.LineSegment(P(-5, 60, 0), P(xl, 60, 0)).toShape()]
    f = Part.Face(Part.Wire(Part.__sortEdges__(e)))
    box = Part.Face(Part.makePolygon([P(-130, -6, 0), P(x1, -6, 0), P(x1, 61, 0), P(-130, 61, 0), P(-130, -6, 0)]))
    return f.common(box)


def prism(face_yz, x0, x1):
    s = face_yz.extrude(P(x1 - x0, 0, 0)); s.translate(P(x0, 0, 0)); return s


def column(face_xy, z0, z1):
    s = face_xy.extrude(P(0, 0, z1 - z0)); s.translate(P(0, 0, z0)); return s


def roof(dz):
    """Solid below the shoulder surface, lowered by dz. Ruled loft over x: flat seam section continued to
    x -14, the ramp to SHOULDER_X2, then straight up out of the way."""
    def sec(x, zs):
        ys = [-6, 8.6, mid['GLASS_Y0'], 61]
        pts = [P(x, y, z - dz) for y, z in zip(ys, zs)] + [P(x, 61, -5), P(x, -6, -5)]
        return Part.makePolygon(pts + [pts[0]])

    z_seam = [5.0, 5.0, mid['TOP'], mid['TOP']]
    secs = [sec(-14, z_seam), sec(X_SEAM, z_seam), sec(SHOULDER_X2, [SHOULDER_Z2] * 4),
            sec(SHOULDER_X2 - 1.5, [40] * 4), sec(-130, [40] * 4)]
    return Part.makeLoft(secs, True, True)


step = lambda s: print('step:', s, flush=True)

# --- body ---------------------------------------------------------------------------------------
# prisms are always LONGER than needed and only the plan clip sets the x extents: any exact coincidence
# between prism ends and clip edges/vertices makes OCCT's classifier segfault
outer = prism(profile(0, PLATE_T), -125, X_SEAM + 3).common(column(plan(0, X_SEAM), -1, 30)).common(roof(0))
step('outer')
cav = prism(profile(WALL, PLATE_T - 3), -128, X_SEAM + 8).common(column(plan(WALL, X_SEAM + 5), -5, 30))
cav = cav.common(roof(ROOF_DZ_IN))
step('cav')
# lip notch: the back edge sits on the rear plate's lip, following its profile along x (end chamfer, J1
# drop, middle zone); the cavity stays WALL above it
def lip_profile(off):
    sgn = 1 if SIDE == 'left' else -1                  # left coordinates -> real x
    xs = [-82.0 + 0.05 * i for i in range(int((82.0 - 10.0) / 0.05) + 1)]
    pts = [(x, lip_z(sgn * x) + off) for x in xs]
    keep = [pts[0]] + [b for a, b, c in zip(pts, pts[1:], pts[2:])
                       if abs((b[1] - a[1]) * (c[0] - b[0]) - (c[1] - b[1]) * (b[0] - a[0])) > 1e-9] + [pts[-1]]
    poly = [P(x, LIP_Y0, z) for x, z in keep] + [P(keep[-1][0], LIP_Y0, -5), P(keep[0][0], LIP_Y0, -5)]
    f = Part.Face(Part.makePolygon(poly + [poly[0]]))
    return f.extrude(P(0, 32, 0))
outer = outer.cut(lip_profile(0.0))
cav = cav.cut(lip_profile(WALL))


def front_edges(shape):
    """Edges where the shoulder's front slope (a plane in front of y 8.6) meets the vertical knob-arch front
    wall (seam .. x ~-30.5). (Continuing the fillet up along the front rounding only works together with the
    slope's own crease at y 8.6, so it's left sharp.)"""
    out = []
    for e in shape.Edges:
        if e.BoundBox.XLength < 0.1 or e.BoundBox.YLength < 0.01:   # (y = 8.6 is the slope's own crease)
            continue
        try:
            pts = e.discretize(8)
        except Exception:
            continue
        if not all(p.y < 8.65 and -40 < p.x < -19.99 and abs(p.z - (5.0 + 0.5 * (-20.0 - p.x))) < 0.02 for p in pts):
            continue
        if any((f.Surface.__class__.__name__ == 'Cylinder' and abs(f.Surface.Axis.z) > 0.999)
               or (f.Surface.__class__.__name__ == 'Plane' and abs(f.Surface.Axis.z) < 0.001)
               for f in shape.ancestorsOfType(e, Part.Face)):
            out.append(e)
    return out


# rounded front edge like the middle part's, filleted on the plain envelope (later cuts, e.g. post 3's
# counterbore, run into that edge and make OCC fillets on the finished part fail)
es = front_edges(outer)
outer = outer.makeFillet(FRONT_FILLET, es)
assert outer.isValid(), 'front fillet failed'
print('front fillet on', len(es), 'edges')
step('notch')
part = outer.cut(cav)
step('shell')

# --- seam: socket for the middle part's spigot, under this part's skin -------------------------------
# solid wedge between the sloped skin and the spigot (the middle part's envelope shrunk by WALL/2), so the
# screwed-down side parts hold the middle part down
sx = X_SEAM - mid['SPIGOT'] - FIT
zb = mid['BOARD_TOP'] + 0.05
part = part.fuse(outer.common(Part.makeBox(X_SEAM + 1 - (sx + 0.2), 70, 30, P(sx + 0.2, -6, zb))))
part = part.cut(mid['envelope'](WALL / 2 - FIT, sx, X_SEAM + 1, mid['BOARD_TOP'] - 1, mid['LIP_FRONT'] + WALL / 2 + FIT))
step('socket')
# board corner at the seam (board arch edge (-22.51, 8.33) -> (-18.93, 8.71)): no wall below the board top there
seam_board = Part.Face(Part.makePolygon([P(-24.5, 8.33 - 0.15, 0), P(-22.51, 8.33 - 0.15, 0), P(-18.93, 8.71 - 0.15, 0),
                                         P(-18.93, 30, 0), P(-24.5, 30, 0), P(-24.5, 8.33 - 0.15, 0)]))
part = part.cut(column(seam_board, -1, mid['BOARD_TOP'] + 0.05))

# --- clearances: rear-plate posts, J1 + its plug, the button body under the shoulder ---------------
for (x, y) in POSTS:
    part = part.cut(Part.makeCylinder(PLATE_POST_D / 2 + 0.1, PLATE_POST_TOP + 0.05 + 1, P(x, y, -1)))
    part = part.cut(Part.makeCylinder(10.4 / 2, 2.45 + 0.2 + 1, P(x, y, -1)))   # the plate's D10 boss under the PCB
if SIDE == 'left':
    jx0, jx1, jy0, jy1, jz0, jz1 = J1_CLR
    part = part.cut(Part.makeBox(jx1 - jx0, jy1 - jy0, jz1 - jz0, P(jx0, jy0, jz0)))
h = SW['body'] / 2 + SW['clr']
part = part.cut(Part.makeBox(2 * h, 2 * h, SW['body_top'] + SW['clr'] - 3.0, P(SW['x'] - h, SW['y'] - h, 3.0)))
x0, x1, y0, y1, zt = SW['top']
part = part.cut(Part.makeBox(x1 - x0, y1 - y0, zt - 3.0, P(x0, y0, 3.0)))
step('clearances')

# --- posts: the front part's post meets the short plate post; M2.5x8 CSK sunk to z 9.5 --------------
for (x, y) in POSTS:
    part = part.fuse(Part.makeCylinder(FRONT_POST_D / 2, 20, P(x, y, FRONT_POST_Z0)).common(outer))
for x0, x1, y0, y1, zt in CFG['keepouts']:
    part = part.cut(Part.makeBox(x1 - x0, y1 - y0, zt - 3.0, P(x0, y0, 3.0)))
for (x, y) in POSTS:
    part = part.cut(Part.makeCylinder(SCREW_CLR / 2, 30, P(x, y, -1)))
    depth = (CSK_D - SCREW_CLR) / 2
    part = part.cut(Part.makeCone(SCREW_CLR / 2, CSK_D / 2, depth, P(x, y, HEAD_Z - depth)))
    part = part.cut(Part.makeCylinder(CBORE_D / 2, 10, P(x, y, HEAD_Z - 0.01)))
step('posts')

# --- H1/H2: boss for the fallback M2 screw from the board's back -------------------------------------
hx, hy = CFG['h']
part = part.fuse(Part.makeCylinder(H_BOSS['boss_d'] / 2, 15, P(hx, hy, mid['BOARD_TOP'])).common(outer))
part = part.cut(Part.makeCylinder(H_BOSS['pilot_d'] / 2, H_BOSS['top'] - 2, P(hx, hy, 2.0)))
step('H boss')

# --- encoder bushings, button knock-out, USB knock-out -------------------------------------------------
for x, y in CFG['enc']:
    part = part.cut(Part.makeCylinder(BUSHING_HOLE / 2, 20, P(x, y, 0)))
# knock-outs are cut from the inside and leave a KNOCKOUT_WEB skin parallel to the outer surface
web_in = roof(KNOCKOUT_WEB * 1.06)
part = part.cut(Part.makeCylinder(KNOCKOUT_D / 2, 20, P(SW['x'], SW['y'], 0)).common(web_in))
ux0, ux1, uz0, uz1 = CFG['usb']
# USBs are mutually exclusive (SuperMini on the left, J2 on the right)
usb = Part.makeBox(ux1 - ux0, 8, uz1 - uz0, P(ux0, -1, uz0)).common(prism(profile(KNOCKOUT_WEB, PLATE_T - 3), ux0 - 1, ux1 + 1))
part = part.cut(usb)
step('knock-outs')

# --- cable: seat for the split clamp insert, exit paths, slot from below ----------------------------
def path_cut(shape, pts, r, slot_w):
    """Cut a round channel (radius r) along the polyline pts and a slot_w wide slot below it to z -1."""
    pts = [P(*p) for p in pts]
    for p0, p1 in zip(pts, pts[1:]):
        d = p1 - p0
        shape = shape.cut(Part.makeCylinder(r, d.Length, p0, d))
        h = P(d.x, d.y, 0); h.normalize()
        n = P(-h.y, h.x, 0) * (slot_w / 2)
        q = [p0 - n, p1 - n, P(p1.x, p1.y, -1) - P(n.x, n.y, 0), P(p0.x, p0.y, -1) - P(n.x, n.y, 0)]
        q = [P(v.x, v.y, z) for v, z in zip(q, (p0.z, p1.z, -1, -1))]
        shape = shape.cut(Part.Face(Part.makePolygon(q + [q[0]])).extrude(n * 2))
    for p in pts[1:-1]:
        shape = shape.cut(Part.makeSphere(r, p))
    return shape


if SIDE == 'left':
    sx0, sx1, sy, sz = SEAT['x0'], SEAT['x1'], SEAT['y'], SEAT['z']
    boss = Part.makeCylinder(SEAT_BOSS_D / 2, sx1 - sx0 + 12, P(sx0 - 12, sy, sz), P(1, 0, 0))
    part = part.fuse(boss.common(outer))
    part = path_cut(part, [(sx1 + 0.5, sy, sz), (sx0, sy, sz)], SEAT['d'] / 2, SLOT_W)
    part = part.cut(Part.makeSphere(EXIT_D / 2, P(sx0, sy, sz)))   # junction of the exit paths
    for pts in CABLE_PATHS:
        part = path_cut(part, [(sx0 + 0.5, sy, sz)] + pts[1:], EXIT_D / 2, SLOT_W)
    # M2 boss hanging from the ceiling (kept out of the seat), then the pocket for the insert's tab (and the web
    # joining it to the bottom half), open towards J1 so the insert slides in; the pocket's top is the boss bottom
    part = part.fuse(Part.makeCylinder(M2['boss_d'] / 2, 20, P(M2['x'], M2['y'], TAB['z0']))
                     .common(outer).common(Part.makeBox(20, 20, 30, P(M2['x'] - 10, SEAT['y'] - SEAT['d'] / 2 - 0.05 - 20, 0))))
    t = TAB_CLR
    part = part.cut(Part.makeBox(-70.0 - (TAB['x0'] - t), SEAT['y'] - 2.3 - (TAB['y0'] - t), TAB['z1'] - (TAB['z0'] - t),
                                 P(TAB['x0'] - t, TAB['y0'] - t, TAB['z0'] - t)))
    part = part.cut(Part.makeCylinder(M2['pilot_d'] / 2, M2['depth'] + 0.1, P(M2['x'], M2['y'], TAB['z1'] - 0.1)))
    part = part.cut(Part.makeCylinder(M2_HEAD_D / 2, TAB['z0'] - t + 1 + 0.05, P(M2['x'], M2['y'], -1)))   # head + driver from below
    # fin left between the seat's slot and the lip notch (y 27.7..28.3, x -79..-74.5, below the lip top)
    part = part.cut(Part.makeBox(4.7, 0.9, 5.4, P(-79.1, 27.5, 0.9)))
    step('cable')

part = part.removeSplitter()


# --- edge finishing (on the final solid) ----------------------------------------------------------------
def faces_of(e):
    return part.ancestorsOfType(e, Part.Face)


def samples(e, n=8):
    try:
        return e.discretize(n)
    except Exception:                                  # degenerate edges
        return None


def rim_edges():
    """Counterbore rims: edges on the counterbore cylinders above the head seat."""
    out = []
    for e in part.Edges:
        pts = samples(e)
        if not pts or min(p.z for p in pts) < HEAD_Z + 0.05:
            continue
        if any(all(abs(math.hypot(p.x - x, p.y - y) - CBORE_D / 2) < 0.01 for p in pts) for x, y in POSTS):
            out.append(e)
    return out


# rim chamfers, post by post; where the rim comes close to the head seat (post 4 at the front rounding) the
# chamfer is reduced until OCC can build it
for x, y in POSTS:
    es = [e for e in rim_edges() if all(abs(math.hypot(p.x - x, p.y - y) - CBORE_D / 2) < 0.01 for p in samples(e))]
    for c in (RIM_CHAMFER, 0.4, 0.3, 0.2):
        try:
            r = part.makeChamfer(c, es)
            if r.isValid():
                part = r
                print('post', (x, y), 'rim chamfer', c, 'on', len(es), 'edges')
                break
        except Exception:
            pass
    else:
        print('RIM CHAMFER FAILED at post', (x, y))
step('edges')
if SIDE == 'right':
    part = part.mirror(P(0, 0, 0), P(1, 0, 0))
step('done')
name = SIDE.capitalize()
part.exportBrep(OUT + name + '.brep')
print(name, part.isValid(), round(part.Volume, 1), part.optimalBoundingBox())
