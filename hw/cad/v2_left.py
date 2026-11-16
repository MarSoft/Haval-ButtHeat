"""ButtHeat v2 front case, LEFT part (SLA): from the seam (x -20) to the left end, which follows the car's
air-vent housing (R20 arc around (-96, 8.95), as v1's cover), with the cable exit in that arc wall.

Run headless; writes hw/cad/out/Left.brep. Needs hw/cad/v2_middle.py (re-used for the seam geometry;
running this also refreshes Middle.brep).

Case coordinates: z 0 = car console surface, rear plate z 0..2 (lip to z 6 from x -76), board top z 3.5,
board left edge x -73.
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
LIP_X = -76.5                   # left of this there is no lip: the back wall goes down to the plate
FIT = 0.15                      # SLA sliding fit at the seam

# shoulder (v1 style): from the middle part's top at the seam up to full height. Ruled surface between
# the middle's seam cross-section (bevel 5.0 at the front edge -> 6.5 from y 16.05) and z SHOULDER_Z2
# at SHOULDER_X2: ~22 deg, passes z 12.4 at x ~ -34.5 (v1: 18 deg, -36.7; steeper here because the v2
# seam is 0.5 lower and the THT switch top must stay under the skin)
SHOULDER_X2, SHOULDER_Z2 = -36.0, 13.0
ROOF_DZ_IN = 1.6                # cavity roof: WALL measured normal to the ~19 deg slope

ENC1 = (-43.0, 11.0); BUSHING_HOLE = 7.4
POSTS = [(-48.0, 26.0), (-33.0, 5.0)]
PLATE_POST_TOP, PLATE_POST_D = 6.0, 6.6              # rear plate spacer tubes (v2_rear_plate.POST_TOP)
FRONT_POST_D, FRONT_POST_Z0 = 8.0, 6.1               # front posts continue down to the plate posts
SCREW_CLR, CSK_D, HEAD_Z = 3.4, 6.3, 9.0             # M3x8 DIN 7991 (head 6.0, 90 deg), head top at z 9
CBORE_D = 6.6                                        # access counterbore above the head
SW = dict(x=-27.0, y=19.0, body=6.0, body_top=7.1, clr=0.2,     # 6x6 tact switch (SW1/SW2), DNP by default
          top=(-29.6, -24.4, 16.3, 21.7, 7.8))                 # THT cover features up to z 7.6: second tier
ENC1_BODY = (-49.5, -36.5, 4.05, 17.95, 9.8)                     # x0, x1, y0, y1, z top (+0.25 clearance)
KNOCKOUT_D, KNOCKOUT_WEB = 4.0, 0.4
H1 = dict(x=-27.0, y=13.0, boss_d=5.0, pilot_d=1.7, top=7.6)   # fallback M2x5 thread-forming from the back
USB_OPEN = (-66.0, -54.5, 2.0, 9.0)                  # SuperMini USB-C (x -64.7..-55.8, z 3.6..7.7) + plug
J1_BUMP = (-75.0, -62.0, 21.0, 14.5)                 # x0, x1, y0, outer top: room for the wires' U-turn
J1_CLR = (-74.5, -65.0, 24.9, 29.1, 3.3)             # x0, x1, y0, y1, z0 (ZH header y 25.09..28.9, plug top 9.4)
CABLE = dict(y=25.2, z=7.0, seat_x=(-79.0, -74.6),   # seat for the replaceable split clamp insert (OD 8.6)
             seat_d=8.8, boss_d=11.0, exit_d=6.5)     # exit hole in the arc wall; its rim takes the pull


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
    x -14, the ramp to SHOULDER_X2, then straight up out of the way (the J1 bump is higher than TOP)."""
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
# lip notch (x >= LIP_X): the back edge sits on the rear plate's lip; the cavity stays WALL above it
notch = lambda z, x0: prism(Part.Face(Part.makePolygon([P(0, LIP_Y0, -5), P(0, 60, -5), P(0, 60, z), P(0, LIP_Y0, z),
                                                        P(0, LIP_Y0, -5)])), x0, X_SEAM + 10)
outer = outer.cut(notch(LIP_TOP, LIP_X))
cav = cav.cut(notch(LIP_TOP + WALL, LIP_X + WALL))
step('notch')
# J1 bump: raised roof over the connector for the wires' U-turn
bx0, bx1, by0, btop = J1_BUMP
bump_face = Part.Face(Part.makePolygon([P(0, by0, TOP - 1), P(0, frame_y(TOP - 1), TOP - 1), P(0, frame_y(btop), btop),
                                        P(0, by0 + (btop - TOP + 1), btop), P(0, by0, TOP - 1)]))
outer = outer.fuse(prism(bump_face, bx0, bx1))
cav = cav.fuse(prism(Part.Face(Part.makePolygon([P(0, by0 + WALL, CEIL - 1), P(0, frame_y(btop - WALL, WALL), CEIL - 1),
                                                 P(0, frame_y(btop - WALL, WALL), btop - WALL),
                                                 P(0, by0 + WALL + (btop - TOP + 1), btop - WALL), P(0, by0 + WALL, CEIL - 1)])),
                     bx0 + WALL, bx1 - WALL))
step('bump')
part = outer.cut(cav)
step('shell')

# --- seam: tongue into the middle part's rebate ------------------------------------------------
part = part.fuse(mid['tongue'](-1, FIT))
step('tongue')
# board corner at the seam (board arch edge (-22.51, 8.33) -> (-18.93, 8.71)): no wall below the board top there
seam_board = Part.Face(Part.makePolygon([P(-24.5, 8.33 - 0.15, 0), P(-22.51, 8.33 - 0.15, 0), P(-18.93, 8.71 - 0.15, 0),
                                         P(-18.93, 30, 0), P(-24.5, 30, 0), P(-24.5, 8.33 - 0.15, 0)]))
part = part.cut(column(seam_board, -1, mid['BOARD_TOP'] + 0.05))

# --- clearances: rear-plate posts, J1 + its plug, the button body under the shoulder ---------------
for (x, y) in POSTS:
    part = part.cut(Part.makeCylinder(PLATE_POST_D / 2 + 0.1, PLATE_POST_TOP + 0.05 + 1, P(x, y, -1)))
    part = part.cut(Part.makeCylinder(10.4 / 2, 2.45 + 0.2 + 1, P(x, y, -1)))   # the plate's D10 boss under the PCB
jx0, jx1, jy0, jy1, jz0 = J1_CLR
part = part.cut(Part.makeBox(jx1 - jx0, jy1 - jy0, btop - WALL - jz0, P(jx0, jy0, jz0)))
h = SW['body'] / 2 + SW['clr']
part = part.cut(Part.makeBox(2 * h, 2 * h, SW['body_top'] + SW['clr'] - 3.0, P(SW['x'] - h, SW['y'] - h, 3.0)))
x0, x1, y0, y1, zt = SW['top']
part = part.cut(Part.makeBox(x1 - x0, y1 - y0, zt - 3.0, P(x0, y0, 3.0)))
step('clearances')

# --- posts: the front part's post meets the short plate post; M3x8 CSK sunk to z 9 ------------------
for (x, y) in POSTS:
    part = part.fuse(Part.makeCylinder(FRONT_POST_D / 2, 20, P(x, y, FRONT_POST_Z0)).common(outer))
x0, x1, y0, y1, zt = ENC1_BODY                       # post 3's front post passes next to ENC1's body
part = part.cut(Part.makeBox(x1 - x0, y1 - y0, zt - 3.0, P(x0, y0, 3.0)))
for (x, y) in POSTS:
    part = part.cut(Part.makeCylinder(SCREW_CLR / 2, 30, P(x, y, -1)))
    depth = (CSK_D - SCREW_CLR) / 2
    part = part.cut(Part.makeCone(SCREW_CLR / 2, CSK_D / 2, depth, P(x, y, HEAD_Z - depth)))
    part = part.cut(Part.makeCylinder(CBORE_D / 2, 10, P(x, y, HEAD_Z - 0.01)))
step('posts')

# --- H1: boss for the fallback M2 screw from the board's back ----------------------------------------
part = part.fuse(Part.makeCylinder(H1['boss_d'] / 2, 15, P(H1['x'], H1['y'], mid['BOARD_TOP'])).common(outer))
part = part.cut(Part.makeCylinder(H1['pilot_d'] / 2, H1['top'] - 2, P(H1['x'], H1['y'], 2.0)))
step('H1')

# --- encoder bushing, button knock-out, SuperMini USB knock-out --------------------------------------
x, y = ENC1
part = part.cut(Part.makeCylinder(BUSHING_HOLE / 2, 20, P(x, y, 0)))
# knock-outs are cut from the inside and leave a KNOCKOUT_WEB skin parallel to the outer surface
web_in = roof(KNOCKOUT_WEB * 1.06)
part = part.cut(Part.makeCylinder(KNOCKOUT_D / 2, 20, P(SW['x'], SW['y'], 0)).common(web_in))
ux0, ux1, uz0, uz1 = USB_OPEN
# USBs are mutually exclusive (SuperMini here, J2 on the right)
usb = Part.makeBox(ux1 - ux0, 8, uz1 - uz0, P(ux0, -1, uz0)).common(prism(profile(KNOCKOUT_WEB, PLATE_T - 3), ux0 - 1, ux1 + 1))
part = part.cut(usb)
step('knock-outs')

# --- cable: seat for the split clamp insert, exit through the ear's arc wall -------------------------
c = CABLE
sx0, sx1 = c['seat_x']
boss = Part.makeCylinder(c['boss_d'] / 2, sx1 - sx0 + 12, P(sx0 - 12, c['y'], c['z']), P(1, 0, 0))
part = part.fuse(boss.common(outer))
part = part.cut(Part.makeCylinder(c['seat_d'] / 2, sx1 - sx0 + 0.5, P(sx0, c['y'], c['z']), P(1, 0, 0)))
part = part.cut(Part.makeCylinder(c['exit_d'] / 2, 20, P(sx0 - 19.99, c['y'], c['z']), P(1, 0, 0)))
step('cable')

part = part.removeSplitter()
step('done')
part.exportBrep(OUT + 'Left.brep')
print('Left', part.isValid(), round(part.Volume, 1), part.BoundBox)
