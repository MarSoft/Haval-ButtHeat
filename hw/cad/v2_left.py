"""ButtHeat v2 front case, LEFT part (SLA): x -76 .. -20 (seam), plus the cable-clamp boss at the left end.

Run headless; writes hw/cad/out/Left.brep. Needs hw/cad/v2_middle.py (re-used for the seam geometry).

Case coordinates: z 0 = car console surface, rear plate z 0..2 (lip to z 6 in the side zones), board top z 3.5.
"""
import math
import FreeCAD as App
import Part

OUT = '/home/mars/my/buttheat/hw/cad/out/'
P = App.Vector

mid = {}
exec(open('/home/mars/my/buttheat/hw/cad/v2_middle.py').read(), mid)   # also refreshes Middle.brep

X_END, X_SEAM = -76.0, -20.0
WALL = 1.5
PLATE_T = 2.0
TOP = 12.4                      # side parts' outer top (v1)
CEIL = TOP - WALL               # 10.9
FRONT_R, FRONT_V = 6.325, 6.075 # v1 front rounding: vertical to z 6.075, then R6.325 to the top
FRAME_Y0, TAN15, TAPE = 29.75, math.tan(math.radians(15)), 0.2
LIP_Y0, LIP_TOP = 28.3, 6.0
FIT = 0.15                      # SLA sliding fit at the seam

ENC1 = (-43.0, 11.0); BUSHING_HOLE = 7.4
POSTS = [(-48.0, 26.0), (-33.0, 5.0)]
POST_BOSS_D, POST_BOSS_BOTTOM = 9.0, TOP - 2.0      # plate posts end at z 10.4
SCREW_CLR, CSK_D = 3.4, 6.3                          # M3 DIN 7991: head 6.0, 90 deg
BUTTONS = [(-27.0, 19.0)]; KNOCKOUT_D, KNOCKOUT_WEB = 4.0, 0.4
USB_OPEN = (-66.0, -54.5, 2.0, 9.0)                  # SuperMini USB-C (x -64.7..-55.8, z 3.6..7.7) + plug
J1_BUMP = (-76.0, -62.0, 21.0, 14.5)                 # x0, x1, y0, outer top
CABLE = dict(y=25.2,   # boss (D12) must stay inside the 15 deg frame plane at all heights
              z=8.0, x_out=-81.0, boss_d=12.0, seat_d=9.2, exit_d=6.5, end_wall=1.0)


def frame_y(z, inset=0.0):
    return FRAME_Y0 - TAPE - inset / math.cos(math.radians(15)) + z * TAN15


def profile(t, z_bottom):
    """Cross-section (y, z) of the side part shrunk by t (outer: t=0)."""
    top = TOP - t
    r = FRONT_R - t
    pts = [P(0, t, z_bottom), P(0, t, FRONT_V)]
    # arc centre (FRONT_R, FRONT_V), from (t, FRONT_V) to (FRONT_R, top)
    c = (FRONT_R, FRONT_V)
    am = math.radians(135)
    arc = Part.Arc(P(0, t, FRONT_V), P(0, c[0] + r * math.cos(am), c[1] + r * math.sin(am)), P(0, FRONT_R, top))
    yb_top, yb_lip = frame_y(top, t), frame_y(LIP_TOP + (t if t else 0), t)
    poly = [P(0, FRONT_R, top), P(0, yb_top, top), P(0, yb_lip, LIP_TOP + (t if t else 0)),
            P(0, LIP_Y0, LIP_TOP + (t if t else 0)), P(0, LIP_Y0, z_bottom), P(0, t, z_bottom)]
    edges = [Part.LineSegment(pts[0], pts[1]).toShape(), arc.toShape()]
    edges += [Part.LineSegment(a, b).toShape() for a, b in zip(poly, poly[1:])]
    return Part.Face(Part.Wire(Part.__sortEdges__(edges)))


def plan_clip(inset, x0, x1):
    """Plan-view region: plate front outline (y 0, knob arch R29.75) up to the seam, x0..x1."""
    K1, K2 = (-40.16528, 0.0), (-21.24, 6.79578)
    def arcpt(a):
        return P(-40.16528 + 29.75 * math.cos(a), 29.75 + 29.75 * math.sin(a), 0)
    a0, a1 = math.atan2(K1[1] - 29.75, K1[0] + 40.16528), math.atan2(K2[1] - 29.75, K2[0] + 40.16528)
    # front line at y -2, NOT y 0: the profile already limits y >= 0, and coincident faces/edges make
    # OCCT's point classifier segfault (Extrema_ExtCC in BRepClass3d) during the booleans
    e = [Part.LineSegment(P(-120, -2, 0), P(K1[0], -2, 0)).toShape(),
         Part.LineSegment(P(K1[0], -2, 0), P(*K1, 0)).toShape(),
         Part.Arc(P(*K1, 0), arcpt((a0 + a1) / 2), P(*K2, 0)).toShape(),
         Part.LineSegment(P(*K2, 0), P(-20.0, 8.60, 0)).toShape(),          # meets the middle part's front
         Part.LineSegment(P(-20.0, 8.60, 0), P(-10.0, 8.60, 0)).toShape(),
         Part.LineSegment(P(-10.0, 8.60, 0), P(-10.0, 50, 0)).toShape(),
         Part.LineSegment(P(-10.0, 50, 0), P(-120, 50, 0)).toShape(),
         Part.LineSegment(P(-120, 50, 0), P(-120, -2, 0)).toShape()]
    f = Part.Face(Part.Wire(Part.__sortEdges__(e)))
    if inset:
        f = f.makeOffset2D(-inset, join=2)
    box = Part.Face(Part.makePolygon([P(x0, -5, 0), P(x1, -5, 0), P(x1, 50, 0), P(x0, 50, 0), P(x0, -5, 0)]))
    return f.common(box)


def plan_clip_inset(t, x0, x1):
    """plan_clip shrunk by t, built explicitly (makeOffset2D gives offset curves that crash OCCT here):
    concentric arch arc R29.75-t, front line far away (the profile limits y), simple seam-side lines."""
    cx, cy, R = -40.16528, 29.75, 29.75 - t
    a0 = -math.pi / 2
    a1 = math.atan2(6.79578 - cy, -21.24 - cx)
    pt = lambda a: P(cx + R * math.cos(a), cy + R * math.sin(a), 0)
    e = [Part.LineSegment(P(-120, -5, 0), P(cx, -5, 0)).toShape(),
         Part.LineSegment(P(cx, -5, 0), pt(a0)).toShape(),
         Part.Arc(pt(a0), pt((a0 + a1) / 2), pt(a1)).toShape(),
         Part.LineSegment(pt(a1), P(-20.0 - t * 0.824, 8.60 + t * 0.567 + 0.5, 0)).toShape(),
         Part.LineSegment(P(-20.0 - t * 0.824, 8.60 + t * 0.567 + 0.5, 0), P(-5, 8.60 + t * 0.567 + 0.5, 0)).toShape(),
         Part.LineSegment(P(-5, 8.60 + t * 0.567 + 0.5, 0), P(-5, 50, 0)).toShape(),
         Part.LineSegment(P(-5, 50, 0), P(-120, 50, 0)).toShape(),
         Part.LineSegment(P(-120, 50, 0), P(-120, -5, 0)).toShape()]
    f = Part.Face(Part.Wire(Part.__sortEdges__(e)))
    box = Part.Face(Part.makePolygon([P(x0, -6, 0), P(x1, -6, 0), P(x1, 51, 0), P(x0, 51, 0), P(x0, -6, 0)]))
    return f.common(box)


def prism(face_yz, x0, x1):
    s = face_yz.extrude(P(x1 - x0, 0, 0)); s.translate(P(x0, 0, 0)); return s


def column(face_xy, z0, z1):
    s = face_xy.extrude(P(0, 0, z1 - z0)); s.translate(P(0, 0, z0)); return s


# --- body ---------------------------------------------------------------------------------------
# prisms are always LONGER than needed and only the plan clip sets the x extents: any exact coincidence
# between prism ends and clip edges/vertices makes OCCT's classifier segfault
outer = prism(profile(0, PLATE_T), X_END - 3, X_SEAM + 3).common(column(plan_clip(0, X_END, X_SEAM), -1, 30))
print('step:', 'outer', flush=True)
cav = prism(profile(WALL, PLATE_T - 3), X_END - 3, X_SEAM + 8).common(column(plan_clip_inset(WALL, X_END + WALL, X_SEAM + 5), -5, 30))
print('step:', 'cav', flush=True)
# J1 bump: raised roof over the connector for the wires' U-turn
bx0, bx1, by0, btop = J1_BUMP
bump_face = Part.Face(Part.makePolygon([P(0, by0, TOP - 1), P(0, frame_y(TOP - 1), TOP - 1), P(0, frame_y(btop), btop),
                                        P(0, by0 + (btop - TOP + 1), btop), P(0, by0, TOP - 1)]))
bump = prism(bump_face, bx0, bx1)
print('step:', 'bump', flush=True)
bump = bump.common(prism(Part.Face(Part.makePolygon([P(0, -5, 0), P(0, 60, 0), P(0, 60, 40), P(0, -5, 40), P(0, -5, 0)])), bx0, bx1))
print('step:', 'bump', flush=True)
outer = outer.fuse(bump)
print('step:', 'outer', flush=True)
cav = cav.fuse(prism(Part.Face(Part.makePolygon([P(0, by0 + WALL, CEIL - 1), P(0, frame_y(btop - WALL, WALL), CEIL - 1),
                                                 P(0, frame_y(btop - WALL, WALL), btop - WALL),
                                                 P(0, by0 + WALL + (btop - TOP + 1), btop - WALL), P(0, by0 + WALL, CEIL - 1)])),
                     bx0 + WALL, bx1 - WALL))
print('step:', 'cav', flush=True)
part = outer.cut(cav)
print('step:', 'part', flush=True)

# --- seam: end wall above the middle part, socket for its spigot ---------------------------------
M = Part.Shape(); M.read(OUT + 'Middle.brep')
mid_env = mid['envelope'](0.0, X_SEAM - 6, X_SEAM + 1, mid['BOARD_TOP'], 45)          # middle outer, around the seam
end_wall = prism(profile(0, PLATE_T), X_SEAM - WALL - 3, X_SEAM + 3).common(column(plan_clip(0, X_SEAM - WALL, X_SEAM), -1, 30))
end_wall = end_wall.common(Part.makeBox(10, 60, 30, P(X_SEAM - 5, -5, mid['BOARD_TOP'] + 0.1)))   # never below the board top
part = part.fuse(end_wall)
print('step:', 'part', flush=True)
part = part.cut(mid_env.makeOffsetShape(FIT, 0.01, join=2))   # room for the middle part + its spigot
print('step:', 'part', flush=True)

# --- clearances: rear-plate post tubes (D7) and J1 + its plug -------------------------------------
POST_CLR_D = 7.6
for (x, y) in POSTS:
    part = part.cut(Part.makeCylinder(POST_CLR_D / 2, POST_BOSS_BOTTOM + 1, P(x, y, -1)))
    part = part.cut(Part.makeCylinder(10.4 / 2, 2.45 + 0.2 + 1, P(x, y, -1)))   # the plate's D10 boss under the PCB
# board corner at the seam (board arch edge (-22.51, 8.33) -> (-18.93, 8.71)): no wall below the board top there
seam_board = Part.Face(Part.makePolygon([P(-24.5, 8.33 - 0.15, 0), P(-22.51, 8.33 - 0.15, 0), P(-18.93, 8.71 - 0.15, 0),
                                         P(-18.93, 30, 0), P(-24.5, 30, 0), P(-24.5, 8.33 - 0.15, 0)]))
part = part.cut(column(seam_board, -1, mid['BOARD_TOP'] + 0.05))
J1_CLR = (-74.3, -65.2, 24.9, 3.3, J1_BUMP[3] - WALL)    # x0, x1, y0, z0, z1 (header 25.4..28.9, plug top 9.4)
jx0, jx1, jy0, jz0, jz1 = J1_CLR
part = part.cut(Part.makeBox(jx1 - jx0, 10, jz1 - jz0, P(jx0, jy0, jz0)))

# --- posts: countersunk M3 through a boss under the skin ------------------------------------------
for (x, y) in POSTS:
    part = part.fuse(Part.makeCylinder(POST_BOSS_D / 2, TOP - POST_BOSS_BOTTOM, P(x, y, POST_BOSS_BOTTOM)).common(outer))
    part = part.cut(Part.makeCylinder(SCREW_CLR / 2, 20, P(x, y, 0)))
    depth = (CSK_D - SCREW_CLR) / 2
    part = part.cut(Part.makeCone(SCREW_CLR / 2, CSK_D / 2, depth, P(x, y, TOP - depth)))
    part = part.cut(Part.makeCylinder(CSK_D / 2, 2, P(x, y, TOP)))

# --- encoder bushing, button knock-outs, SuperMini USB --------------------------------------------
x, y = ENC1
part = part.cut(Part.makeCylinder(BUSHING_HOLE / 2, 20, P(x, y, 0)))
print('step:', 'part', flush=True)
for (x, y) in BUTTONS:
    part = part.cut(Part.makeCylinder(KNOCKOUT_D / 2, TOP - KNOCKOUT_WEB, P(x, y, 0)))   # blind from inside, 0.4 web
ux0, ux1, uz0, uz1 = USB_OPEN
# USBs are mutually exclusive (SuperMini here, J2 on the right): knock-out, cut from the inside,
# 0.4 mm web left on the outer face of the front wall
part = part.cut(Part.makeBox(ux1 - ux0, 6, uz1 - uz0, P(ux0, KNOCKOUT_WEB, uz0)))
print('step:', 'part', flush=True)

# --- cable exit boss at the left end ----------------------------------------------------------------
c = CABLE
L = X_END - c['x_out'] + WALL
boss = Part.makeCylinder(c['boss_d'] / 2, L, P(c['x_out'], c['y'], c['z']), P(1, 0, 0))
boss = boss.common(Part.makeBox(30, 60, 40, P(c['x_out'] - 1, -10, PLATE_T)))           # never below the plate top
part = part.fuse(boss)
print('step:', 'part', flush=True)
part = part.cut(Part.makeCylinder(c['seat_d'] / 2, L - c['end_wall'] + 0.01, P(c['x_out'] + c['end_wall'], c['y'], c['z']), P(1, 0, 0)))
print('step:', 'part', flush=True)
part = part.cut(Part.makeCylinder(c['exit_d'] / 2, c['end_wall'] + 0.02, P(c['x_out'] - 0.01, c['y'], c['z']), P(1, 0, 0)))
print('step:', 'part', flush=True)

part = part.removeSplitter()
print('step:', 'part', flush=True)
part.exportBrep(OUT + 'Left.brep')
print('Left', part.isValid(), round(part.Volume, 1), part.BoundBox)
