"""ButtHeat v2 rear plate (PETG, FDM), generated into the open ButtHeatModel document.

Run headless (MCP execute_code_headless or freecadcmd); it writes hw/cad/out/RearPlate_{A,B}.brep.
Load them into the GUI with hw/cad/v2_load.py (no booleans in the GUI process: they crashed it).
Driving numbers come from V2Vars (read from the saved .FCStd when headless); the rest are the
constants below (move them to V2Vars once they settle).

Coordinates: case = KiCad grid origin = car knob centre; z = 0 car console surface, rear plate
z 0..plate_t, carrier PCB bottom at plate_t + gap. Display frame plane: y = 29.75 + z*tan(15 deg).

Objects (in group "V2 Case"):
  RearPlate_A  display option 1: full cut-out for the OLED driver board hanging under the carrier
  RearPlate_B  display option 3: shallow pocket for the rear passives under the glass
"""
import math
import FreeCAD as App
import Part

MODEL = '/home/mars/tmp/3dp/buttheat/ButtHeatModel.FCStd'
OUT = '/home/mars/my/buttheat/hw/cad/out/'
doc = (App.getDocument('ButtHeatModel') if 'ButtHeatModel' in App.listDocuments()
       else App.openDocument(MODEL))
V = doc.getObject('V2Vars')
v = lambda n: float(getattr(V, n).Value)

PLATE_T = v('plate_t')            # 2.0
GAP = 0.5                         # PCB bottom above plate top
PCB_T = v('pcb_t')                # 1.0
TAPE_GAP = 0.2                    # clearance to the display frame (thin tape)
FRAME_Y0, FRAME_TAN = 29.75, math.tan(math.radians(15))
LIP_Y0, LIP_H = 28.3, 6.0         # lip: wedge from just behind the PCB top edge (y 28 + 0.3) to the frame
LIP_J1 = (-74.3, -65.2, 3.3)      # J1 (ZH) body overhangs the board edge from z 3.5: lip lowered there
MID_SEAM, MID_LIP = 23.0, 5.0     # middle front part x -20..20 + 3 mm spigots: its skin (z 5.0..6.5) rests on the lip
EAR_X, EAR_R = 96.0, 20.0         # ear: R20 quarter disc, v1 thin strip to x 106 removed
MAIN_X = 76.0
POST_D = 6.6                      # spacer tube (1.0 wall around the insert); board notches are D7.5 (V2Vars.notch_d)
BOSS_D, BOSS_TOP = 10.0, None     # boss under the board (top set below to the PCB bottom)
INSERT_D = v('insert_pilot_d')    # 4.6 (M3 x 3 x 4.5 heat-set), pressed in from the plate underside
INSERT_DEPTH = 5.0                # pilot from z 0 up to a shoulder at z 5: the 3 mm insert is pressed in
                                  # from below up to it (sits z 2..5); the screw pull pushes it into the boss
SCREW_CLR = 3.4                   # M3 clearance through the spacer tube
SCREW_HEAD_POCKETS = [(-27.0, 13.0), (27.0, 13.0)]   # H1/H2 fallback M2 screw heads under the PCB
SCREW_HEAD_D = 5.5                # through-hole (plate is only 2 mm)
LIP_CHAMFER = 1.0                 # lip ends and J1 steps: 45 deg (run = rise)
POST_TOP = 6.0                    # short posts (easy to lift the front + board off); the front parts' posts
                                  # continue from z 6.1. Screws M3x8 CSK, head top at z 9.0 (tip at z 1.0)
POSTS = [(-v('post1_left'), v('post1_y')), (v('post2_x'), v('post2_y')),
         (-v('post3_left'), v('post3_y')), (v('post4_x'), v('post4_y'))]
PAD_H = GAP                       # support pads touch the PCB bottom
ENC_PADS = [(-43.0, 11.0), (43.0, 11.0), (65.0, 15.0)]   # shaft axes
ENC_PAD_D = 6.0                   # clears A/B/C, switch pins and lugs (>= 5.6 mm from axis)
BTN_PADS = [(-27.0, 19.0), (27.0, 19.0)]
BTN_PAD_D = 4.0                   # inside the 4 legs (r 3.9)
# parts on the PCB back (other than the display passives): x, y, w, h (pad extent + margin), pocket depth
BACK_POCKETS = [(-71.0, 23.3, 3.6, 3.6, 1.0),     # D1 SOT-23
                (-40.7, 24.8, 1.6, 2.9, 1.0),     # C1 0603 (rot 90) under U3 pin 3
                (-62.9, 3.8, 2.1, 3.5, 1.0)]      # C20 0805 10uF (rot 90, <= 1.25 tall) at the WROOM 3V3
DRV_BOARD = (-19.29 - 0.5, 18.71 + 0.5, 16.0 - 0.5, 28.0 + 0.3)   # option 1 cut-out (x0, x1, y0, y1)
PASSIVES = (-5.5, 13.0, 13.5, 25.0)                                 # option 3 pocket
PASSIVES_POCKET = 0.8


def arch_wire(z):
    """Plate outline at height z (knob arch from v1 CaseBase, ears without the thin strip)."""
    yt = FRAME_Y0 + z * FRAME_TAN - TAPE_GAP
    P = App.Vector
    e = []
    def line(a, b): e.append(Part.LineSegment(P(*a, z), P(*b, z)).toShape())
    def arc(a, m, b): e.append(Part.Arc(P(*a, z), P(*m, z), P(*b, z)).toShape())
    def arcc(c, a, b):
        """Arc around centre c from exact point a to exact point b (shorter way)."""
        a0 = math.atan2(a[1] - c[1], a[0] - c[0])
        a1 = math.atan2(b[1] - c[1], b[0] - c[0])
        da = (a1 - a0 + math.pi) % (2 * math.pi) - math.pi
        r = math.hypot(a[0] - c[0], a[1] - c[1])
        m = (c[0] + r * math.cos(a0 + da / 2), c[1] + r * math.sin(a0 + da / 2))
        arc(a, m, b)
    # left half (from the top-left corner round to the centre), then mirrored, then the top edge
    K1, K2, K3 = (-40.16528, 0.0), (-21.24, 6.79578), (-18.92529, 8.70422)   # v1 CaseBase arch points
    line((-EAR_X, yt), (-EAR_X, 28.95))
    arcc((-EAR_X, 8.95), (-EAR_X, 28.95), (-MAIN_X, 8.95))        # ear quarter arc R20
    line((-MAIN_X, 8.95), (-MAIN_X, 0.0))
    line((-MAIN_X, 0.0), K1)
    arcc((-40.16528, 29.75), K1, K2)                               # knob arch, outer arc R29.75
    line(K2, K3)
    arcc((0.0, -14.25), K3, (0.0, 15.5))                           # knob arch, centre arc R29.75
    left = e[:]
    m = App.Matrix(); m.scale(-1, 1, 1)
    right = [s.transformGeometry(m) for s in reversed(left)]
    e.extend(right)
    e.append(Part.LineSegment(P(EAR_X, yt, z), P(-EAR_X, yt, z)).toShape())
    return Part.Wire(Part.__sortEdges__(e))


def plate_body():
    w0 = arch_wire(0.0)
    base = Part.Face(w0).extrude(App.Vector(0, 0, PLATE_T))   # top edge vertical; the lip follows the frame
    assert base.isValid(), 'plate base invalid'
    # lip along the top edge, leaning on the display frame
    P = App.Vector
    prof = Part.makePolygon([P(0, LIP_Y0, 0), P(0, FRAME_Y0 - TAPE_GAP, 0),
                             P(0, FRAME_Y0 - TAPE_GAP + LIP_H * FRAME_TAN, LIP_H),
                             P(0, LIP_Y0, LIP_H), P(0, LIP_Y0, 0)])
    lip = Part.Face(prof).extrude(P(2 * MAIN_X, 0, 0))
    lip.translate(P(-MAIN_X, 0, 0))
    x0, x1, zj = LIP_J1

    def wedge_cut(xa, xb, z0, rising_right):
        """45 deg end cut over x xa..xb starting at height z0 (material kept below the slope)."""
        h = LIP_H - z0 + 1
        if rising_right:   # slope goes up towards +x: remove the triangle above it, on the left
            pts = [P(xa, 0, z0), P(xb, 0, z0 + (xb - xa)), P(xb, 0, LIP_H + 1), P(xa, 0, LIP_H + 1)]
        else:
            pts = [P(xa, 0, z0 + (xb - xa)), P(xb, 0, z0), P(xb, 0, LIP_H + 1), P(xa, 0, LIP_H + 1)]
        f = Part.Face(Part.makePolygon(pts + [pts[0]]))
        s = f.extrude(P(0, 10, 0)); s.translate(P(0, LIP_Y0 - 2, 0))
        return s

    run = LIP_H                                     # 45 deg over the full lip height at the outer ends
    lip = lip.cut(wedge_cut(-MAIN_X - 0.01, -MAIN_X + run, 0.0, True))
    lip = lip.cut(wedge_cut(MAIN_X - run, MAIN_X + 0.01, 0.0, False))
    # middle zone: the middle part's skin (z MID_LIP..6.5) rests on the lip -> lower it there, 45 deg flanks
    mx, mz = MID_SEAM, MID_LIP
    lip = lip.cut(Part.makeBox(2 * mx, 6, LIP_H, P(-mx, LIP_Y0 - 1, mz)))
    lip = lip.cut(wedge_cut(-mx - (LIP_H - mz), -mx, mz, False))
    lip = lip.cut(wedge_cut(mx, mx + (LIP_H - mz), mz, True))
    jrun = LIP_H - zj                               # J1 step: down from LIP_H to zj on both sides
    lip = lip.cut(Part.makeBox(x1 - x0, 6, LIP_H, P(x0, LIP_Y0 - 1, zj)))
    lip = lip.cut(wedge_cut(x0 - jrun, x0, zj, False))
    lip = lip.cut(wedge_cut(x1, x1 + jrun, zj, True))
    solid = base.fuse(lip)
    boss_top = PLATE_T + GAP - 0.05                 # just under the PCB
    footprint = Part.Face(w0).extrude(P(0, 0, boss_top + 0.1))   # bosses never leave the plate outline
    for (x, y) in POSTS:
        solid = solid.fuse(Part.makeCylinder(BOSS_D / 2, boss_top, P(x, y, 0)).common(footprint))
        solid = solid.fuse(Part.makeCylinder(POST_D / 2, POST_TOP - PLATE_T, P(x, y, PLATE_T)))
    for (x, y) in ENC_PADS:
        solid = solid.fuse(Part.makeCylinder(ENC_PAD_D / 2, PAD_H, P(x, y, PLATE_T)))
    for (x, y) in BTN_PADS:
        solid = solid.fuse(Part.makeCylinder(BTN_PAD_D / 2, PAD_H, P(x, y, PLATE_T)))
    for (x, y) in POSTS:
        solid = solid.cut(Part.makeCylinder(INSERT_D / 2, INSERT_DEPTH, P(x, y, -0.01)))
        solid = solid.cut(Part.makeCylinder(SCREW_CLR / 2, POST_TOP + 1, P(x, y, 0)))
    for (x, y) in SCREW_HEAD_POCKETS:
        solid = solid.cut(Part.makeCylinder(SCREW_HEAD_D / 2, PLATE_T + 1, P(x, y, -0.5)))
    for x, y, w, h, d in BACK_POCKETS:
        solid = solid.cut(Part.makeBox(w, h, d + 0.01, P(x - w / 2, y - h / 2, PLATE_T - d)))
    return solid.removeSplitter()


def box(r, z0, dz):
    x0, x1, y0, y1 = r
    return Part.makeBox(x1 - x0, y1 - y0, dz, App.Vector(x0, y0, z0))


base = plate_body()
A = base.cut(box(DRV_BOARD, -1, PLATE_T + 2)).removeSplitter()
B = base.cut(box(PASSIVES, PLATE_T - PASSIVES_POCKET, PASSIVES_POCKET + 0.01)).removeSplitter()

import os
os.makedirs(OUT, exist_ok=True)
for name, shape in (('RearPlate_A', A), ('RearPlate_B', B)):
    shape.exportBrep(OUT + name + '.brep')
    print(name, shape.isValid(), round(shape.Volume, 1), shape.BoundBox)
