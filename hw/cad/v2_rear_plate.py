"""ButtHeat v2 rear plate (PETG, FDM), generated into the open ButtHeatModel document.

Run inside FreeCAD (MCP execute_code):  exec(open('/home/mars/my/buttheat/hw/cad/v2_rear_plate.py').read(), {})
Re-running replaces the objects. Driving numbers come from V2Vars where they exist; the rest are
the constants below (move them to V2Vars once they settle).

Coordinates: case = KiCad grid origin = car knob centre; z = 0 car console surface, rear plate
z 0..plate_t, carrier PCB bottom at plate_t + gap. Display frame plane: y = 29.75 + z*tan(15 deg).

Objects (in group "V2 Case"):
  RearPlate_A  display option 1: full cut-out for the OLED driver board hanging under the carrier
  RearPlate_B  display option 3: shallow pocket for the rear passives under the glass
"""
import math
import FreeCAD as App
import Part

doc = App.getDocument('ButtHeatModel')
V = doc.getObject('V2Vars')
v = lambda n: float(getattr(V, n).Value)

PLATE_T = v('plate_t')            # 2.0
GAP = 0.5                         # PCB bottom above plate top
PCB_T = v('pcb_t')                # 1.0
TAPE_GAP = 0.2                    # clearance to the display frame (thin tape)
FRAME_Y0, FRAME_TAN = 29.75, math.tan(math.radians(15))
LIP_DEPTH, LIP_H = 2.5, 6.0       # lip along the top edge, leaning on the frame
EAR_X, EAR_R = 96.0, 20.0         # ear: R20 quarter disc, v1 thin strip to x 106 removed
MAIN_X = 76.0
POST_D = v('post_d')              # 6.5
INSERT_D = v('insert_pilot_d')    # 4.6 (M3 x 3 x 4.5 heat-set)
INSERT_DEPTH = 5.0
SIDE_TOP = 12.4                   # side parts' outer top (v1)
WALL = 1.5                        # SLA wall
POST_TOP = SIDE_TOP - WALL        # posts reach the side parts' inner ceiling
POSTS = [(-v('post1_left'), v('post1_y')), (v('post2_x'), v('post2_y')),
         (-v('post3_left'), v('post3_y')), (v('post4_x'), v('post4_y'))]
PAD_H = GAP                       # support pads touch the PCB bottom
ENC_PADS = [(-43.0, 11.0), (43.0, 11.0), (65.0, 15.0)]   # shaft axes
ENC_PAD_D = 6.0                   # clears A/B/C, switch pins and lugs (>= 5.6 mm from axis)
BTN_PADS = [(-27.0, 19.0), (27.0, 19.0)]
BTN_PAD_D = 4.0                   # inside the 4 legs (r 3.9)
D1 = (-71.0, 23.3, 3.6, 3.6)      # SOT-23 on the back: x, y, w, h
D1_POCKET = 1.0
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
    prof = Part.makePolygon([P(0, FRAME_Y0 - TAPE_GAP - LIP_DEPTH, 0), P(0, FRAME_Y0 - TAPE_GAP, 0),
                             P(0, FRAME_Y0 - TAPE_GAP + LIP_H * FRAME_TAN, LIP_H),
                             P(0, FRAME_Y0 - TAPE_GAP - LIP_DEPTH, LIP_H), P(0, FRAME_Y0 - TAPE_GAP - LIP_DEPTH, 0)])
    lip = Part.Face(prof).extrude(P(2 * MAIN_X, 0, 0))
    lip.translate(P(-MAIN_X, 0, 0))
    solid = base.fuse(lip)
    for (x, y) in POSTS:
        post = Part.makeCylinder(POST_D / 2, POST_TOP - PLATE_T, P(x, y, PLATE_T))
        solid = solid.fuse(post)
    for (x, y) in ENC_PADS:
        solid = solid.fuse(Part.makeCylinder(ENC_PAD_D / 2, PAD_H, P(x, y, PLATE_T)))
    for (x, y) in BTN_PADS:
        solid = solid.fuse(Part.makeCylinder(BTN_PAD_D / 2, PAD_H, P(x, y, PLATE_T)))
    for (x, y) in POSTS:
        solid = solid.cut(Part.makeCylinder(INSERT_D / 2, INSERT_DEPTH, P(x, y, POST_TOP - INSERT_DEPTH)))
    x, y, w, h = D1
    solid = solid.cut(Part.makeBox(w, h, D1_POCKET, P(x - w / 2, y - h / 2, PLATE_T - D1_POCKET)))
    return solid.removeSplitter()


def box(r, z0, dz):
    x0, x1, y0, y1 = r
    return Part.makeBox(x1 - x0, y1 - y0, dz, App.Vector(x0, y0, z0))


base = plate_body()
A = base.cut(box(DRV_BOARD, -1, PLATE_T + 2)).removeSplitter()
B = base.cut(box(PASSIVES, PLATE_T - PASSIVES_POCKET, PASSIVES_POCKET + 0.01)).removeSplitter()

grp = doc.getObject('V2_Case') or doc.addObject('App::DocumentObjectGroup', 'V2_Case')
grp.Label = 'V2 Case'
for name, shape, label in (('RearPlate_A', A, 'Rear plate A (display option 1: driver-board cut-out)'),
                           ('RearPlate_B', B, 'Rear plate B (display option 3: passives pocket)')):
    o = doc.getObject(name)
    if o is None:
        o = doc.addObject('Part::Feature', name)
        grp.addObject(o)
    o.Label = label
    o.Shape = shape
print('A', A.isValid(), round(A.Volume, 1), A.BoundBox)
print('B', B.isValid(), round(B.Volume, 1), B.BoundBox)
