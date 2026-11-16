"""ButtHeat v2: cable exit geometry at the left end, shared by v2_left.py, v2_rear_plate.py and v2_clamp.py
(exec'd into their namespace). Case coordinates.

The cable (spare USB 2.0, textile jacket) is clamped to the front assembly by a replaceable split insert
(v2_clamp.py) in a seat in the left part. A SLOT_W slot below the seat and every exit path lets the
already crimped cable be laid in from below; the insert (OD > slot) then slides into the seat from the J1
side, its outer end face bearing on the seat's end (takes the pull). Halves split top/bottom at the axis;
the bottom half's tab at the J1 end is held by an M2 self-tapping screw from below into a boss hanging
from the left part's ceiling.
"""
import FreeCAD as App

_FALLBACK_CABLE_D = 4.5        # used when V2Vars.cable_d isn't in the saved document yet


def _cable_d():
    try:
        d = (App.getDocument('ButtHeatModel') if 'ButtHeatModel' in App.listDocuments()
             else App.openDocument('/home/mars/tmp/3dp/buttheat/ButtHeatModel.FCStd'))
        return float(d.getObject('V2Vars').cable_d.Value)
    except Exception:
        return _FALLBACK_CABLE_D


CABLE_D = _cable_d()                                  # cable outer diameter (V2Vars.cable_d)
SEAT = dict(x0=-79.0, x1=-74.6, y=25.2, z=6.35, d=8.8)   # axis along x; the insert rests just above the plate
SEAT_BOSS_D = 11.0                                    # material around the seat (clipped to the part)
EXIT_D = 6.0                                          # exit channels from the seat's outer end
SLOT_W = 5.0                                          # lay-in slot below the seat and the exit paths
assert CABLE_D <= SLOT_W - 0.2, 'cable does not fit through the lay-in slot'
_S = (SEAT['x0'], SEAT['y'], SEAT['z'])
CABLE_PATHS = [
    [_S, (-96.5, 27.8, 2.5), (-106.0, 27.8, 2.5)],   # A: down into the V between display frame and vent
                                                      #    housing: at x -96 it touches the frame and z ~0
    [_S, (-84.75, 25.43, 2.5), (-92.0, 25.43, 2.5)]]  # B: straight into the housing (if it gets a cut-out)
PLATE_NOTCH_R = 3.0                                   # rear plate ear notched along both paths

# insert (v2_clamp.py)
INSERT_OD = 8.6                                       # 0.1 radial clearance in the seat
INSERT_GAP = 0.05                                     # axial clearance at each end of the seat
SPLIT_GAP = 0.2                                       # between the halves (each trimmed 0.1 off the axis plane)
SPLIT_STEP = 1.2                                      # on the -y (tab) side the split is raised by this: the
                                                      # bottom half's -y wall carries the tab solidly
BORE_SQUEEZE = 0.5                                    # bore = CABLE_D - this: assembled, the halves stand 8.9
                                                      # tall in the 8.8 seat -> clamped
GRIP_H, GRIP_W, GRIP_N = 0.3, 0.6, 3                  # grip rings inside the bore
PEG_D, PEG_H, PEG_HOLE_D = 1.0, 0.8, 1.2              # bottom half -> top half (axial + rotation lock)

# M2 self-tapping screw (from below) through the bottom half's tab into the left part's boss
M2 = dict(x=-74.2, y=19.0, boss_d=5.0, pilot_d=1.6, depth=4.0, clr_d=2.3)
TAB = dict(x0=-76.0, x1=-72.4, y0=17.0, y1=20.7, z0=5.8, z1=7.3)   # tab top = boss bottom; z0: the M2 head
                                                                   # (z 4.5..5.8) clears the SuperMini (top 4.29)
M2_HEAD_D = 4.4                                       # clearance below the tab for the head (and the driver)
TAB_CLR = 0.2                                         # pocket around the tab in the left part
