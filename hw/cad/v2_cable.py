"""ButtHeat v2: cable entry and strain relief at the left end, shared by v2_side.py (left part), v2_rear_plate.py
and v2_clamp.py (exec'd into their namespace; names land there, keep them distinct). Case coordinates.

2026-11-20 redesign (replaces the round seat + split insert):
  - the jacketed cable (spare USB 2.0, textile, CABLE_D) comes in from the V-groove between the display frame
    and the vent housing (path A, through the ear tip), then follows the ear's inner arc wall (R ~23.45 around
    the vent centre, a gentle bend for the jacket) down towards J1, at z CABLE_Z;
  - on the arc it's clamped: the case has a boss under the ceiling carrying the groove's upper half (all of it
    above z BOSS_Z0: J1's header and plug reach z 8.35 right next to it); a SADDLE from below carries the lower
    half, one M2x4 self-tapper + one pin (both on the outer side of the curve: the inner side is the wall,
    further out is J1); the groove is a gentle S (radial wave) for grip;
  - the jacket ends at PHI_JACKET_END; the conductors then run down, loop over the WROOM shield (splices with the
    pre-crimped ZH leads rest there) and come back up into J1's plug (faces -y);
  - the ear wall is thinned to WALL_THIN along the cable (the tight place between J1 and the wall).
Laying in: the front part upside down, the board not fitted: lay the cable into the groove and the exit channel
(slot from below), put the saddle on, screw it.
"""
import math
import FreeCAD as App
import Part

_FALLBACK_CABLE_D = 4.5        # used when V2Vars.cable_d isn't in the saved document


def _cable_d():
    try:
        d = (App.getDocument('ButtHeatModel') if 'ButtHeatModel' in App.listDocuments()
             else App.openDocument('/home/mars/tmp/3dp/buttheat/ButtHeatModel.FCStd'))
        return float(d.getObject('V2Vars').cable_d.Value)
    except Exception:
        return _FALLBACK_CABLE_D


CABLE_D = _cable_d()                       # cable outer diameter (V2Vars.cable_d); the user's cable fits 4.5
SQUEEZE = 0.5                              # groove = CABLE_D - SQUEEZE when the saddle is screwed home
ARC_C = (-96.0, 8.95)                      # ear arc centre (car vent housing)
WALL_THIN, WALL_THIN_PHI = 1.0, (12.0, 58.0)   # ear wall thinned to this along the cable (deg)
CABLE_R = 20.0 + WALL_THIN + 0.4 + CABLE_D / 2  # cable centreline radius around ARC_C (23.65; the wave's
                                           # squeezed groove then stays 0.15 off the thinned wall)
CABLE_Z = 8.2                              # cable centre height (bottom 5.95 over the boards, top 10.45 < ceiling)
PHI_ENTRY, PHI_JACKET_END = 52.0, 31.0     # jacket on the arc between these (deg); entry from path A
CLAMP_PHI = (50.0, 32.0)                   # clamped section
WAVE_A = 0.5                               # S-wave amplitude (radial), one period over the clamped section
BOSS_Z0 = 8.5                              # case boss underside (J1 header/plug top 8.35)
SADDLE_Z = (4.8, 8.0)                      # saddle bottom/top (0.5 below the boss: the squeeze gap)
SADDLE_R = (21.3, 25.8)                    # saddle radial extent
SCREW = dict(phi=45.0, r=26.3, pilot_d=1.6, clr_d=2.3, lobe_r=2.0, lobe_z0=6.5)   # M2x4 from below
PIN = dict(phi=39.5, r=26.3, d=1.4, hole_d=1.6, lobe_r=1.6, len=1.6)            # saddle pin into the boss
BOSS_COL_D = 4.4                           # boss columns around the screw / pin hole (from the ceiling to BOSS_Z0)
EXIT_D, SLOT_W = 6.0, 5.0                  # outside exit channel + lay-in slot below it
assert CABLE_D <= SLOT_W - 0.2, 'cable does not fit through the lay-in slot'


def arc_pt(phi_deg, r=CABLE_R, z=CABLE_Z):
    a = math.radians(phi_deg)
    return (ARC_C[0] + r * math.cos(a), ARC_C[1] + r * math.sin(a), z)


def polar(phi_deg, r):
    a = math.radians(phi_deg)
    return (ARC_C[0] + r * math.cos(a), ARC_C[1] + r * math.sin(a))


def cable_path(n=60):
    """Jacket centreline on the arc, PHI_ENTRY -> PHI_JACKET_END, with the S-wave over the clamped section."""
    pts = []
    for i in range(n + 1):
        phi = PHI_ENTRY + (PHI_JACKET_END - PHI_ENTRY) * i / n
        r = CABLE_R
        c0, c1 = CLAMP_PHI
        if c1 <= phi <= c0:
            r += WAVE_A * math.sin(2 * math.pi * (c0 - phi) / (c0 - c1))
        pts.append(arc_pt(phi, r))
    return pts


# outside: from the arc entry through the ear tip down into the V between display frame and vent housing
# (at x -96 the cable touches the frame and z ~0), then along -x
_E = arc_pt(PHI_ENTRY)
CABLE_PATHS = [[_E, (-96.5, 27.8, 2.5), (-106.0, 27.8, 2.5)]]
PLATE_NOTCH_R = 3.0                        # rear plate ear notched along the path


def cable_tube(d, phi0=PHI_ENTRY, phi1=PHI_JACKET_END, n=60):
    """Solid tube of diameter d along the jacket centreline (with the clamp's S-wave) from phi0 to phi1."""
    pts = [p for p in cable_path(n) if min(phi0, phi1) - 1e-9 <= _phi_of(p) <= max(phi0, phi1) + 1e-9]
    v = [App.Vector(*p) for p in pts]
    spine = Part.Wire(Part.BSplineCurve(v).toShape()) if len(v) > 2 else Part.Wire(Part.LineSegment(v[0], v[1]).toShape())
    t = (v[1] - v[0]); t.normalize()
    circ = Part.Wire(Part.makeCircle(d / 2, v[0], t))
    ps = Part.BRepOffsetAPI.MakePipeShell(spine)
    ps.setBiNormalMode(App.Vector(0, 0, 1))   # planar path: fixed binormal (Frenet flips at the S-wave's inflections)
    ps.add(circ)
    ps.build()
    ps.makeSolid()
    return ps.shape()


def _phi_of(p):
    return math.degrees(math.atan2(p[1] - ARC_C[1], p[0] - ARC_C[0]))

