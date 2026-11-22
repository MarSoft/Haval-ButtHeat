"""ButtHeat v2: cable entry and strain relief at the left end, shared by v2_side.py (left part), v2_rear_plate.py
and v2_clamp.py (exec'd into their namespace; names land there, keep them distinct). Case coordinates.

2026-11-21 (replaces the round seat + split insert, then the S-groove clamp):
  - the jacketed cable (spare USB 2.0, textile, CABLE_D) comes in from the V-groove between the display frame
    and the vent housing (path A, through the ear tip), then follows the ear's inner arc wall (R ~23.45 around
    the vent centre, a gentle bend for the jacket) down towards J1;
  - on the arc it's clamped by THREE BARS across the channel, alternating: two hang from the case ceiling
    (CASE_BAR_PHI), one rises from the SADDLE between them (SADDLE_BAR_PHI). Tightening the saddle's M2x4
    (right at the middle bar, on the outer side of the curve) bends the cable into a zig-zag around the bars
    (grip by bending, not by crushing; works over a range of cable diameters); the saddle's screw lobe bottoms
    out on the case's screw column at the maximum bend (ZIG). The screw pulls the outer side up and the cable
    pushes the middle down, so the inner (wall-side) edge is carried by a foot rib (SADDLE_FOOT) resting on the
    rear plate (no locating pin: the screw plus the arc wall 0.05 off the inner edge keep it from turning);
  - the jacket ends at PHI_JACKET_END; the conductors then run down, loop over the WROOM shield (splices with the
    pre-crimped ZH leads rest there) and come back up into J1's plug (faces -y);
  - the ear wall is thinned to WALL_THIN along the cable (the tight place between J1 and the wall).
Laying in: the front part upside down, the board not fitted: lay the cable under the case bars and into the
exit channel (slot from below), put the saddle on, screw it until it stops.
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


CABLE_D = _cable_d()                       # nominal cable diameter (V2Vars.cable_d; the user's cable 4.5)
ARC_C = (-96.0, 8.95)                      # ear arc centre (car vent housing)
WALL_THIN, WALL_THIN_PHI = 1.0, (12.0, 58.0)   # ear wall thinned to this along the cable (deg)
CABLE_R = 20.0 + WALL_THIN + 0.2 + CABLE_D / 2  # cable centreline radius around ARC_C (23.45)
PLATE_TOP_Z = 2.0                          # rear plate top in the ear region
CABLE_Z = 7.7                              # nominal (straight) cable centre height
PHI_ENTRY, PHI_JACKET_END = 52.0, 31.0     # jacket on the arc between these (deg); entry from path A
CHANNEL_W = CABLE_D + 0.3                  # radial width of the bars (the channel)
BAR_T, BAR_TIP_R = 1.2, 0.6                # bar thickness along the cable, fully rounded tip
SADDLE_BAR_PHI = 41.0                      # middle bar (saddle), right at the screw
BAR_PITCH = 2.5                            # along the cable centreline
_dphi = math.degrees(BAR_PITCH / CABLE_R)
CASE_BAR_PHI = (SADDLE_BAR_PHI + _dphi, SADDLE_BAR_PHI - _dphi)   # ~47.1, ~34.9
ZIG = 1.6                                  # max zig-zag of the cable centre (saddle home)
CASE_BAR_TIP = CABLE_Z - ZIG / 2 + CABLE_D / 2   # 9.15: cable centre under them 6.9 (max bend)
SADDLE_BAR_TIP = CABLE_Z + ZIG / 2 - CABLE_D / 2 # 6.25 when home: cable centre over it 8.5
SADDLE_BASE = (3.7, 4.45)                  # saddle base plate (under the cable at the case bars: bottom 4.65)
SADDLE_PHI = (CASE_BAR_PHI[1] - 1.8, CASE_BAR_PHI[0] + 1.8)
SADDLE_R = (CABLE_R - CHANNEL_W / 2, CABLE_R + CHANNEL_W / 2)
STOP_Z = 5.3                               # saddle lobe top = case column bottom (hard stop) when home
SCREW = dict(phi=SADDLE_BAR_PHI, r=27.35, lobe_r=1.6, pilot_d=1.6, clr_d=2.3, pilot_depth=3.0)   # M2x4 from below
SADDLE_FOOT = (20.0 + WALL_THIN + 0.1, 22.2, PLATE_TOP_Z + 0.05)   # rib under the saddle's inner edge: R0, R1,
                                           # bottom z (0.05 over the rear plate top; the plate takes that edge)
RING_WEB_R0, RING_WEB_W = CABLE_R + CABLE_D / 2 + 0.2, 3.0   # saddle: full-height web lobe -> body from this R
                                           # (outside the cable), half-width at that R (mm along the arc)
EXIT_D, SLOT_W = 6.0, 5.0                  # outside exit channel + lay-in slot below it
assert CABLE_D <= SLOT_W - 0.2, 'cable does not fit through the lay-in slot'


def arc_pt(phi_deg, r=CABLE_R, z=CABLE_Z):
    a = math.radians(phi_deg)
    return (ARC_C[0] + r * math.cos(a), ARC_C[1] + r * math.sin(a), z)


def polar(phi_deg, r):
    a = math.radians(phi_deg)
    return (ARC_C[0] + r * math.cos(a), ARC_C[1] + r * math.sin(a))


def cable_path(n=60):
    """Jacket centreline on the arc, PHI_ENTRY -> PHI_JACKET_END, bent around the bars (saddle home)."""
    zc_case, zc_mid = CASE_BAR_TIP - CABLE_D / 2, SADDLE_BAR_TIP + CABLE_D / 2
    keys = [(PHI_ENTRY, CABLE_Z), (CASE_BAR_PHI[0], zc_case), (SADDLE_BAR_PHI, zc_mid), (CASE_BAR_PHI[1], zc_case),
            (PHI_JACKET_END, CABLE_Z)]
    pts = []
    for i in range(n + 1):
        phi = PHI_ENTRY + (PHI_JACKET_END - PHI_ENTRY) * i / n
        for (p0, z0), (p1, z1) in zip(keys, keys[1:]):
            if p1 <= phi <= p0:
                t = (p0 - phi) / (p0 - p1)
                z = z0 + (z1 - z0) * (1 - math.cos(math.pi * t)) / 2     # smooth between the key points
                break
        pts.append(arc_pt(phi, CABLE_R, z))
    return pts


def straight_tube(d, phi0, phi1):
    """Tube of diameter d along the nominal (straight, z CABLE_Z) arc, phi0 -> phi1: clearance cuts."""
    a0, a1 = min(phi0, phi1), max(phi0, phi1)
    t = Part.makeTorus(CABLE_R, d / 2, App.Vector(ARC_C[0], ARC_C[1], CABLE_Z), App.Vector(0, 0, 1), -180, 180, a1 - a0)
    t.rotate(App.Vector(ARC_C[0], ARC_C[1], 0), App.Vector(0, 0, 1), a0)
    return t


# outside: from the arc entry through the ear tip down into the V between display frame and vent housing
# (at x -96 the cable touches the frame and z ~0), then along -x
_E = arc_pt(PHI_ENTRY)
CABLE_PATHS = [[_E, (-96.5, 27.8, 2.5), (-106.0, 27.8, 2.5)]]
PLATE_NOTCH_R = 3.0                        # rear plate ear notched along the path


def cable_tube(d, phi0=PHI_ENTRY, phi1=PHI_JACKET_END, n=60):
    """Solid tube of diameter d along the jacket centreline (zig-zag around the bars) from phi0 to phi1."""
    pts = [p for p in cable_path(n) if min(phi0, phi1) - 1e-9 <= _phi_of(p) <= max(phi0, phi1) + 1e-9]
    v = [App.Vector(*p) for p in pts]
    spine = Part.Wire(Part.BSplineCurve(v).toShape()) if len(v) > 2 else Part.Wire(Part.LineSegment(v[0], v[1]).toShape())
    t = (v[1] - v[0]); t.normalize()
    circ = Part.Wire(Part.makeCircle(d / 2, v[0], t))
    ps = Part.BRepOffsetAPI.MakePipeShell(spine)
    ps.setBiNormalMode(App.Vector(0, 0, 1))   # fixed binormal (Frenet flips at the path's inflections)
    ps.add(circ)
    ps.build()
    ps.makeSolid()
    return ps.shape()


def _phi_of(p):
    return math.degrees(math.atan2(p[1] - ARC_C[1], p[0] - ARC_C[0]))

