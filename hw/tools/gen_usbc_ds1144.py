#!/usr/bin/env python3
"""USB-C receptacle Connfly DS1144-07 (16P, top-mount SMT, L 6.5 mm) for J2, as ButtHeat:USB_C_DS1144-07.

Signal pad row placed like USB_C_XKB_U262-16XN-4BVC11_noSBU (drop-in for the existing routing, same
origin); SBU pads A8/B8 omitted (USB 2.0 only). On our board the front face can't be flush with the board
edge as the datasheet intends (the pads would hit ENC3's switch pins): with J2's origin 2.175 mm from the
edge (KiCad (217.8, 95.825)), the front shell-tab copper would end 0.17 mm from the edge. So the front tab
pads are custom shapes cut flat CU_EDGE_CLR before the board edge (the ring at the slot's front end becomes
~0.07 mm): the board shows exactly what gets made. Re-generate if J2 moves (EDGE_Y).
Geometry from the datasheet C-DS1144-07-XXXXXXX-A (tab pad lengths drill + 0.4: not dimensioned).
"""
import math
import uuid
U=lambda: str(uuid.uuid4())
out=[]
def line(x1,y1,x2,y2,layer,w):
    out.append(f'''	(fp_line
		(start {x1:g} {y1:g})
		(end {x2:g} {y2:g})
		(stroke
			(width {w:g})
			(type solid)
		)
		(layer "{layer}")
		(uuid "{U()}")
	)''')
def rect(x1,y1,x2,y2,layer,w):
    line(x1,y1,x2,y1,layer,w); line(x2,y1,x2,y2,layer,w); line(x2,y2,x1,y2,layer,w); line(x1,y2,x1,y1,layer,w)
def text(t,x,y,layer,size=1,th=0.15):
    out.append(f'''	(fp_text user "{t}"
		(at {x:g} {y:g} 0)
		(layer "{layer}")
		(uuid "{U()}")
		(effects
			(font
				(size {size:g} {size:g})
				(thickness {th:g})
			)
		)
	)''')
EDGE_Y = 2.175            # board edge in footprint coordinates (J2 origin to the board's front edge)
CU_EDGE_CLR = 0.3         # copper-to-edge clearance kept by the front tab pads
# geometry (local; +y towards the board edge / mouth). Signal pad row at y -3.67 like XKB.
PY=-3.67
FRONT_TAB_Y=PY+4.575      # DS: front slot centre 5.15 from pad rear edge (=PY-0.575)
REAR_TAB_Y=PY+0.575       # DS: rear slot 4.00 behind front slot
FACE_Y=FRONT_TAB_Y+2.10   # DS: front face = 'P.C.B EDGE', 2.10 ahead of front slot
BODY_REAR=FACE_Y-6.50
FAB_W=4.47
head=f'''(footprint "USB_C_DS1144-07"
	(version 20260206)
	(generator "pcbnew")
	(generator_version "10.0")
	(layer "F.Cu")
	(descr "USB Type C receptacle, 16P, top-mount SMT, L=6.5 mm, Connfly DS1144-07 (C-DS1144-07-XXXXXXX-A). Signal pad row placed like USB_C_XKB_U262-16XN-4BVC11_noSBU (drop-in for routing); SBU pads A8/B8 omitted (USB 2.0 only, easier hand soldering). Datasheet PCB edge (front face flush) is drawn on Dwgs.User.")
	(tags "USB C Type-C Receptacle SMD Connfly DS1144")
	(property "Reference" "REF**"
		(at 0 -5.715 0)
		(layer "F.SilkS")
		(uuid "{U()}")
		(effects
			(font
				(size 1 1)
				(thickness 0.15)
			)
		)
	)
	(property "Value" "USB_C_Receptacle_Connfly_DS1144-07"
		(at 0 {FACE_Y+1.2:g} 0)
		(layer "F.Fab")
		(uuid "{U()}")
		(effects
			(font
				(size 1 1)
				(thickness 0.15)
			)
		)
	)
	(property "Datasheet" ""
		(at 0 0 0)
		(unlocked yes)
		(layer "F.Fab")
		(hide yes)
		(uuid "{U()}")
		(effects
			(font
				(size 1.27 1.27)
				(thickness 0.15)
			)
		)
	)
	(property "Description" ""
		(at 0 0 0)
		(unlocked yes)
		(layer "F.Fab")
		(hide yes)
		(uuid "{U()}")
		(effects
			(font
				(size 1.27 1.27)
				(thickness 0.15)
			)
		)
	)
	(attr smd)
	(duplicate_pad_numbers_are_jumpers no)'''
# silk: side lines between the tab pads (pads x 3.82..4.82)
SX=4.58
r_end=REAR_TAB_Y+0.9+0.2; f_start=FRONT_TAB_Y-1.1-0.2
for s in (-1,1):
    line(s*SX, round(r_end,3), s*SX, round(f_start,3), "F.SilkS", 0.12)
# courtyard
cy1=PY-0.575-0.5; cy2=FACE_Y+0.5
rect(-5.32, round(cy1,2), 5.32, round(cy2,2), "F.CrtYd", 0.05)
# fab body
rect(-FAB_W, round(BODY_REAR,3), FAB_W, round(FACE_Y,3), "F.Fab", 0.1)
# fab: receptacle opening 8.34 wide at the front face (as a line marker)
line(-4.17, round(FACE_Y-0.3,3), 4.17, round(FACE_Y-0.3,3), "F.Fab", 0.1)
text("${REFERENCE}", 0, round(BODY_REAR/2+FACE_Y/2,3), "F.Fab")
# datasheet PCB edge
line(-5.6, round(FACE_Y,3), 5.6, round(FACE_Y,3), "Dwgs.User", 0.1)
text("DS PCB EDGE (face flush)", 0, round(FACE_Y+0.6,3), "Dwgs.User", 0.5, 0.08)
pads=[("A1",-3.2,0.6),("A4",-2.4,0.6),("A5",-1.25,0.3),("A6",-0.25,0.3),("A7",0.25,0.3),("A9",2.4,0.6),("A12",3.2,0.6),
      ("B1",3.2,0.6),("B4",2.4,0.6),("B5",1.75,0.3),("B6",0.75,0.3),("B7",-0.75,0.3),("B9",-2.4,0.6),("B12",-3.2,0.6)]
for n,x,w in pads:
    out.append(f'''	(pad "{n}" smd roundrect
		(at {x:g} {PY:g})
		(size {w:g} 1.15)
		(layers "F.Cu" "F.Mask" "F.Paste")
		(roundrect_rratio 0.25)
		(uuid "{U()}")
	)''')
for s in (-1,1):                           # rear tabs: plain oval pads
    out.append(f'''	(pad "SH" thru_hole oval
		(at {s*4.32:g} {round(REAR_TAB_Y,3):g})
		(size 1 1.8)
		(drill oval 0.6 1.4)
		(property pad_prop_mechanical)
		(layers "*.Cu" "*.Mask")
		(remove_unused_layers no)
		(uuid "{U()}")
	)''')
# front tabs: 1.0 x 2.2 oval ring, cut flat at y = EDGE_Y - CU_EDGE_CLR (pad-relative coordinates)
cut = round(EDGE_Y - CU_EDGE_CLR - FRONT_TAB_Y, 4)
r, c = 0.5, 0.6                            # end radius, end-centre offset (2.2 = 2*0.6 + 2*0.5)
t1 = math.asin(min(1.0, (cut - c) / r))    # front arc reaches the cut at angle t1 (and pi - t1)
pts = []
for i in range(0, 11):                     # front arc from angle 0 up to t1
    t = t1 * i / 10; pts.append((r * math.cos(t), c + r * math.sin(t)))
for i in range(0, 11):                     # ... then from pi - t1 to pi (the flat cut joins them)
    t = math.pi - t1 + t1 * i / 10; pts.append((r * math.cos(t), c + r * math.sin(t)))
for i in range(0, 19):                     # rear arc, pi .. 2 pi
    t = math.pi + math.pi * i / 18; pts.append((r * math.cos(t), -c + r * math.sin(t)))
poly = " ".join(f"(xy {x:.4f} {y:.4f})" for x, y in pts)
for s in (-1,1):
    out.append(f'''	(pad "SH" thru_hole custom
		(at {s*4.32:g} {round(FRONT_TAB_Y,3):g})
		(size 0.6 0.6)
		(drill oval 0.6 1.8)
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
		(uuid "{U()}")
	)''')
# our board edge (with J2 at its placement)
line(-5.6, EDGE_Y, 5.6, EDGE_Y, "Dwgs.User", 0.1)
text("BOARD EDGE (J2 placement)", 0, round(EDGE_Y+0.4,3), "Dwgs.User", 0.5, 0.08)
tail='''	(embedded_fonts no)
	(model "${KIPRJMOD}/3d/DS1144-07.step"
		(offset
			(xyz 0 0 0)
		)
		(scale
			(xyz 1 1 1)
		)
		(rotate
			(xyz 0 0 0)
		)
	)
)
'''
open("/home/mars/my/buttheat/ButtHeatCarrier/ButtHeat.pretty/USB_C_DS1144-07.kicad_mod","w").write(head+"\n"+"\n".join(out)+"\n"+tail)
print("front tab copper cut at local y", EDGE_Y - CU_EDGE_CLR, "slot ends at", round(FRONT_TAB_Y + 0.9, 3), "-> front ring", round(EDGE_Y - CU_EDGE_CLR - FRONT_TAB_Y - 0.9, 3))
