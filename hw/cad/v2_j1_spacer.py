"""ButtHeat v2: spacer under J1 (JST ZH side-entry S4B-ZR) for the WROOM builds.

The S4B is soldered 1.09 mm above the board (body bottom z 4.59) so it clears the SuperMini (top z 4.5),
on which it then rests. With the WROOM there's nothing under it, and plugging pushes it sideways (along
y) on its four thin pins, so this spacer goes under it before soldering: board top (z 3.5) to the header's
underside, footprint = the header body over the board, 0.2 mm clear of the WROOM module's edge (y 22.4)
and inside the board edge (y 28.0), with slots for the pins and their top-side solder fillets.

Run headless; writes hw/cad/out/J1Spacer.brep (case coordinates).
"""
import FreeCAD as App
import Part

OUT = '/home/mars/my/buttheat/hw/cad/out/'
P = App.Vector

BOARD_TOP, HEADER_BOTTOM = 3.5, 4.59
X0, X1 = -73.55, -66.05                 # header body
Y0, Y1 = 22.4 + 0.2, 28.0 - 0.1         # WROOM edge + clearance .. board edge
PINS_X, PIN_Y = (-72.05, -70.55, -69.05, -67.55), 26.70
SLOT_W, SLOT_L = 1.4, 1.8                # around the 0.5 mm pins + fillets, a little play along y

s = Part.makeBox(X1 - X0, Y1 - Y0, HEADER_BOTTOM - BOARD_TOP, P(X0, Y0, BOARD_TOP))
for x in PINS_X:
    slot = Part.makeBox(SLOT_W, SLOT_L - SLOT_W, 3, P(x - SLOT_W / 2, PIN_Y - (SLOT_L - SLOT_W) / 2, BOARD_TOP - 1))
    for dy in (-(SLOT_L - SLOT_W) / 2, (SLOT_L - SLOT_W) / 2):
        slot = slot.fuse(Part.makeCylinder(SLOT_W / 2, 3, P(x, PIN_Y + dy, BOARD_TOP - 1)))
    s = s.cut(slot)
s = s.removeSplitter()
s.exportBrep(OUT + 'J1Spacer.brep')
print('J1Spacer', s.isValid(), round(s.Volume, 2), s.BoundBox)
