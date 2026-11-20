"""Load generated v2 case shapes (hw/cad/out/*.brep) into the open ButtHeatModel document, group "V2 Case".

Run in the GUI (MCP execute_code):  exec(open('/home/mars/my/buttheat/hw/cad/v2_load.py').read(), {})
Only reads BREP files and assigns shapes: no booleans in the GUI process.
"""
import glob
import os
import FreeCAD as App
import Part

LABELS = {'RearPlate': 'Rear plate, FDM (heat-set inserts)',
          'RearPlate_SLA_L': 'Rear plate SLA, left half (nuts)', 'RearPlate_SLA_R': 'Rear plate SLA, right half (nuts)',
          'Middle': 'Front middle (SLA)', 'Left': 'Front left (SLA)', 'Right': 'Front right (SLA)',
          'ClampTop': 'Cable clamp insert, top half (SLA)', 'ClampBottom': 'Cable clamp insert, bottom half + tab (SLA)',
          'J1Spacer': 'J1 (S4B-ZR) spacer, WROOM builds', 'J1WireCurl': 'J1 wire curl (reference, not printed)'}
doc = App.getDocument('ButtHeatModel')
grp = doc.getObject('V2_Case') or doc.addObject('App::DocumentObjectGroup', 'V2_Case')
grp.Label = 'V2 Case'
for f in sorted(glob.glob('/home/mars/my/buttheat/hw/cad/out/*.brep')):
    name = os.path.splitext(os.path.basename(f))[0]
    sh = Part.Shape(); sh.read(f)
    o = doc.getObject(name)
    if o is None:
        o = doc.addObject('Part::Feature', name)
        grp.addObject(o)
    o.Label = LABELS.get(name, name)
    o.Shape = sh
    print('loaded', name, round(sh.Volume, 1))
