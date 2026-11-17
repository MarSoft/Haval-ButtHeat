# kcaa (KiCad MCP) issues found while building ButtHeatCarrier

Environment: KiCad 10.0.4 on NixOS, board/schematic files in KiCad 10 format
(`(version 20260206)` board, `(version 20260306)` schematic). Coordinates below are KiCad board coords.
Ordered roughly by severity.

## Schematic

### S1. Wrong instance path for symbols on sub-sheets (blocker)
- `add_symbol_to_schematic` on a child sheet writes
  `(instances (project "X" (path "/<child-file-uuid>" (reference "SW1") (unit 1))))`.
- KiCad expects `/<root-sheet-uuid>/<sheet-symbol-uuid>` (the `uuid` of the `(sheet …)` block in the
  parent), recursively for deeper levels. With the wrong path the symbol is effectively unannotated
  in the hierarchy.
- Repro: root `ButtHeatCarrier.kicad_sch` (uuid 48c8f88e-…), `add_sheet_symbol(create_child=true)` →
  `controls.kicad_sch` (sheet uuid e0460a9c-…, child file uuid 7264e522-…); place a symbol in the
  child → path is `/7264e522-…` instead of `/48c8f88e-…/e0460a9c-…`.

### S2. Renaming via `set_symbol_property(Reference)` doesn't update the instance reference (blocker)
- Only the visible `(property "Reference" …)` changes; `(instances … (reference "SW1"))` keeps the old
  value. `kicad-cli sch export netlist` still reports the old refs (SW1…SW7 instead of ENC1… / SW1…).
- Related: `add_symbol_to_schematic` has no `reference` argument, so every placement needs a rename.
  Adding a `reference` parameter would avoid most renames.

### S3. No way to set DNP / exclude-from-BOM / exclude-from-board flags
- `(dnp yes)` etc. are symbol flags, not properties; `set_symbol_property` can't set them.

## PCB — footprints

### P1. Rotated placement doubles pad rotation (blocker for any non-0° part)
- `add_footprints_to_pcb` and `set_footprint_position` (with `rotation`) write pad angles that already
  include the footprint rotation (absolute, correct for KiCad), but put the footprint's own `(at x y rot)`
  **after** the pads/properties. KiCad 10 then applies the footprint rotation to the pads again.
- Repro: ESP32-C3-WROOM-02 at rotation 90 → pads end up at 180°, adjacent pads overlap
  (DRC clearance / solder-mask-bridge errors between pads 1–2, 2–3, …).
- Fix: emit `(at …)` right after `(layer …)` / before any child items (or store pad angles relative
  if the header comes later — but match what pcbnew writes).

### P2. `add_footprints_to_pcb` copies library UUIDs
- Several instances of the same library footprint get identical UUIDs (footprint items, properties).
- Repro: place three `RotaryEncoder_Alps_EC12E-Switch_Vertical_H20mm` → identical UUIDs; DRC then
  attributes violations to the wrong footprint (e.g. "ENC3 pad B @207.5,86" reported against U2 pads
  that were actually near ENC1).
- Fix: regenerate every `uuid` on placement.

### P3. `flip_footprint` only half flips
- Moves pads/mask/paste/silk/fab to B.*, but:
  - leaves `fp_rect` courtyards on `F.CrtYd`,
  - doesn't mirror local X of pads/graphics (pad 1 stays at −x),
  - doesn't set text `(justify mirror)` → DRC `nonmirrored_text_on_back_layer` for every field.
- pcbnew's own flip (F) on such a footprint just flips "as is", so the damage can't be repaired in the
  GUI; the parts had to be removed and re-added on the front, then flipped in the GUI.

### P4. `set_footprint_position` / `move_footprints_by_delta` collision guard
- `move_footprints_by_delta` refuses on courtyard overlap with no `force` option (had to use
  `set_footprint_position(force=true)` for a 0.2 mm nudge).

### P5. Nets for pads with empty number
- `add_footprints_to_pcb` requires a `nets` entry for pad `""` (NPTH / paste-only pads), e.g.
  `MountingHole_2.2mm_M2`, WROOM paste pads, USB-C NPTH pegs → must pass `{"": ""}`. Should default
  unnumbered pads to net 0.

## PCB — graphics / zones / setup

### G1. No Edge.Cuts circles
- `add_board_outline_arc` with 0…360° writes start == end (degenerate arc). Needs a circle tool
  (or arc 0…360 → `gr_circle`).

### G2. No deleting a single graphic item
- Only `clear_board_outline` (all Edge.Cuts). Changing one notch means redrawing ~30 items.
  Need delete/modify by UUID (and `get_board_outline` should return UUIDs).

### G3. `add_zone` writes an invalid keepout for KiCad 10
- Emits `(keepout … (text allowed))`; KiCad 10 rejects it: `expected tracks, vias or copperpour. Got
  "text"` → board fails to load in pcbnew (kicad-cli DRC/netlist also fail silently-ish).
- Repro: `add_zone(zone_type="keepout", keepout_footprints="not_allowed", …)` on F.Cu.

### G4. No setters for board setup
- Grid origin, drill/place (aux) origin, board stackup thickness can't be set.

## Libraries / indexing

### L1. Nested fp-lib-table / sym-lib-table `(type "Table")` (KiCad 10 default user table)
- Initially no stock footprints resolved (`Library 'Connector_JST' not found`). Fixed by the user
  ("library path" issue) — after that, `sync_footprint_index force=true` was needed.
- Symbol index was also empty until `sync_symbol_index force=true` (took ~7 min for 222 libs).
  Worth syncing automatically when the index is empty.

## GUI interaction

### I1. `reload_kicad` reports success without effect
- Twice the IPC revert returned `success: true` while pcbnew kept showing the old board (once while
  the board file had the invalid zone from G3; once with no obvious cause). File → Revert in pcbnew
  also did nothing; closing and reopening the board worked.

### I2. File edits under an open pcbnew are unsafe
- The tools edit the file on disk. If pcbnew has the board open, a later GUI save silently overwrites
  the MCP's changes, and once pcbnew crashed after the file changed under it.
- Suggestion: perform edits through the IPC API (kipy) when pcbnew has the board open, or at least
  refuse/warn when the board's `.lck` exists.

### I3. `run_drc_check` only opens the dialog
- No results returned; had to use `kicad-cli pcb drc --format json` instead. A tool returning the
  JSON report would be more useful.

## mixelpixx kicad server (found while redrawing power.kicad_sch)

### M1. Project symbol library written by kcaa not found until upgraded
- `batch_list_symbol_pins` / placement reported `Symbol 'ButtHeat:MP2307' not found` although the
  project sym-lib-table resolved and `validate_symbol_library` said valid. The file was in the old
  `(version 20220914)` format (as written by kcaa); after `kicad-cli sym upgrade` it worked.

### M2. Power symbols placed with visible references
- `batch_add_components` with `power:GND`, `power:+5V`, `power:PWR_FLAG` etc. leaves the `#PWRnn` /
  `#FLGnn` Reference fields visible. KiCad hides them by default; had to hide 23 one by one
  (`batch_set_schematic_property_positions` has `visible`, but needs a position for each).

### M3. No bulk wire delete / "clear sheet wiring"
- Re-spacing a sheet meant 51 single `delete_schematic_wire` calls. `move_schematic_component`
  (preserveWires) only drags wire ends that sit on that symbol's pins; free junction points between
  wires stay put, so moving a whole group distorts the wiring — a group move or a bulk delete would help.

### M4. `add_schematic_wire` "floating" warnings for intentional junction points
- Every endpoint that is a deliberate junction / power-symbol point (not a pin) produces a
  "may be floating" warning. Noise; a flag to suppress, or checking against other wire endpoints and
  power-symbol pins, would make real problems visible.

### M5. Custom-symbol pin grid (kcaa)
- Symbols created by kcaa `create_symbol` (MP2307, XC6206, SuperMini) have pins at ±5.715 mm, off the
  1.27 mm grid → ERC `endpoint_off_grid` warnings on every wire to them, and small bodies whose pin
  names overlap (U6). Fix in the library (pins at ±7.62, larger bodies), not a mixelpixx issue.

### M6. `update_symbol_from_library` leaves connections dangling when pins move
- After a symbol is redrawn with different pin positions, the update just swaps the cached definition;
  wires/labels that ended on the old pin tips are left dangling (here: all 16 global labels of U2 on
  mcu.kicad_sch, and every U4/U6 wire on power.kicad_sch). KiCad's own "Update symbol" behaves the same,
  but an option to re-snap attached labels/wire ends to the new pin positions would help.

### M7. `autoplace_schematic_fields` worsens dense areas
- On the Power sheet it put J1's value on top of the CANH global label and stacked R4/C3 and L1's fields
  onto each other/wires. Manual `batch_set_schematic_property_positions` was needed afterwards.

### M8. Field position semantics on rotated symbols are unclear
- `batch_set_schematic_property_positions` x/y/angle on 90/180/270°-rotated symbols (R4, C7, D3, L1)
  doesn't land where an absolute coordinate would put it, and `angle: 0` gives vertical text on a
  270°-rotated part — positions had to be found by trial and render.

### M9. `get_schematic_view` is the only working rasteriser here
- Not an MCP bug: on this NixOS host neither cairosvg nor ImageMagick SVG rendering is available, so the
  tool's PNG output is the only way to look at a sheet.

### M10. No way to delete or move no-connect markers / free text
- mixelpixx has `add_no_connect` / `batch_add_no_connects` but no delete or move; deleting a part
  leaves its NC flags orphaned at the old pin positions (MCU sheet: J2 had to be re-placed at its
  old position so its SBU markers re-attached). Same for `add_schematic_text`: no move/delete.

### M11. Field angles on rotated symbols are relative
- For a symbol placed at rotation 90/270, `batch_set_schematic_property_positions` with the
  default `angle: 0` renders the Reference/Value text vertically; `angle: 90` is needed for
  horizontal text (seen on rotated power symbols and R13/R14 on the MCU sheet). Related to M8.

### M12. `delete_schematic_net_label` is one label per call
- No batch delete (kcaa's `delete_label_from_schematic` has `positions`); clearing 16 stale labels
  took 16 calls.

- **M13 add_schematic_text: no multi-line, no delete.** A text containing `\n` is written with a literal
  backslash-n (renders as "…closed.\nSN65…" on one line). Since there is no tool to delete/edit free text
  (see M10), the bad note on can.kicad_sch (at 73.66, 76.2, starting "TJA1051T/3: JP1 1-2 (5V), JP2
  closed.\nSN65…") had to be left in place; it must be deleted in eeschema.
- **M14 field justify on rotated symbols is mirrored.** For a symbol placed at rotation 180,
  `fieldPositions.justify: "right"` renders left-justified (and vice versa); rotation 90/270 fields also
  need angle 90 to read horizontally (cf. M11).

### M15 (display sheet). add_schematic_wire warns "may be floating" for every endpoint that lands on another wire's endpoint/corner (T-junctions) even though KiCad connects them fine — noisy; it only checks pins, not wire endpoints.
### M16 (display sheet). batch_set_schematic_property_positions: fields are rendered left-justified from the given x (no justify option in the batch tool, only in edit_schematic_component), so "centre under the part" needs manual x offsets.

### Found while setting up rules (main session)
- **mixelpixx `create_netclass`**: creates the class in .kicad_pro but does not persist the `nets`
  assignment (no `netclass_patterns`); warns "in-memory NETCLASS update failed: 'NETINFO_ITEM' object
  has no attribute 'SetClass'". **`assign_net_to_class`** then fails with "Net class not found" even
  after `open_board` (session never re-reads net classes from .kicad_pro). Workaround: kcaa
  `assign_nets_to_class`.
- **mixelpixx stackup**: `get_stackup` / `set_stackup_layer` fail on a board without a `(stackup …)`
  section ("Board has no stackup"); no tool creates one or sets the board thickness.
- **mixelpixx `add_copper_pour`** without `outline` uses a 4-point bounding rectangle, not the board
  outline (works because KiCad clips to Edge.Cuts, but surprising).
- **kcaa `add_custom_rule`** writes a malformed `.kicad_dru`: `version` and `1` on separate lines
  instead of `(version 1)`, rule name with backslash-escaped spaces instead of a quoted string.
- **kcaa `set_design_rules`** rejects valid KiCad keys (`min_via_diameter`,
  `min_through_hole_diameter`, `min_hole_to_hole`, `min_hole_clearance`, `min_copper_edge_clearance`).

### Found while reviewing display/CAN routing (board open in pcbnew, IPC backend)
- **mixelpixx `get_component_list` / `query_traces` return nothing on the IPC backend** — with the
  board open in pcbnew (`get_backend_state`: ipc, board loaded) both return `count 0` with or without
  a bounding box / layer / net filter, although the board has 62 footprints and many tracks. Tools
  that fall back to SWIG (`get_component_pads`, `get_ratsnest`) work, but read the file on disk, not
  the live board.

### FreeCAD MCP (found 2026-11-15)
- **`execute_code` always crashes in its sandbox**, even for `print(1+1)`: a freecadcmd backtrace from libFreeCADApp.so (FreeCAD 1.1.3, NixOS). The structured tools (`read_variable_set`, `edit_variable_set`, `read_spreadsheet`, `edit_spreadsheet`, `describe_model`, `get_document_state`) work.
- `describe_model` on a PartDesign feature (e.g. an AdditiveCylinder) returns the whole body's cumulative shape, not the feature's own parameters (radius, placement). There's no read-only way to get a feature's parameters or a sketch's geometry list without `execute_code`.

### mixelpixx `add_board_text` over IPC is lost (2026-11-15)
- With the board open in pcbnew (IPC backend), `add_board_text` on B.SilkS returned `success: "Added text … (visible in KiCAD UI)"`, but the text never appeared in the GUI and wasn't in the file after the user saved.
- Over SWIG with the board closed, it worked but auto-save was refused ("on-disk PCB changed externally"), so the change was dropped silently unless `open_board` was re-run first.
- Workaround: close the board, `open_board`, add the text, `save_board`, and check the text is in the file.
- 2026-11-15 update: FreeCAD `execute_code` works again.
- mixelpixx `add_board_text` has no mirror/justify option: text on B.SilkS is written without `(justify mirror)`, so it reads backwards from the back. Fix by hand: *Properties → Mirrored*.
- FreeCAD `edit_sketch`: the `expression` field in `add_constraints` is accepted but not bound; bind afterwards with `set_expression(sketch, 'Constraints[N]', …)`.
- mixelpixx `batch_move_components` with `layer: B.Cu`: the `rotation` isn't applied as an absolute value when the part is already on B.Cu. The same call twice (rotation 180) gave two different pad layouts, and the response reported rotation 0 the first time and 180 the second. Always check pads with `get_component_pads` after reloading.
- mixelpixx backend mixing (2026-11-15): with only the KiCad project manager running (no lock file, PCB editor closed), `open_board` picked the IPC backend. `update_graphic` then edited the SWIG copy ("_backend: swig"), while `refill_zones`/`save_board` went over IPC and reported success. The edits never reached the file. Workaround: quit KiCad completely before MCP edits, and check the file afterwards.
- FreeCAD MCP `execute_code` (2026-11-15, after importing the 371-solid board STEP into ButtHeatModel): every call times out (30 s), even `print(len(d.Objects))`. The sandbox appears to reload the document per call. Adding the board shape via `execute_code` also timed out. Workaround: import big STEPs through the GUI; do heavy geometry checks with a local `freecadcmd` script (read-only, no time limit).
