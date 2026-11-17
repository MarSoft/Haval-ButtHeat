# ButtHeat v2 hardware spec

Single source of truth for the v2 (carrier-PCB) hardware. FreeCAD (`3d/ButtHeatModel.FCStd`,
VarSet `V2Vars`) holds the driving numbers, KiCad (`ButtHeatCarrier/`) is the implementation;
when either disagrees with this file, fix one of them and note it here.

Project: car climate controller (seat heat / fan), ESP32-C3 + CAN + 0.91" OLED + 3 encoders.
v2 = FDM PETG rear plate + 3-piece SLA (ABS-like) front + one carrier PCB.
Prototype (v1) parts `CaseBase` / `CaseCover` / `CarBase` stay in the model as fit references.

## 1. Coordinates

- **Case origin (0,0)** = centre of the car's knob. XY = plane of the car panel (almost
  vertical in the car), X along the case length (0 = centre), +Y up, +Z out of the panel
  towards the user.
- Prototype case bottom edge is y = 0 (knob centre line), top edge y = 29.75, ends x = ±106 (ears).
- **KiCad**: case (x, y) = KiCad (150 + x, 100 − y). Grid origin and Drill/Place origin are both
  (150, 100), so STEP export with "use drill/place origin" lands in FreeCAD coordinates.
  - To *display* case coordinates: Preferences → PCB Editor → Origins & Axes →
    origin = grid origin, Y axis increases upward.

## 2. Z stack (middle of the case, at the display)

| z (mm)   | what                                           |
|----------|------------------------------------------------|
| 0 … 2.0  | rear plate (PETG)                              |
| 2.0 … 2.5| gap (0.5 mm nubs on the plate)                 |
| 2.5 … 3.5| carrier PCB, **1.0 mm** (not thinner: rigidity; JLCPCB option)  |
| 3.5 … 4.8| display glass lying on the carrier (1.2 + 0.1 film)             |
| 4.8 … 5.0| clearance                                                        |
| 5.0 … 6.2| SLA front skin 1.2 → **6.2 mm** total over the display           |

- Prototype is **6.8 mm tall over the display and that already gets in the way of gripping the
  knob**. v2 must not be taller there, ideally lower, with a good facet at the edge just
  below the display.
- Facet (middle): rim at the scallop (y 15.5) ≈ 4.3 (board edge + ~0.8 skin) → glass bottom edge
  (y 16.5) ≥ 5.5 → full height at the window edge (y ≈ 19.2, 0.3 before the first pixel row).
  Alternative: whole middle zone 0.8 mm skin (~5.8) with a bevelled window edge.
- Side zones (away from the knob) may go up to 12.4 mm.

## 3. Board outline

- Rectangle x −73 … +73, y 2 … 28 (wall 2.0 all round), 2-layer FR4.
- **Knob scallop** (bottom middle): follows the prototype `CaseBase` shape, not the old
  single-arc approximation. Prototype case curve (per side, mirrored about x = 0):
  - convex arc R29.8, centre (0, −14.305), apex (0, 15.495) → (±18.929, 8.712)
  - 3.0 mm tangent line → (±21.231, 6.788)
  - concave arc R29.8, centre (±40.166, 29.8), tangent to y = 0 at x = ±40.166
- Board follows that curve **in line with the case over the convex arc** (no wall over the knob;
  8-segment tangent polygon, never protrudes, ≤ 0.11 mm inside) and **offset 2 mm (wall) on the
  concave flanks** (2 chords each, ≤ 0.41 mm extra cut). One short segment per side joins
  (±22.508, 8.329) on the offset flank to (±18.929, 8.712) on the case arc.
  Generator: `hw/tools/scallop_outline.py` (prints KiCad segments/arcs).
- **Flex slot** (display option 1): x 16.8 … 18.8, y 12.4 … 25.0 (2 × 12.6, R1 ends) — glass
  (11.5 × 1.3) threads through with ≈ 0.3 mm spare per end; the flex (7.8 wide) passes within it.
  Board left: 3.0 above, ≈ 2.1 below.
- Post notches Ø7.5 (case posts Ø6.5, M3 heat-set inserts):
  (−48, 26) top, **(−33, 5)** bottom (moved from (−35, 4), §“Post 3 moved”), **(58, 4)** bottom
  (moved from (66, 4) for J2).
  Post 2 (78.5, 24) is beside the board on the ear.
- Encoder **shaft positions** (markers only, not holes): (−43, 11), (43, 11), (65, 15).
- M2 fallback mounting holes Ø2.2 at (±27, 13) — footprints H1/H2 (`MountingHole_2.2mm_M2`).
- Glass rule area (F.Cu, no footprints) drawn by hand in the GUI (MCP `add_zone` writes an
  invalid keepout for KiCad 10).

## 4. Components

### J1 — power/CAN connector
- **JST-ZH 4-pin vertical THT, B4B-ZR, 1.5 mm pitch** (`Connector_JST:JST_ZH_B4B-ZR_1x04_P1.50mm_Vertical`),
  rotated 180°: pin 1 (+12 V) at case (−67.55, 26.70), pins 2/3/4 (GND, CAN+, CAN−) towards the
  left edge (x −69.05 / −70.55 / −72.05). Body x −73.55 … −65.74, y 25.09 … 28.9: overhangs the
  top edge by 0.9 mm and the left edge by ~0.5 mm, clears the SuperMini body by 0.29 mm, sits
  inside U1's antenna keep-out courtyard (DRC-excluded; antenna-WROOM builds don't fit J1).
- Plug inserts **from the front**; the case is adapted (top-left cable exit, split clamp, rib over
  the plug) — see §“J1 = JST ZH vertical (2026-11-16)” for measurements and cable routing.
- Replaces the side-entry JST-PH S4B-PH-K (its mouth faced the car's air-vent wall — no room for
  the plug).
- Pinout: 1 = +12 V, 2 = GND, 3 = CANH, 4 = CANL.

### MCU — exactly one of two variants is fitted (same PCB)
- **A: ESP32-C3 SuperMini** (U2), footprint `ButtHeat:ESP32-C3-SuperMini`
  (generated by `hw/tools/gen_supermini_fp.py`; pads named by function, standard silkscreen:
  USB end → 5V, GND, 3V3, IO4, IO3, IO2, IO1, IO0 on the left / IO5 … IO10, IO20, IO21 on the
  right, top view USB down). Module x −69.5 … −51.5, y 2.0 … 24.8, centre (−60.5, 13.4), USB-C
  face at y 0.6. Antenna keep-out (no copper, both layers) on the top 6 mm. Position is boxed in:
  post 1 (top-right corner 0.45 from the post), ENC1, board edge. Uses its own 3V3 LDO.
  Its USB is **not usable in the car** (cable hits the car buttons) — flash on the bench / OTA.
- **B: ESP32-C3-WROOM-02U** (U1), **rotated 90°** and centred on the SuperMini: its pad rows
  (y = 13.4 ± 8.75, x −66.95 … −54.05) never touch the SuperMini pad columns (≥ 0.75 mm).
  **Three build variants** (the U1 footprint is the WROOM-02's; a WROOM-02U uses the same pads):
  | Variant | Module | Antenna | J1 |
  |---|---|---|---|
  | A | SuperMini (U2) | its own | fitted |
  | B1 | WROOM-02 (U1) | on-board, covers J1 pins 3/4 | **not fitted**: cable soldered into J1's holes from the back, trimmed flush on top before the module goes on |
  | B2 | WROOM-02U (U1) | u.FL pigtail or none | fitted (the shorter module clears it) |
  Placed as **WROOM-02 (on-board antenna)**, anchor at (−59.7, 13.4), antenna end pointing left
  and **inside the board** (module x −72.8 … −52.8; nothing may protrude past the board/case
  edge — the car's air ducts are there). Its pad rows clear the SuperMini pads by ≥ 0.55 because
  the SuperMini carrier pads reach only 1.2 mm inside the module edge. Wi-Fi/BLE are not used,
  so its antenna keep-out (J1, SuperMini pads inside it) is advisory — DRC exclusions;
  WROOM-02U + u.FL antenna is the fallback if radio is ever needed.
  Own LDO + EN/BOOT parts. Separate USB-C for debug/flash that **must sit at the far left or
  far right bottom corner** (car buttons: x ±26 … ±65, y −24 … −33, 10 mm tall; real-world check:
  a cable fits at the board's extreme ends). Data lines may be long (USB FS).
  Bottom-left is impossible: the receptacle's signal-pad row (y ≈ 8 … 9.5) overlaps the SuperMini
  5V/GND/3V3 pads in copper → **bottom-right**, see §6.

### Encoders
- Currently EC12S, 24 detents / 24 pulses, 14 mm, **with push switch**, D-shaft 6 × 14 mm.
  Spares have no thread (front-shell compression + foam), M2 fallback holes exist.
  Model may change — keep footprints generic where possible.
- ENC1 (−43, 11) driver temp, ENC2 (43, 11) passenger temp, ENC3 **(65, 17)** fan/mode (moved up
  2 mm for J2; its knob must still clear the car's display frame above — ≤ 2 mm allowed).
- Footprint: stock `Rotary_Encoder:RotaryEncoder_Alps_EC12E-Switch_Vertical_H20mm` — matches the
  hand-measured FreeCAD model (A/B/C at −7.5 from the shaft, switch pins at +7.0, tabs ±5.7).
  ENC1 and ENC3 **rotated 270°** (A/B/C row 7.5 above the shaft, switch pins 7.0 below, tabs
  left/right): keeps ENC1 1.15 mm clear of the SuperMini pads and ENC3's side tabs 1.15 mm from
  the edge. ENC2 at **0°** (A/B/C left, switch right, tabs above/below) to free room for the buck.

### Buttons (optional, DNP by default)
- Secondary option to ENC3. Leave footprints for **both SMD and THT** 6×6 tact switches at
  (−27, 19) and (27, 19). Rule: PCB orders are slow/expensive → make the board universal.

### Display — 0.91" 128×32 SSD1306 module (flex label GME12832-0L V3)
- Module board 38 × 12 × 1.0, glass 30.0 × 11.5 × 1.3 (measured; incl. polariser + removable film).
- Flex pitch 0.62 mm (14 pins). Pinout (vendor sheet, checked): 1 C2P, 2 C2N, 3 C1P, 4 C1N,
  5 VBAT, 6 VBREF, 7 VSS, 8 VDD, 9 RES#, 10 SCL, 11 SDA, 12 IREF, 13 VCOMH, 14 VCC.
- **Vendor geometry wins over the FreeCAD model**: active area 22.384 × 5.584 at 2.1 from the
  left / 2.1 from the top glass edge; visible area 24.384 × 7.584 at 1.1 / 1.1. Borders: left 2.1,
  right 5.516 (COG/flex side), top 2.1, bottom 3.816 (assumes the vendor drawing has the flex on
  the right, as mounted). Active area centred at x = 0 → glass x −13.292 … 16.708.
- Flex: 7.8 mm wide (9.2 mm at the solder area), ~7.5 mm free length glass edge → solder area,
  soldered under the glass, 14 pins. Flex wraps around the right end of the module.
- Driver board parts: Q1 "662K" (XC6206-class 3.3 V LDO), D1, C1…C9, R1 = 560k (564),
  R2…R4 = 4.7k (472), R5 not fitted.
- FreeCAD part `ssd1xxx display` is trustworthy for glass, active area and rear components
  (checked against real measurements); its flex is simplified (7.0 wide, no wrap) — use the
  numbers above for the flex.
- Active area (`TheScreen` Face17): 23.2 × 6.5, glass-local x 7.8 … 31.0 on glass 6 … 36;
  inactive border 3.0 at the bottom (knob side), 1.5 at the top.
- v2 glass position: x −13.292 … 16.708, **y 16.25 … 27.75** → active area x −11.192 … 11.192,
  y 20.07 … 25.65. KiCad rule area (F.Cu, no footprints, tracks/vias allowed) x −13.5 … 16.8,
  y 16.05 … 27.95.
- Mounting (decided): glass unglued and lying **on** the carrier, active area centred at x = 0.
  The board supports two variants:
  - **(1) primary**: driver board under the carrier (it hangs 2.2 mm below the carrier: 1.0 board + 1.2 parts; with the 0.5 mm carrier-to-plate gap that's 1.7 mm into the 2 mm plate → rear-plate version A gets a full cut-out there; version B (option 3) only pockets for the rear passives), flex through the
    slot right of the glass (§3).
    Driver board (38 × 12, header on the **left** end, flex on the right) sits under the carrier
    at ≈ x −19.29 … 18.71, y 16 … 28 (can shift in X — flex has slack). **J3** = 4 THT holes
    matching its header, 2.0 mm from its left edge: x −17.29, y 25.81 / 23.27 / 20.73 / 18.19 =
    SDA, SCL, VCC, GND (top → bottom). Pins through both boards carry signals and hold it.
    **VCC = +3V3** (decided): the module's 662K/XC6206 (3.3 V out, ≤ 6 V in) then just passes
    ~3.25 V through — fine for the panel (rated 3.3 V), as in the prototype. If the display ever
    runs dim/unstable: switch to option 3 or wire-mod J3 pin 3 to +5V.
  - **(3) option, DNP**: no driver board. The flex is soldered on the carrier's **front**, under
    the glass's right end, exactly as on the driver board (flex loops out past the glass end and
    back under it — it does not wrap under the board): **J4** `ButtHeat:OLED_FPC_14P_P0.62mm`
    (gen_oled_fpc_fp.py) at (14.45, 20.25), pin 1 = C2P at the **bottom**; sits inside the glass
    rule area on purpose (DRC exclusion). Its passives (0603, desoldered from the module) are on
    the **rear side under the glass**, in the driver board's spot (mutually exclusive with (1)):
    C10 C2P–C2N, C11 C1P–C1N, C12 VBAT, C13/C14 VDD, C15 RES#, C16 VCOMH, C17/C18 VCC (values per
    the module), R8 560k IREF, R9 4.7k RES# pull-up, D4 SOD-523 RES# discharge diode,
    R10/R11 4.7k SCL/SDA pull-ups. No LDO (VBAT/VDD from +3V3). VBREF (pin 6) left open.

### WROOM support parts (variant B, added with the schematic)
- R12 10k EN pull-up + C19 1 µF EN delay: front, **below the WROOM's bottom pad row** at (−65, 3) /
  (−62, 3): R12 pads sit right under U1 pin 1 (3V3) / pin 2 (EN), C19 next to it, short EN path.
  Variant B: outside the module body (ends at y 4.4). Variant A: bare pads under the SuperMini —
  covered by the kapton that its exposed USB GND legs need anyway. Courtyard overlaps with U1/U2 →
  DRC exclusions.
- R13/R14 5.1k USB-C CC pull-downs + D5 VBUS→+5V Schottky (SOD-123): between ENC2 and ENC3 at
  (55, 14) / (55, 12) / (55, 9.3); ~11 mm CC runs to J2, D5's +5V side next to the buck output.

### 3.3 V LDO (variant B / WROOM only)
- Two alternative footprints on the same nets, fit one: **U5 SOT-23-5** (ME6211 / AP2112K, 500 mA —
  enough for Wi-Fi) or **U6 SOT-23** (the display module's 662K / XC6206, ~200 mA — fine without
  radio). Pinouts differ, so they can't be stacked. C8 (5 V in) / C9 (3.3 V out) 0603 shared.
  Front, above the left button: U5 (−28.6, 25.0), C8 (−25.7, 25.1), C9 (−24.2, 25.1), U6 (−21.6, 25.0)
  (moved 2026-11-16 from below J3 so the middle case part can bevel towards the knob).
  U5's EN is fed from +5V through a short **0.15 mm** stub (EN input current only — deliberate,
  2026-11-17; keep its DRC exclusion when re-running checks against the Power class).

### Buck 12 V → 5 V (discrete, Mini360 donor parts)
- Prototype uses a **Mini360** module (MP2307). It doesn't fit anywhere on the carrier without
  displacing a button, so v2 has the **same circuit discretely**, populated with parts
  desoldered from a Mini360 (sizes from its STEP: SOIC-8, IHLP-2525 inductor, 1206 / 0603).
- MP2307 SOIC-8: 1 BS, 2 IN, 3 SW, 4 GND, 5 FB, 6 COMP, 7 EN, 8 SS.
  Board refs (Mini360 ref → ours): U4 MP2307; L1 2 µH IHLP-2525 (SW→+5V); C3 12 µF 1206 input
  (C1); C4 12 µF 1206 output (C2); C5 4.7 nF 0603 COMP→RC (C3); R3 8.2k 0603 RC→GND; C6 0.1 µF
  0603 SS (C4); C7 10 nF 0603 BS–SW (C5); R4 100k 0603 EN→VIN; R2 8.2k 0603 FB→GND;
  **R5 39k 0603 FB→+5V_BUCK (≈5.27 V before D6)**, RV1 220k trimmer (TC33X-class footprint for the EVM-3E)
  in parallel position, normally **DNP**.
- Input protection ahead of it: D2 reverse-polarity Schottky (SMA, e.g. SS34), D3 TVS (SMA,
  e.g. SMAJ18A — MP2307 input max ~23 V). No fuse (1 A fuse at the car tap).
- Placement: top band right of the display — D2/D3 at x 20 … 34 above the right button,
  L1/U4/caps at x 34 … 51, 0603 column + RV1 at x 51 … 58 (y 17.5 … 28). ENC2 rotated back to
  **0°** (pins left/right) to free the band above it.
- Module 3D/STEP kept in `ButtHeatCarrier/3d/mini360.{step,f3z}`; footprint
  `ButtHeat:Mini360` (gen_mini360_fp.py) kept for reference, not used.

### CAN (above ENC1)
- U3 SOIC-8 at (−40, 24.9); default part **TJA1051T/3**, footprint also takes SN65HVD230
  (and 5 V-only TJA1050/TJA1051T via the RXD divider).
  - JP1 (3-pad solder jumper, below U3): U3 pin 3 VCC ← +5 V (TJA) or +3V3 (SN65).
  - JP2 (2-pad): pin 5 → +3V3 — closed for TJA1051T/3 (VIO), **open** for SN65 (Vref).
  - R6 0603 RXD series (0 Ω default) + R7 0603 RXD→GND (DNP): divider for 5 V RXD parts.
  - R1 0603 pin 8 (S / Rs) → GND. C1 / C2 0603 100 nF on VCC / VIO.
  - D1 CAN TVS SOT-23 (PESD2CAN-class) on the **back** next to J1 at (−71.0, 23.3) (moved 2026-11-16;
    its pins 1/2 swapped in the schematic, the part is symmetric).
  - No termination (car bus already terminated).
- 0603 column at x −33.7: C1, C2, R1, JP2, R6 (top → bottom).

### Buttons (DNP by default)
- Dual footprints at (−27, 19) SW1 THT `SW_PUSH_6mm` + SW2 SMD `SW_SPST_PTS645Sx43SMTR92`, and
  (27, 19) SW3 + SW4. Same nets on overlapping pads (pin 1 BTN_L/BTN_R, pin 2 GND).
- M2 fallback holes moved to **(±27, 13)** to clear the button pins.

### GPIO map (ESP32-C3, same for SuperMini and WROOM)
| Function | GPIO | Note |
|---|---|---|
| ENC1 A / B / SW | 2 / 3 / 4 | prototype |
| ENC2 A / B / SW | 5 / 6 / 7 | prototype |
| ENC3 A / B / SW | 10 / 20 / 21 | swapped with CAN 2026-11-15 (GPIO21 = U0TXD: boot log on ENC3_SW, harmless) |
| Buttons L / R | 10 / 20 | shared with ENC3 A / B (SW1/SW2 on ENC3_A, SW3/SW4 on ENC3_B) |
| I2C SDA / SCL | 8 / 9 | fixed (prototype) |
| CAN TX / RX | 0 / 1 | moved from 20/21 2026-11-15 (TWAI is routed via the GPIO matrix; firmware pins change) |
| USB D− / D+ | 18 / 19 | WROOM only (J2) |
GPIO2/8/9 are strapping pins. I2C pull-ups keep 8/9 high; the encoders have no external pull-ups
(internal ones are enabled in firmware), but the prototype boots fine with ENC1-A on GPIO2.

## 5. Case interfaces
- J1 (vertical ZH): plug goes in from the front; cable exits at the top-left corner through the
  split clamp (§“Cable and strain relief (decided 2026-11-16)”).
- Bottom wall: USB-C cutout (variant A: under the SuperMini; variant B: bottom-right, §“J2 USB-C”).
- Rear plate: VHB to the car panel; M2.5 heat-set inserts + M2.5×8 CSK posts (§“Rear plate v2”),
  ears ending at the R20 arc, x ±96.
- Front: 3-piece SLA; seams at ±20 (middle-part spigots to ±23 slide under the side skins,
  §“Case design status”); display window, scallop + facet for knob grip.

## 6. Open decisions / to verify
1. ~~Display mounting~~ → options (1) + (3), see §4 Display. (Glass through the board was ruled
   out: scallop apex to board top is 12.5 mm, glass needs 11.4. Option (2), driver board flat
   right of the glass, dropped.) Still to check: flex free length (7.5 mm) is enough to wrap
   through the 1.0 mm carrier to the driver board.
2. ~~Carrier thickness~~ → 1.0 mm decided. Front skin over the display 1.2 vs 0.8 — case design.
3. ~~Ø7.2 encoder holes~~ → only shaft-position markers (FreeCAD skeleton), no board holes.
4. Edge clearance board ↔ case wall (currently 0 at y = 2); suggest 0.3–0.5 mm.
5. ~~Flex pitch / pinout~~ → 0.62 mm, pinout from the vendor sheet (§4 Display).
7. ~~Glass height~~ → measured **30.0 × 11.5** (matches vendor GME12832-02; the FreeCAD model's
   11.0 is wrong — fix `TheScreen`). Slot lengthened accordingly. Vendor active area
   22.384 × 5.584 (model's 23.2 × 6.5 is likely the visible window).
6. ~~Glass active-area position~~ → from the FreeCAD model, see §4 Display.

8. ~~Variant-B USB-C (J2)~~ → **XKB U262-161N-4BVC11**, bottom-right corner, facing down (end walls
   blocked by the car's air ducts; a cable fits at the board's extreme ends) — §“J2 USB-C (variant B)
   — final part (2026-11-15)”. **Post 4 moved to (58, 4).** (First pick, GCT USB4085, proved unorderable.)
10. ~~Post 3 (−35, 4) collides with ENC1's body~~ → moved to **(−33, 5)**, §“Post 3 moved
    (2026-11-15)”.
11. **CAN transceiver** on the carrier (prototype used a TJA1050 5 V module — its RXD is 5 V, not
    safe for the ESP32-C3). Proposed SOIC-8 footprint that takes either TJA1051T/3 (automotive,
    5 V + VIO 3.3 V) or SN65HVD230 (3.3 V only); bus TVS (PESD2CAN-class, SOT-23), split
    termination DNP (car bus is already terminated). Prototype pins: TWAI TX GPIO20, RX GPIO21;
    I2C SDA GPIO8, SCL GPIO9.
12. **12 V input protection**: a 1 A fuse already sits at the car-side tap, so no on-board fuse
    (optional PTC footprint DNP at most). On board: reverse-polarity diode + TVS; Mini560 input
    is only rated ~20–24 V, so the TVS choice matters (load dump).
9. Check the SuperMini underside for exposed copper (USB shell legs etc.) over the WROOM's
   centre GND pad / vias.

## 7. Tooling notes
- KiCad 10 on NixOS; stock libraries resolve in the kcaa MCP (after its library-path fix; rebuild
  with `sync_footprint_index force=true` if lookups fail). Project fp-lib-table will be needed
  for custom footprints (SuperMini).
- kcaa MCP can't draw Edge.Cuts circles (use 2 semicircle arcs or MountingHole footprints), can't
  delete single graphic items (only clear all Edge.Cuts), can't set origins; DRC tool only opens
  the dialog → use `kicad-cli pcb drc`.
- MCP edits the file on disk: always `reload_kicad` afterwards and don't save from the GUI with a
  stale copy open.
- FreeCAD MCP: `execute_code` / headless runner broken on nix FreeCAD 1.1.3 — structured tools only.

### J2 USB-C (variant B) — final part (2026-11-15)
- **XKB U262-161N-4BVC11** (LCSC/JLC stock, 'Extended' = only matters for JLC assembly), 16-pin SMD +
  4 THT shell tabs + 2 pegs. Footprint `ButtHeat:USB_C_XKB_U262-16XN-4BVC11_noSBU`
  (hw/tools/gen_usbc_nosbu.py: stock XKB minus SBU pads A8/B8 — easier hand soldering).
- Placement: centre x 67.8, plug face ≈ 0.5 mm inside the case bottom line → footprint origin at case
  (67.8, 4.175) = KiCad (217.8, 95.825), rot 0. Front shell tabs start at y 2.3 (on board), rear
  signal pads end at y ≈ 8.42 (0.58 below ENC3's switch pins), shell tabs 0.38 from the right edge.
- Replaces GCT USB4085 (hard to order; its custom J2 pin-pitch DRC rule removed).

### WROOM-02 antenna keep-out relaxed to pour-only (2026-11-15)
- `ButtHeat:ESP32-C3-WROOM-02_Via0.3` keeps the antenna keep-out area, but it now forbids only copper
  pour; tracks, vias and pads are allowed. So J1 (pins under the antenna end) can be routed, while the
  GND plane stays away from the antenna (a plane under it would make the radio practically dead).
  Expect reduced range in build B1 (J1's pads/cable and a few tracks near the antenna); radio isn't used.
  If Wi-Fi/BLE is ever needed in earnest, use build B2 (WROOM-02U + u.FL pigtail).
- Keep tracks in that area few and short; J1 pin 2 (GND) needs a real track (the pour can't reach it).

## Post 3 moved (2026-11-15)
- Bottom-left screw post 3 moved from (-35, 4) to **(-33, 5)** (y raised 4 -> 5 so the Ø6.5 post stays ~0.7 mm inside the case-base outline), Ø6.5 post, Ø7.5 board notch. At the old spot it overlapped ENC1's body (x <= -36.8, y >= 4.4) by about 1.4 mm.
- The board notch now runs straight into the knob-arch edge: bottom edge to x -35.25, arc over y 8.75, then the arch line from (-29.29, 4.48).
- FreeCAD: `V2Vars.post3_left` = 33 mm; `PCB_Constraints` notes updated. The post-3 cylinder (`V2 Layout Draft`) and its notch circle in `BoardSkeleton/Cutouts` are now bound to `V2Vars.post3_left`/`post3_y`/`notch_d` (posts 1, 2 and 4 still have fixed placements).
- Case construction: the PETG rear plate (heat-set inserts) is taped in the car; the 3-part SLA front + PCB is held by 4 countersunk screws from the front. Posts can move but must stay.

- D1 (PESD2CAN) moved to the back next to J1 (2026-11-15); its pins 1/2 swapped in the schematic (symmetric part). The rear plate needs a ~1.2 mm pocket under it (PCB-to-plate gap is 0.5 mm).

## J1 = JST ZH vertical (2026-11-16)
- The JST PH side-entry J1 was unusable: its mouth faced the car's air-vent wall (x ≈ −78 at board level) with no room for the plug.
- J1 is now **JST ZH 4-pin vertical, through-hole (B4B-ZR, 1.5 mm pitch)**, rotated 180°, pin 1 (+12V) at (−67.55, 26.70); pins 2/3/4 = GND, CAN+, CAN− towards the left (x −69.05/−70.55/−72.05).
  - Body x −73.55 … −65.74, y 25.09 … 28.9: it overhangs the top edge by 0.9 mm and the left edge by about 0.5 mm, and clears the SuperMini body by 0.29 mm.
  - The plug goes in from the front.
- D1 (back) moved next to it: (−71.0, 23.3), rotated −90°.
- **Antenna WROOM-02 builds**: J1 is not fitted (the cable is soldered from the back), as before. DRC rule "J1 pins in WROOM antenna courtyard" plus the accepted J1/U1 and J1/U2 courtyard touches.
- **Case**:
  - The car's display frame is 1.8 mm above J1's body top at board level (frame bottom y 30.69 at z 3.5, rising about 0.27 mm per mm of height). The top wall must thin or notch there; that's also the natural cable exit.
  - A rib in the front case should press the ZH plug down, since ZH has no latch.
  - Check that the mated plug plus the wire bend fit under the front case.
- **3D**: KiCad's library has no B4B-ZR model; download it (JST / LCSC) like the USB one, then re-export the board STEP.
- **Measured (2026-11-16, ZH + ZHR-4 models on J1):** header + plug x −73.55…−66.05, y 25.4…28.9, plug top **z 9.4** (board top z 3.5). In the band y 24–30, the car has no geometry above **z ≈ 10** to the left of the board (checked to x −110); the frame starts at y ≈ 32.7 there.
- **Cable route (user, 2026-11-16):** the cable leaves at the **top-left corner, to the left, above the air vent**; this is the only possible exit. Wires leave the plug upwards, bend left at z ≈ 10.5–12, y ≈ 26–28. The front case's inside ceiling at the top-left corner must be at z ≥ 13.
- **Strain relief (plan):** anchor the cable to the **rear plate's left ear** (car-fixed): a saddle clamp with a screw into a heat-set insert, or a cable tie through slots, plus a zig-zag channel. Leave a 3–5 cm service loop to J1, so the front + PCB can lift off and J1 can be unplugged. Exit slot at the top-left corner (z ≥ 10) guides the cable but doesn't clamp it. A front-case rib (forked around the wires) holds the ZH plug down. Clamp size depends on the cable's diameter (to be decided).

## Rear-plate support pads (2026-11-16, user)
- Encoders (push switches) and buttons must not let the PCB flex when pressed. The rear plate gets **support pads 0.5 mm high** (PCB-to-plate gap) under each one:
  - encoders: annular pad about 6–8 mm across, centred on the shaft axis, inside the ring of pins and lugs, at (−43, 11), (43, 11), (65, 15);
  - buttons: under each button centre (−27, 19) and (27, 19), between the legs.
- Keep the pads clear of through-hole solder fillets and the rear-plate pockets/cut-out. Pressing on the back-side solder mask over copper is fine.

## Cable and strain relief (decided 2026-11-16)
- Cable: spare USB 2.0 cable, 4 cores (12V/GND on the power pair, CAN on D+/D−, probably twisted), probably unshielded, textile jacket, about 4–5 mm OD (to be measured).
- **Disconnect point stays at the car end** (car-grade locking connector). J1 (ZH) stays plugged in for good; a front-case rib holds the plug down.
- **The cable is clamped to the removable front assembly**, in the left front part at the exit, using a **replaceable printed split-clamp insert, 6 mm bore** (inner ribs for the textile jacket, closed by one M2 screw or squeezed by the parts). Another cable needs only a new insert, not a new case. Removing the device: undo the 4 screws and take out the front assembly with the cable attached; unplug at the car end. No zip ties (no room).
- **Exit as low as possible** (user). Measured free gap between the air vent and the display frame:
  - at x −74…−78 everything below y ≈ 30.5 is free down to z ≈ 1;
  - a 6 mm window needs its centre at z ≈ 4.5 (x −86), ≈ 6–6.5 (x −92), ≈ 6.5–7 (x −100), y ≈ 25–32.
  - So: exit in the left wall at x ≈ −76…−78, cable centre **z ≈ 6, y ≈ 28**. Inside, the wires leave J1's plug upwards (top z 9.4) and U-turn down to the clamp.
- The v1 `CaseBase` left ear's **leftmost leg** must go in the v2 design (it was broken off the prototype to make room for this cable).

## Rear plate v2 (2026-11-16, generated by hw/cad/v2_rear_plate.py → hw/cad/out/RearPlate_{A,B}.brep)
- Outline: v1 CaseBase base shape (knob arch) with the ears ending at the R20 arc (x ±96). The v1 thin strip to x ±106 is removed. The top edge is 0.2 mm short of the 15° display frame (y = 29.75 + z·tan15°).
- **Lip** along the top edge: a wedge from y 28.3 (PCB edge + 0.3) back to the frame, 6 mm tall, giving a glue face on the frame. Lowered to z 3.3 under J1's overhang. 45° chamfers at both ends and at the J1 steps (FDM strength). The front case's back wall stands on it.
- **Posts** (4): a Ø10 boss up to just under the PCB (clipped to the plate outline), then a **short Ø6.0 tube to z 6.0** (2.5 mm above the board top, so it still locates the board in its Ø7.5 notch), with Ø2.9 screw clearance. The **M2.5×4×3.5 heat-set insert** (the user has these; Ø3.6 pilot, 4.3 deep) **is pressed in flush from the plate's underside**, so it sits at z 0…4 and the screw's pull pushes it into the post, which then can't be torn off. The M2.5×8 tip at z 1.0 gives 3 mm of thread, so the screw's pull pushes it into the boss. The front parts continue with **Ø7.6 posts from z 6.1**. Screws: **M2.5×8 DIN 965** (cross-recess countersunk), head top sunk to **z 9.0** (tip at z 1.0, 3 mm into the insert) in a Ø6.0 counterbore (0.65 mm headroom around the Ø4.7 head). The rim gets a real 0.5 mm chamfer (OCC makeChamfer on the rim edges, the same width all round). The earlier cone-based chamfer cut through the skin around posts 3/4. (History: Ø7 tube to z 10.4 with M3×12 flush at 12.4 → M3×8 at z 9 → M2.5, all on 2026-11-16.)
- **Cable notches** in the left ear along the cable paths (see the left part).
- **Arch rib** 0.5 mm tall (touches the PCB) under the board's knob-arch edge, where the middle part's front edge presses: the plate's own front edge continued up to the PCB (one extrusion, flush with the front), 2.0 mm deep in y, |x| ≤ 18.9 (there the plate arch and the board arch coincide; further out the side parts' front walls stand on the plate). In plate B, the C10/C11 bay trims its back edge at x 8.85…12.35; plate A has no C10/C11 cut-out.
- **Plate B** passives pocket (0.8 deep) moved behind the rib: x −5.5…13, y 18.6…25, plus a bay for C10/C11 (x 8.85…12.35, y 15.45…18.7). The user moved C10/C11 to y 17.0 and the row behind to y 19.4 (rerouted 2026-11-16). The front wall at x 0 is now a full 2.1 mm (it used to run out to nothing). **Plate A**: the driver-board cut-out starts at y 15.8, leaving 0.3 mm of plate at x 0. That's the FDM minimum, set by the driver board's own edge at y 16.0.
- Support pads 0.5 mm: Ø6 under the encoder shafts, Ø4 under the buttons.
- **H1/H2 fallback M2 screw heads** (±27, 13): Ø5.5 through-holes.
- 1 mm pockets for the back-side parts: D1 (−71, 23.3), C1 (−40.7, 24.8; CAN VCC decoupling under U3 pin 3), C20 (−62.9, 3.8; WROOM 3V3 10 µF 0805). A: driver-board through cut-out (x −19.8…19.2, y 15.5…28.3). B: 0.8 mm passives pocket (x −5.5…13, y 13.5…25).
- Checked headless: no overlap with the car model; the posts clear the board notches by 0.25 mm; only through-hole leads overlap (trimmed or bent anyway).

## LDO cluster moved; front middle part (2026-11-16)
- The front case's **middle part** (hw/cad/v2_middle.py → hw/cad/out/Middle.brep) spans x −20…20, plus 3 mm spigots to ±23 that go under the side parts' skin (trapped):
  - top z 6.5 over the glass (0.2 mm air, 1.5 mm wall);
  - bevel plane from the glass's bottom edge (y 16.05, z 6.5) down to z 5.0 at the seams;
  - front edge flush with the board's knob-arch edge (the arch is our own clearance shape, not a car feature, so it can still change);
  - back face on the 15° frame, its skin resting on the rear plate's lip, which is lowered to z 5.0 for |x| ≤ 23;
  - display window = visible area + 0.2, flared 45°.
- To let the bevel work, **U5/U6/C8/C9 moved** from the left-front corner (x −21…−14, y 11–16) to above the left button (y ≈ 25, x −28.6…−21.6), under the left side part. The user re-routed; the USB pair lost 2 via pairs.
- Knob clearance (car knob R≈14.5 at the base, centred on the origin): about 1 mm at the top centre (limited by the glass), 2.5 mm at x ±10, 7 mm at the seams.

## Case design status (2026-11-16) — see ai-memory notes/v2-case-design-status.md for the full list
- Generators: hw/cad/v2_rear_plate.py, v2_middle.py, v2_left.py (→ hw/cad/out/*.brep), loaded by v2_load.py. Run them headless only (OCCT segfaults on coincident geometry; heavy booleans crashed the GUI).
- **Side parts' back edge follows the rear plate's lip profile** (2026-11-17): 45° end chamfers from z 0 at |x| 76, the J1 drop to z 3.3 (left) and the 5.0 zone at the seams. They sit on the lip with no gap (gaps used to show at x ≈ ±74 and x −65…−76).
- **Rounded shoulder front edge**: R1 like the middle part's, where the shoulder slope meets the vertical knob-arch front wall (seam to x ≈ ±30.5). It's filleted on the plain envelope before the posts are cut. Continuing it up the front rounding would also need the slope's own crease at y 8.6 rounded, so that part is left sharp.
- **Side parts: one generator, hw/cad/v2_side.py** (v2_left.py / v2_right.py set SIDE). Both are built in left coordinates and the right one is mirrored, so the shoulder, seam socket, ear, posts, chamfers and H bosses are identical by construction; the per-side features are in its `CFG` table.
- **Right part** (2026-11-17): x 20…96, ear along the R20 arc around (96, 8.95) (the car's right vent housing mirrors the left one).
  - Bushing holes for ENC2 (43, 11) and **ENC3 (65, 17)**. ENC3's shaft is at y 17, not 15: the rear plate's support pad is corrected.
  - **Posts 2 and 4** (58, 4). **Post 2 moved to (77, 25)** (V2Vars, 2026-11-17): at (78.5, 24) its counterbore was 0.1 mm from the ear's arc face. It's off the board and goes through no notch, so it moves freely; it now has 1.9 mm of wall, and the plate post starts at x 74.0, 1 mm clear of the board edge. The side script reads all post positions from V2Vars. ENC2/ENC3 body keep-outs.
  - SW3/SW4 on the shoulder (same two-tier pocket, 0.2 mm more room towards the seam for the THT switch); H2 boss.
  - **D2** (SMA, top 5.71) pocket under the shoulder near the seam, about 0.75 mm skin.
  - **J2 USB-C knock-out**, x 61.6…74.0, z 2…8.4, around the mouth (x 63.35…72.25, z 3.39…6.6), with a 0.4 mm skin along the front rounding. J2 overhangs the board edge to y 0.62, inside the knock-out.
  - Checked headless: no overlap with the car, plates, middle part or board, except the ENC2/ENC3 nut/washer in the skin (accepted, as for ENC1).
- **Left part** (hw/cad/v2_left.py, reworked 2026-11-16):
  - left end follows the car's vent housing: the ear above the plate's ear, bounded by the R20 arc around (−96, 8.95) (v1 leaned it to R19.1 at the top; v2 keeps it vertical), tip at x −96; no lip there, so its back wall goes down to the plate;
  - **shoulder** from the middle part's seam section (5.0 at the front edge, 6.5 from y 16.05) up to the full height of 12.4 at x ≈ −34.5 (about 22°; v1 was 18° to x −36.7). It's steeper because the v2 seam is 0.5 lower and the THT switch's top (features up to z 7.6) must stay under the skin. Over SW1/SW2 there's a two-tier pocket inside (body to 7.3, top to 7.8), leaving about 0.5 mm of skin at the seam-side corner. The button knock-out leaves a 0.4 mm web parallel to the slope;
  - **seam:** the middle part's spigots (its inner wall half, 3 mm past the seam) slide **under** the side part's sloped skin. The side part has a solid wedge between its skin and the socket (FIT 0.15), so the screwed-down side parts hold the middle part down. (A brief "tongue + rebate" variant was reverted: it left the middle part on top and loose.)
  - **posts 1/3:** Ø7.6 front posts with the M2.5 head at z 9.5 in a chamfered counterbore; an ENC1 body keep-out is cut from post 3's front post. The front wall next to post 3 is 1.1 mm thick;
  - **H1:** a Ø5 boss from the board top with a Ø1.7 pilot to z 7.6 for an **M2×4 self-tapping screw** from the back (3 mm into the boss);
  - USB knock-out keeps a 0.4 mm web parallel to the front rounding (no break-through);
  - **cable:** the Ø8.8 seat for the split insert (OD 8.6, bore 6) runs along −x at y 25.2, x −79…−74.6, sitting low (**centre z 6.35**, so the insert rests just above the plate). Two Ø6 exit channels start from its outer end (the end face takes the pull):
    - **A:** down into the V between the display frame and the vent housing, reaching (−96.5, 27.8, 2.5): at x −96 the cable touches the frame and z ≈ 0, then runs along −x. The rear plate's ear is notched along it;
    - **B:** straight into the housing at (−84.75, 25.43, z 1…4), in case the housing gets a cut-out (plate notched too).
    - **A 5 mm slot from every cable path down to z min** lets the crimped cable be laid in from below, so a reprinted part needs no re-pinning. The insert (OD 8.6, wider than the slot) can't drop out.
    - **Clamp insert** (hw/cad/v2_clamp.py → out/ClampTop.brep, ClampBottom.brep; shared geometry in hw/cad/v2_cable.py; cable diameter **V2Vars.cable_d**, default 4.5):
      - OD 8.6, 4.3 long; bore = cable_d − 0.5 with 3 grip rings 0.3 high, entry chamfers, a 0.4 lead-in on the outer end;
      - halves split top/bottom with a 0.2 gap, at the axis on the +y side and 1.2 higher on the −y (tab) side, so the bottom half's −y wall carries the tab solidly. On the cable they stand 8.9 tall in the 8.8 seat, so they're squeezed onto it;
      - two Ø1.0 pegs on the bottom half go into Ø1.2 holes in the top half (axial and rotation lock);
      - the bottom half has a tab at the J1 end (x −76…−72.4, y 17…20.7, z 5.8…7.3), joined to the bottom half's −y wall by a 1.5 mm web and fixed by an **M2×4 self-tapping screw from below** (the same as the H1/H2 fallback screws; 2.5 mm into the boss) into a Ø5 boss hanging from the ceiling at (−74.2, 19) (Ø1.6 pilot, 4 deep). The head sits at z 4.5…5.8, clear of the SuperMini (top 4.29), in a Ø4.4 clearance open to the bottom;
      - assembly, front part upside down and the board not yet in: lay the cable in from below, put the halves on it, slide them in from the J1 side, screw.
    - The 0.6 mm fin between the seat slot and the lip notch (x −79…−74.5, y 27.7…28.3) is removed.
  - J1 bump x −75…−62 to z 14.5; J1 clearance limited to y ≤ 29.1, so no sliver of back wall is left.
  - Checked headless: no overlap with the car, plates, middle part or board, except the accepted EC11 nut/washer in the skin.
- Encoder nut/washer may sit partly in the skin (accepted). Clamp insert retained by one M2 screw (split, keyed halves).
- Board follow-ups: WROOM 3V3 decoupling 10 µF + 100 nF at U1 pin 1 (to add to the schematic); CAN VCC cap C1 closer to U3 pin 3; a buck input HF cap at U4 pin 2; NECK rule scope; more GND bridges between the MCU-side and main pours.
