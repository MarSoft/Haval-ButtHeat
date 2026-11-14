# ButtHeatCarrier — routing guide

Coordinates are case coordinates (mm). In pcbnew, *Preferences → PCB Editor → Origins & Axes*:
display origin = grid origin, Y axis increases up — then the status bar shows these numbers directly.
Rules are already set (net classes Default 0.25 / Power 0.5 / Buck_SW 0.6 mm, vias 0.6/0.3 mm).

## 0. Setup & handy keys

- Track width dropdown (top toolbar): **"Use net class width"** — then every net gets its class width.
- *Route → Interactive Router Settings*: mode **Shove** (pushes other tracks aside) or **Walk around**.
  Start with Walk around: less surprising.
- Keys:
  - **X** route from a pad / track · **V** while routing = place a via and switch layer
  - **/** toggle the bend posture · **Backspace** undo last segment · **Esc** finish/cancel
  - **D** drag a track segment (keeps 45° shape) · **G** drag free
  - **U** select the connected track · **E** properties · **Del** delete
  - **`** (backtick) highlight net under cursor · **B** refill all zones · **Ctrl+B** unfill
  - **PgUp / PgDn** front / back copper as active layer
- After each block: save, run DRC (untick *Warnings* to hide silk noise), refill zones (B). Tell me when
  a block is done — I'll run DRC + render and point out problems.

## 1. General rules for this board

- **Back (B.Cu) = ground.** It's a solid GND pour. Keep back-side traces *short* (jumping under
  something), never long runs that cut the pour in two. Every GND pad on the front gets a **via to
  the back right next to it** (one via per GND pad, two for the buck GND).
- **Front (F.Cu)** carries almost all signals and power.
- 45° bends, no 90° corners (the router does this for you).
- Power nets (+12V, VIN, +5V, +3V3) at their class width; if a pad is narrower than the track,
  neck down only for the last ~0.5 mm into the pad.
- THT pads in the pour use thermal reliefs (already set) — easier to solder.
- Leave ~0.5 mm between tracks and the board edge (DRC enforces 0.5).
- Both MCU variants share nets: route each GPIO net to **both** U1 (WROOM) and U2 (SuperMini) pads.

## 2. Routing order

1. Buck converter (most critical) — §3
2. Power distribution: +12V from J1, +5V and +3V3 rails — §4
3. CAN block — §5
4. MCU fan-out (U1/U2 shared GPIOs, EN, USB) — §6
5. Display (J3 / J4 + rear passives) — §7
6. Encoders and buttons — §8
7. Finish: second GND pour on the front (optional), stitching vias, silkscreen pass — §9

## 3. Buck converter (U4 MP2307, top band x 20…58, y 17…28)

### What matters
A buck converter has two current loops that switch at ~340 kHz. Their **area** is what radiates and
what causes noise — not the track width:
- **Input loop:** C3 (+) → U4 IN (pin 2) → U4 GND (pin 4) → C3 (−). Make it as small as possible.
- **Output loop:** U4 SW (pin 3) → L1 → C4 → GND → back to U4 GND (pin 4).
- **SW node** (U4 pin 3 ↔ L1 pad 2 ↔ C7): short and compact (0.6 mm, net class Buck_SW). Don't run
  anything sensitive next to it. No vias on SW.
- **FB node** (U4 pin 5 ↔ R5/R2/RV1): high impedance, sensitive — keep it short and **away from
  SW and L1**. R5/R2 are already at the right of U4, good.

### Pin map (case coords, approximate)
| Part | Pin | Net | Position |
|---|---|---|---|
| U4 | 1 BS / 2 IN / 3 SW / 4 GND | | left column x ≈ 45.1, y ≈ 25.2 / 23.9 / 22.7 / 21.4 |
| U4 | 5 FB / 6 COMP / 7 EN / 8 SS | | right column x ≈ 50.1, y ≈ 21.4 / 22.7 / 23.9 / 25.2 |
| L1 | 1 / 2 | +5V / BUCK_SW | (35.8, 24.4) / (41.6, 24.4) |
| C3 | 1 / 2 | VIN / GND | (43.9, 19.4) / (46.9, 19.4) |
| C4 | 1 / 2 | +5V / GND | (34.5, 19.55) / (37.5, 19.55) |
| C7 | 1 / 2 | BS / SW | above U4 at y ≈ 26.75 |
| D2 | 1 K / 2 A | VIN / +12V | (21.1, 26.2) / (25.1, 26.2) |
| D3 | 1 K / 2 A | VIN / GND | (28.6, 26.2) / (32.6, 26.2) |

### Placement tweaks to consider first (tell me, I can apply them)
- **D2 is reversed for the flow**: +12V arrives from the left (J1) but D2's anode is on its right.
  Rotate D2 by 180° so +12V enters on the left and VIN leaves to the right towards D3/C3.
- **C3 is a bit far from IN/GND**: its VIN pad must reach pin 2 around pin 3/4. Acceptable, but if
  routing gets awkward we can nudge parts (e.g. C3 closer under pins 2–4).

### Steps
1. **GND first**: U4 pin 4 → short wide track to C3 pad 2, and **2–3 vias** to the back pour right
   there. C4 pad 2, C5's R3, C6, R2 → each gets its own GND via.
2. **Input loop**: C3 pad 1 (VIN) → U4 pin 2, shortest path. Then VIN from D2/D3 to C3 (0.5 mm).
3. **SW**: U4 pin 3 → L1 pad 2, short; C7 between pin 1 (BS) and the SW track.
4. **Output**: L1 pad 1 → C4 pad 1 (+5V), then +5V onward (§4).
5. **Small signals** (Default 0.25 mm): EN pin 7 → R4 → VIN; SS pin 8 → C6; COMP pin 6 → C5 → R3;
   FB pin 5 → R5 / R2 / RV1 pin 1; R5/RV1 other ends to +5V (take +5V from C4, not from L1's pad).
6. Refill zones (B), DRC.

## 4. Power distribution (outline, details when we get there)
- **+12V**: J1 pin 1 (−71.5, 25.5) → along the top edge → D2 anode. Long run, 0.5 mm, front.
- **+5V**: from C4 to U2 5V pin, U5/U6 inputs (C8), D5 cathode, CAN JP1 pin 1.
- **+3V3**: from U5/U6 outputs (C9) to U1 pin 1, U2 3V3, CAN JP1/JP2, display (J3 pin 3, R9–R11, C12–C14).
- **VBUS** (WROOM only): J2 VBUS pins → D5 anode, short.

## 5–8. CAN, MCU, display, encoders
Detailed notes will follow per block once the buck and power are done (the placement there may still
shift a little). Headlines:
- **CAN**: CANH/CANL from J1 pins 3/4 to U3 pins 7/6 as a pair (side by side, similar length); D1 at
  the transceiver. TX/RX to the MCUs at 0.25 mm.
- **MCU**: U1 and U2 overlap — fan each GPIO out to a short "bus" that touches both footprints' pads.
  USB D+/D− (J2 → U1 pins 14/13) side by side, similar length, no vias if possible.
- **Display**: J4 pads are 0.35 mm wide at 0.62 mm pitch — use 0.2 mm tracks there (Default class
  minimum allows 0.15). Rear passives connect with vias to the J4 nets.
- **Encoders/buttons**: plain 0.25 mm signals to the MCU; switch common pins to GND vias.
