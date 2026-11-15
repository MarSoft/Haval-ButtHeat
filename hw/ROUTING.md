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

## Updates (2026-11-15)

### Buck output diode D6 (USB back-feed fix)
- New net **`/Power/+5V_BUCK`** = L1 pad 1, C4 pad 1, R5/RV1 (feedback top), D6 anode.
  **`+5V`** (rail) starts at D6's cathode. R5 = 39 k → buck at ~5.27 V, rail ~4.9–5.0 V after D6.
- Feedback still senses **before** D6 (stable loop): take R5/RV1's top from C4's pad.
- The old +5V tracks from L1/C4 to the rail must go: route L1 → C4 → D6 anode, and the rail from D6's
  cathode. D6 sits near (39.5, 16.3)-ish; place it where the old +5V track left C4.

### J2 CC2 (pin B5) escape
- The two pin rows (y 8.11 / 6.76) leave a 0.65 mm channel between them. From B5 (x 68.9) go right along
  y ≈ 7.43, past the last pin pair (x ≈ 70.6), then up through the 0.55 mm gap between pin A12 and the
  right shell leg, then into the board towards R14. Use **0.15 mm** track for that piece (board minimum),
  which keeps the default 0.2 mm clearance.

### Display neck / flex slot
- Board copper-to-edge clearance is now **0.3 mm** (slot edges included): above the slot y 25.3–27.7
  (2.4 mm), below it y ≈ 10.7–12.1 (~1.4 mm).
- Rule area **`NECK`** (draw it: *Place → Rule Area*, name `NECK`, no constraints ticked, both copper
  layers) over x 12.5…21.5, y 8…28 (KiCad X 162.5…171.5, Y 72…92). Inside it the custom rules give
  signal tracks 0.2 mm, power tracks 0.4 mm, clearance 0.15 mm.
- Channel plan at the slot:
  | Where | Layer | Nets |
  |---|---|---|
  | above slot | front | +12V, +5V (0.4), USB D+, D− (0.2) |
  | above slot | back | ENC2_A/B/SW, ENC3_A/B/SW (0.2) |
  | below slot | back | **GND pour channel** — keep free of tracks |
  | below slot | front | spare / second GND path |
- Left of the slot (under the glass) there's room on both layers; mind J4's front pads (x 13–16),
  J3's pins (one 0.25 track fits between neighbours) and the rear option-3 parts (x 2–12).
- Fallback if it doesn't fit: drop the slot (display option 1) and keep option 3 only.

### +12V / CAN (done by hand)
- +12V runs on the rear along the top edge, then switches to the front before the display. Keep rear
  tracks at the edge so the GND pour stays in one piece; cross the CANH/CANL pair at an angle, not
  under it lengthwise.

### Display option 3 (J4 + rear passives), placement 2026-11-15
All J4 nets leave the pads to the **left on the front** (only ~1 mm between the pads and the slot),
then drop to the rear part with a via next to its pad. The top band (y > 25.5) is taken by +12V, +5V and
ENC3_B (front) and the encoder bus (rear), so the passives sit at y ≤ 24, stacked in J4 pin order:

| case y | x ≈ −4 | x ≈ −0.5 | x ≈ 3 | x ≈ 6.5 | x ≈ 10 | x ≈ 9.6 / 11.6 | J4 pins |
|---|---|---|---|---|---|---|---|
| 23.6 | R11 (SDA pull-up) | C18 | C17 | C16 | R8 | | 14 VCCI, 13 VCOMH, 12 IREF |
| ~22 | *SDA/SCL on the front, straight left* | | | | | | 11, 10 |
| 20.0 | R10 (SCL pull-up) | | D4 | R9 | C15 | | 9 RES |
| 18.3 | | | C12 | C13 | C14 | | 8 3V3, 7 GND, 5 3V3 |
| 15.9 | | | | | | C11 / C10 (vertical) | 4–1 charge pump |

- Each signal pad (pad 1; on the pump caps both pads) faces J4. GND pads face left, one GND via each.
- Front fan-out: spread the 0.62 mm pitch to ≥ 0.9 mm before dropping vias. The bottom track of the
  upper group (IREF) turns down first; the same applies to the other groups, so nothing crosses.
- J4 GND (pin 7): via right at the pad end. 3V3 pins 8 and 5: one front track each to the caps.
- Rear below the slot, x 12–17, y 9–14: keep free, it's the GND pour's path to the channel under the slot.
