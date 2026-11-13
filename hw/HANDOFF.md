# ButtHeat v2 — handoff

Spec (single source of truth): [SPEC.md](SPEC.md). This file only tracks where we stopped.

## State (2026-11-14)
- KiCad `ButtHeatCarrier/`: re-originated (case (x,y) = KiCad (150+x, 100−y)); outline rebuilt
  (prototype-traced scallop, post notches, flex slot), DRC clean for the outline; J1 = S4B-PH-K;
  H1/H2 = M2 holes; U1 = WROOM-02U placeholder. Encoder Ø7.2 holes not drawn (SPEC §6.3).
  Stackup still says 1.6 mm → set 1.0 in Board Setup.
- FreeCAD `ButtHeatModel.FCStd`: `V2Vars`, `PCB_Constraints` (text cells partly stale — JST,
  MCU USB, display; SPEC.md wins), `BoardSkeleton`, `V2 Layout Draft`.

## Next steps
1. Encoder footprints (EC12S w/ switch) → settles the Ø7.2 hole question.
2. SuperMini footprint (project library); FPC pads for display option (3) once pitch is measured.
3. Trim `PCB_Constraints` to a pointer at SPEC.md; update `V2Vars.jst_*`.
4. Schematic (MCU A/B, CAN transceiver, buck/discrete supply, encoders, buttons, display).
5. Case: rear plate, 3-piece front, knob facet.
