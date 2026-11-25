# Repository Guidelines

ButtHeat: a custom seat-heater/climate controller retrofitted onto a Haval Dargo's BD-CAN-1 CAN bus. Two-part repo:

- **Firmware** (primary: repo root + `main/`) — ESP-IDF 5.5 app for ESP32-C3: listens to heater/AC status frames, drives three rotary encoders + SSD1306 128×32 display, sends control frames. SoftAP OTA update mode on button-held boot.
- **Hardware** (`hw/` + root-level artifacts) — KiCad board + FreeCAD case. Note the split: `hw/` holds only *docs and generator scripts*; the KiCad project is `ButtHeatCarrier/` and the FreeCAD model is `3d/` (symlink to outside of this repo, not committed), both at repo **root**.

`README.md` documents vehicle wiring, CAN protocol semantics, failure modes, and firmware updating. `hw/SPEC.md` is the declared single source of truth for hardware.

## Architecture & Data Flow

Firmware modules mirror the KiCad sheets: `buttheat.c` (app core/wiring + startup), `can.c`, `controls.c`, `display.c`, `ota.c` (SoftAP update mode), shared types in `buttheat.h`.

FreeRTOS tasks (all `xTaskCreatePinnedToCore`, stack 4096, `tskNO_AFFINITY`):

| Task | Prio | Role |
|---|---|---|
| `encoder_router_task` | 16 | routes encoder events; hand-rolled staged long-press via `esp_timer` synthetic events |
| `display_task` | 10 | SSD1306 render (passive banner / active overlay / CAN-error states) |
| `twai_transmit_task` | 9 | encodes + sends control frames |
| `twai_receive_task` | 8 | decodes status frames → per-handler queues |
| `generic_handler_task` ×7 | 3 | per-control logic (AC L/R, FAN, AIRFLOW, BUTT L/R, MASSAGE); arg = `handler_config_t*` |

Flow: TWAI RX ISR → `twai_rx_queue` → `twai_receive_task` → `xQueueOverwrite` into 1-deep per-handler `can_queue`s (latest-value-wins) → each `generic_handler_task` multiplexes its `control_queue` + `can_queue` via a FreeRTOS **queue-set** → emits `data_update_t` to `display_queue` (immediate) and `tx_task_queue` (throttled to 100 ms, plus a "settle" window of `CONFIG_TX_SETTLE_TIMEOUT_MS` that ignores CAN echoes after user input). All three encoders share one queue; events disambiguated by `.sender` handle.

- CAN RX: `0x2D1` heater, `0x385` AC/fan, `0x29D` airflow. TX: `0x36D`, `0x1EE`, `0x367`, `0x1ED`. Uses the **new `esp_twai.h` node API** (not legacy `driver/twai.h`), dual hardware mask filter, bus-off auto-recovery in the state-change ISR callback. Startup ordering matters: `twai_node_enable` runs only after all handler tasks have built their queue-sets.
- Display: vendored nopnop2002 `ssd1306` driver on new `i2c_master` API; custom 16×28 font from generated `main/font16x28.h`.
- Encoders: managed component `esp-idf-lib/encoder` v3.0.2; library long-press deliberately disabled (own staged long-press instead).
- Shared file-scope statics are cross-task and unsynchronized — single-core + comment-documented ownership; don't "fix" without reading the ownership comments.
- OTA update mode: holding BTN_LEFT (GPIO 4) at power-on branches into `ota_mode()` in `ota.c` **before** any task/TWAI init — SoftAP `ButtHeat-OTA` (WPA2 PSK from factory MAC), HTTP upload page at 192.168.4.1, streaming `esp_ota_*` write. Two 1.5 MB OTA slots (`partitions.csv`, no factory app); bootloader rollback + `ota_boot_watchdog()` (5 s) decide image validity — OTA mode itself never marks valid.

Known quirks (leave alone unless asked): `done_sem` never given → `app_main` teardown is dead code; BUTT control queues hold `uint8_t` but the handler receives `rotary_value_t` (harmless, value ignored); massage handler is placeholder wiring (CAN ID 0x000, hardcoded off); airflow handler has no encoder wired; README's "spinner" terminology is stale.

## Key Directories

| Path | Purpose |
|---|---|
| `main/` | All first-party firmware: `buttheat.c`, `can.c`, `controls.c`, `display.c`, `ota.c` + `ota_page.html` (embedded upload page), `buttheat.h`, `Kconfig.projbuild` (12 GPIO pins + `TX_SETTLE_TIMEOUT_MS`), generated `font16x28.h` |
| `libs/ssd1306/` | git submodule (nopnop2002); only its `components/ssd1306` is built, via `EXTRA_COMPONENT_DIRS` |
| `managed_components/` | idf.py-managed (`esp-idf-lib__encoder`), gitignored — never edit |
| `ButtHeatCarrier/` | KiCad project (root): root schematic + 5 sheets (`power`, `mcu`, `can`, `display`, `controls`), `.kicad_pcb` (routed, fab-ready), `ButtHeat.kicad_sym` + `ButtHeat.pretty/` custom libs, `.kicad_dru` waivers, `ibom.config.ini`, `bom/ibom.html` |
| `hw/` | Hardware docs + generators only: `SPEC.md`, `ROUTING.md`, `HANDOFF.md`, `MCP_ISSUES.md`, `cad/` (FreeCAD case), `tools/` (KiCad footprint generators) |
| `3d/` → `~/tmp/3dp/buttheat/` | FreeCAD model `ButtHeatModel.FCStd` (V2 case group, `V2Vars` VarSet) + STL/CTB prints |
| `tools/` (root) | `gen_font16x28.py` — firmware display-font generator |
| `case/` | v0.1 prototype STLs (superseded by `3d/` v1.0) |
| root `*.bdf`, `leggie/` | BDF font sources for `gen_font16x28.py` |

Stale entries: `hw/HANDOFF.md` (2026-11-14) lags `hw/SPEC.md`. Hardware naming: **v0.1** = prototype (older docs said "v1"), **v1.0** = first PCB variant (older docs said "v2"); `hw/cad/v2_*.py` and FreeCAD `V2*` names are internal and unchanged.

## Development Commands

Firmware (must be inside the nix shell — it's the only source of IDF + riscv toolchain):

```bash
direnv allow          # once after clone (.envrc = "use nix"); or: nix-shell
idf.py build
idf.py -p <port> flash monitor   # port isn't pinned in sdkconfig; monitor 115200
idf.py menuconfig                 # "ButtHeat config" menu = pins
```

Partitioning is the custom `partitions.csv` (two 1.5 MB OTA slots, no factory app; USB flash
targets `ota_0` + initialized otadata). First flash after switching to OTA partitions must be a
full USB flash. Releases: tag `vX.Y.Z` → `.github/workflows/release.yml` builds in the
espressif/idf Docker image, publishes the ESP-Web-Tools installer to GitHub Pages (`gh-pages`,
latest only) and a GitHub release. App version = `git describe` (no version files).

Hardware:

```bash
kicad-cli pcb drc --format json ButtHeatCarrier/ButtHeatCarrier.kicad_pcb  # DRC (NOT the MCP dialog)
python3 hw/tools/gen_usbc_16pf015.py …            # regenerate footprints → ButtHeat.pretty/
freecadcmd hw/cad/v2_clamp.py                     # headless; writes hw/cad/out/*.brep
python3 tools/gen_font16x28.py --bdf spleen-16x32.bdf > main/font16x28.h
python3 canscan.py canlog.csv 2D1 385             # diff CAN-analyzer CSV exports by ID
```

Case model: `v2_load.py` must run in the FreeCAD **GUI** (MCP `execute_code`); geometry generators run headless only (`freecadcmd` / MCP `execute_code_headless`) — GUI booleans have segfaulted (see `hw/MCP_ISSUES.md`).

## Code Conventions & Common Patterns

Firmware (C):
- snake_case; `_t`-suffixed typedefs; ALL_CAPS constants (`DU_*` event kinds, `CAN_ID_*`). Nearly everything `static`; the deliberate exceptions are `generic_handler_task`, `seat_memory_apply`, `disable_twozone_ac`.
- Error handling: `ESP_ERROR_CHECK` for one-shot init; runtime queue sends logged-if-full (`ESP_LOGE("queue full!")`); ISR/TX hot paths ignore return codes. Single log tag `TAG "ButtHeat"`.
- Config split: user-facing pins/timeouts in `main/Kconfig.projbuild`; display pins live in the *vendored ssd1306's* Kconfig (SDA=8/SCL=9); remaining magic numbers are `#define`s at the top of `buttheat.c`. `CAN_DEBUG` change-logging is an `#ifdef` that is never defined — enable ad hoc.
- Section banner comments; frame bit-layouts documented in comments near encode/decode (e.g. 0x36D layout at buttheat.c:404-437 — its bit positions deliberately differ from 0x2D1).

Hardware:
- **Generated artifacts are never hand-edited**: `ButtHeat.pretty/*.kicad_mod`, `hw/cad/out/*.brep`, `main/font16x28.h`, `bom/ibom.html`, `3d/OLED_0.91_glass_on_J4.wrl`. Regenerate via the owning script; generators and `hw/SPEC.md` are authoritative.
- `hw/cad/v2_*.py`: module-level scripts (no `__main__`), params read from the model's `V2Vars` VarSet, reuse via `exec()` not imports, geometry built in left coords and mirrored; `v2_load.py` loads `.brep`s into the FCStd `V2 Case` group.
- `hw/tools/gen_*.py`: stdlib-only Python; two styles — from-scratch s-expr emitters and regex patchers over stock KiCad footprints. Outputs use absolute/argv paths.
- After any MCP edit to KiCad files: `reload_kicad` / verify the change landed on disk (MCP_ISSUES.md I2/G3). Check `hw/MCP_ISSUES.md` before fighting a misbehaving MCP tool — 30+ known bugs with workarounds are documented there.

## Important Files

- `main/buttheat.c` — app core: handler registry, `generic_handler_task`, `app_main` startup + OTA-mode gate.
- `main/ota.c` / `main/ota_page.html` — SoftAP OTA update mode.
- `partitions.csv` — custom OTA partition table (source of truth for flash layout).
- `sdkconfig` / `sdkconfig.old` — target `esp32c3`; `sdkconfig.old` is idf.py's regeneration backup.
- `dependencies.lock` — pins IDF 5.5.2 + `esp-idf-lib/encoder` 3.0.2.
- `ButtHeatCarrier/ButtHeatCarrier.kicad_dru` — custom rule waivers (accepted courtyard touches, NECK area, J2 peg rules, GND spoke relaxation).
- `hw/SPEC.md` — hardware single source of truth (coords map: case (x,y) = KiCad (150+x, 100−y); Z-stack; board status "ready for fabrication" 2026-11-17).
- `hw/HANDOFF.md` — where hardware work stopped; update when resuming.
- `.mcp.json` — MCP servers `kicad-kcaa`, `kicad` (kicad-mcp-server, "mixelpixx" in docs), `freecad`.

## Runtime/Tooling Preferences

- ESP-IDF via nix only (`shell.nix`, mirrexagon/nixpkgs-esp-dev overlay pinned to `refs/pull/121/head`); direnv auto-activates. Target: esp32c3, riscv.
- Python scripts (canscan, generators) are stdlib-only — run with `python3`, no venv/package manager involved.
- KiCad 10 on NixOS; MCP-driven EDA is the established workflow for board edits.
- No linters/formatters configured; follow existing file style.

## Testing & QA

**No automated tests or hooks exist** (no test dirs, no pytest/unity; the only CI is the release-build workflow). Don't invent scaffolding; verification is manual:

- Firmware: `idf.py build` is the only automated check. Runtime observation on-target (`CAN_DEBUG` logs, display states, 500 ms RX-timeout → CAN-error screen) and offline CAN-log analysis with `canscan.py` (prints per-ID data-byte changes from analyzer CSVs).
- Hardware: `kicad-cli pcb drc --format json` + `.kicad_dru` waivers; ERC in KiCad with documented exclusions; InteractiveHtmlBom (`Sourced/Placed` checkboxes) for assembly tracking; `hw/SPEC.md` §6 tracks open to-verify items.
- When changing firmware behavior, at minimum build + flash + observe the affected display/CAN path; when changing board files, re-run DRC and check `SPEC.md`/`HANDOFF.md` state notes.
