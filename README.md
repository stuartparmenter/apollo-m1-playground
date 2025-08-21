# Apollo M‑1 Playground

ESPHome (ESP‑IDF) playground for the Apollo M‑1 HUB75 controller — base config, dashboards, effects, and example “apps”.

This repo contains ESPHome configs for the **Apollo M‑1 (rev1)** HUB75 controller, plus bundled fonts and reusable LVGL pages. It targets **ESPHome 2025.8** on **ESP‑IDF** (Arduino is not supported).

> **Rev2 note:** a future *rev2* controller board will have different pins. Will add a new base file when I get the hardware.

---

## Repository layout

```
apollo-m1/
├─ apollo-m1.example.yaml      # template device config (includes base + styles + pages)
├─ vars.example.yaml           # safe defaults for substitutions
├─ secrets.example.yaml        # template for your secrets
├─ styles.yaml                 # shared fonts + LVGL styles (only Spleen fonts for text)
├─ packages/                   # drop‑in packages and LVGL pages
├─ src/                        # small C++ helpers used by pages
├─ fonts/                      # fonts used by LVGL pages
├─ LICENSE                     # MIT
└─ THIRD_PARTY_LICENSES.md     # font/icon licenses
```

---

## Quick start

1. **Copy templates**
   ```bash
   cp apollo-m1-playground/vars.example.yaml apollo-m1-playground/vars.yaml
   cp apollo-m1-playground/secrets.example.yaml apollo-m1-playground/secrets.yaml
   cp apollo-m1-playground/apollo-m1.example.yaml apollo-m1-playground/apollo-m1.yaml
   ```

2. **Edit `vars.yaml`**
   - Display size: `DISPLAY_W`, `DISPLAY_H`
   - Weather entities: `WX_ENTITY`, `HIGH_ENTITY`, `LOW_ENTITY`, `UNITS`
   - Presence/security: `PERSON_A`, `PERSON_B`, `ALARM_ENTITY`
   - DDP/WebSocket streaming: `WS_DPP_HOST`, `WS_DPP_PORT`, `DDP_PORT`
   - MSR‑2 radar prefix: `MSR2_PREFIX`

3. **Edit `secrets.yaml`**
   - Wifi, HA API, etc.

34 **Edit `apollo-m1.yaml`** if you want to remove or add pages: just comment out or delete the `!include packages/page-*.yaml` lines.

5. **Build/flash with ESPHome**
   ```bash
   esphome run apollo-m1/apollo-m1.yaml
   ```

---

## Build & Flash

You need **ESPHome ≥2025.8** with **ESP‑IDF** toolchain (Arduino is not supported).

### Option 1: pip + venv (recommended)

```bash
# Create a fresh virtualenv
python3 -m venv .venv
source .venv/bin/activate  # (Linux/macOS)
# .venv\Scripts\activate   # (Windows PowerShell)

# Install the pinned ESPHome version
pip install esphome==2025.8.0

# Build & upload (USB or OTA)
esphome run apollo-m1/apollo-m1.yaml
```

### Option 2: Docker

```bash
docker run --rm -v "${PWD}":/config -it esphome/esphome run apollo-m1/apollo-m1.yaml
```

---

## Pages overview

- **BIOS** (`page-bios.yaml`) – chip/memory/IDF info via tiny C++ helpers in `src/page_bios.h`.
- **Clock + Weather + Status** (`page-clock-dashboard.yaml`) – Spleen text, MDI icons; presence + alarm glyphs configurable via `PERSON_A/B` + `PERSON_A/B_ICON`.
- **Fireworks / Fireplace** – fun pixel effects; `page_fireworks.h` is self‑contained (LVGL v8 safe).
- **Pong** – configurable AI, speeds, and sizes via `substitutions:`.
- **DDP Stream** (`page-ddp-stream.yaml`) – LVGL canvas fed by a UDP DDP receiver and optional WebSocket control.  
  > ⚠️ **Note:** Requires the [lvgl-ddp-stream](https://github.com/stuartparmenter/lvgl-ddp-stream) server running on your network.
- **MSR‑2 Radar** – pulls HA entities by `MSR2_PREFIX` and renders sweep/targets/LEDs.

---

## Configuration & substitutions

Global substitutions live in `vars.yaml` (private) and are referenced by pages. Page‑local knobs are defined inside each page `substitutions:` section.

- Keep personal Home Assistant entity IDs (e.g., `person.*`, `alarm_control_panel.*`) **in `vars.yaml`**, not inside tracked files.
- The DDP/WebSocket host (`WS_DPP_HOST`) is typically a LAN IP or hostname—don’t hardcode it in page YAMLs.

---

## External components

- HUB75: `github://stuartparmenter/ESPHome-HUB75-MatrixDisplayWrapper@use-official-library`
- DDP stream + WS control: `github://stuartparmenter/lvgl-ddp-stream@v0.1.0`

> For reproducible builds, prefer **tags** or **commit SHAs** rather than a moving branch.

---

## ESP‑IDF notes

- This project uses **ESP‑IDF** only; components and `#include`s are IDF‑safe (no Arduino headers).
- PSRAM is currently disabled because of HUB75 pin conflicts on the target board (see comments in `apollo-m1-base.yaml`).

---

## Contributing

- Keep new pages under `packages/` with the `page-*.yaml` naming.
- C/C++ helpers go in `src/` as `page_*.h`/`.cpp`.
- Stick to Spleen fonts for text; if you need icons, prefer MDI glyphs (add glyphs explicitly).

---

## Security & privacy

- Never commit `secrets.yaml`, `vars.yaml`, or `apollo-m1.yaml`.
- Avoid real IPs, SSIDs, or personal entity IDs in tracked files and docs. Use placeholders or `substitutions:` examples.
- If you open issues, strip logs of IPs/MACs and tokens.

---

## License

- Code and configs: **MIT** (see `LICENSE`).
- Fonts and icons: see `THIRD_PARTY_LICENSES.md`.
