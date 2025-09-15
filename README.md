# Apollo M‑1 Playground

ESPHome (ESP‑IDF) playground for the Apollo M‑1 HUB75 controller — base config, reusable LVGL pages/effects, and small example “apps.” Targets **ESPHome 2025.8.x** on **ESP‑IDF** (Arduino is not supported).

---

## What’s new (Sept 2025)
- **Refactor:** example templates removed. Two factory configs now cover hardware revisions.
- **Releases:** GitHub Actions builds **pre‑compiled firmware** for **rev4** and **rev6** on every tagged release.
- **On‑device config:** After adoption, all tunable variables live in the ESPHome Dashboard (Builder) for that device.

---

## Hardware revisions
Pick the file/firmware that matches your controller:
- **rev4:** `apollo-m1/apollo-m1-rev4.factory.yaml`
- **rev6:** `apollo-m1/apollo-m1-rev6.factory.yaml`

> GitHub Releases provide prebuilt binaries for these **factory** files. After you **Adopt** the device in ESPHome Builder, it will generate a device config from `apollo-m1-rev4.yaml` or `apollo-m1-rev6.yaml`.

---

## Flash prebuilt firmware (no local toolchain)
1. Go to **[Releases](/stuartparmenter/apollo-m1-playground/releases)** and download the correct **`apollo-m1-rev[4|6]`** firmware asset for your controller.
2. Open **https://web.esphome.io/** and connect your device via USB.
3. Click Install and select the downloaded firmware and flash.
4. When prompted, set Wi‑Fi credentials. The device will reboot and announce itself on the network.  If not prompted, click the 3 dots to set wifi.
5. Add it to **Home Assistant** (it will be auto‑discovered) or **Adopt** it in the ESPHome Dashboard.

---

## Build & flash locally (developers)
Requires **ESPHome 2025.8.x** with the **ESP‑IDF** toolchain.

**pip + venv (recommended)**
```bash
python3 -m venv .venv
source .venv/bin/activate        # Linux/macOS
# .venv\Scripts\activate        # Windows PowerShell
pip install "esphome>=2025.8,<2025.9"

# Build & upload (USB or OTA)
esphome run apollo-m1/apollo-m1-rev4.factory.yaml
# or
esphome run apollo-m1/apollo-m1-rev6.factory.yaml
```

**Docker**
```bash
docker run --rm -it -v "$PWD":/config esphome/esphome run apollo-m1/apollo-m1-rev6.factory.yaml
```

---

## Configure after adoption
All user‑tunable settings are exposed in the **ESPHome Dashboard** for your device (ESPHome Builder fields / substitutions). Typical options include:
- Display size and layout
- Weather entities and units
- Presence / alarm entities
- DDP / WebSocket streaming host/ports
- Device‑specific toggles for pages/effects

> Keep personal entity IDs and secrets in your local device config inside ESPHome; they’re **not** tracked in this repo.

---

## Repository layout
```
apollo-m1/
├─ apollo-m1-rev4.factory.yaml     # Factory config for rev4 (CI builds this)
├─ apollo-m1-rev6.factory.yaml     # Factory config for rev6 (CI builds this)
├─ packages/common-theme.yaml      # Shared LVGL theme (default font + colors)
├─ packages/                       # Drop‑in LVGL pages/effects
├─ src/                            # Small C/C++ helpers used by pages
├─ fonts/                          # Icon fonts used by pages
├─ .github/workflows/              # CI that builds release firmware (rev4 & rev6)
├─ LICENSE                         # MIT
└─ THIRD_PARTY_LICENSES.md         # Font/icon licenses
```

---

## Pages overview
- **BIOS** — chip/memory/IDF info via tiny C++ helpers.
- **Clock • Weather • Status** — Spleen text + Material Design Icons.
- **FX** — fireworks, fireplace, and other LVGL canvas effects.
- **Pong** — configurable AI/speeds/sizes via substitutions.
- **DDP Stream** — LVGL canvas fed by a UDP DDP receiver + optional WebSocket control.  
  > ⚠️ **Note:** Requires the [lvgl-ddp-stream](https://github.com/stuartparmenter/lvgl-ddp-stream) server running on your network.
- **MSR‑2 Radar** — renders sweep/targets/LEDs based on HA entities.
- **Microphone / Music Visualizer** — real‑time FFT bars and peaks driven by the on‑board mic.

---

## External components
- HUB75: `github://stuartparmenter/ESPHome-HUB75-MatrixDisplayWrapper`
- DDP stream + WS control: `github://stuartparmenter/lvgl-ddp-stream`
- LVGL Canvas FX: `github://stuartparmenter/lvgl-canvas-fx`
- Chipmunk2D ESPHome: `github://stuartparmenter/chipmunk2d-esphome`

> For reproducible builds, prefer **tags** or **commit SHAs** rather than a moving branch.

---

## ESP‑IDF notes
- This project is **ESP‑IDF only**; avoid Arduino headers.
- **Memory:** rev6 boards use PSRAM; rev4 boards do not. The correct factory file handles this automatically—no manual toggles needed.

---

## Troubleshooting
- **Device not found in HA:** Power‑cycle, ensure Wi‑Fi credentials were set during flashing.
- **Picked the wrong revision:** Re‑flash with the correct `rev4` or `rev6` factory firmware.
- **USB permissions (Linux):** Add user to dialout/plugdev or use `udev` rules.

---

## Security & privacy
- Don’t commit secrets or personal entity IDs.
- Strip IPs/MACs/tokens from logs when filing issues.

---

## License
- Code & configs: **MIT**
- Fonts & icons: see `THIRD_PARTY_LICENSES.md`
