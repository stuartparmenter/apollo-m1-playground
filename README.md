# Apollo M‑1 Playground

ESPHome (ESP‑IDF) playground for HUB75 matrix controllers — base config, reusable LVGL pages/effects, and small example "apps." Supports **Apollo Automation M‑1** controllers and **Adafruit Matrix Portal S3**. Targets **ESPHome 2025.9.x** on **ESP‑IDF** (Arduino is not supported).

---

## Supported Hardware
Pick the file/firmware that matches your controller:

### Apollo Automation M‑1 Controllers
- **rev4:** `apollo-automation-m1-rev4.factory.yaml`
  - ESP32-S3 with inaccessible PSRAM (effectively no PSRAM)
  - HUB75 matrix display support
  - No microphone or accelerometer

- **rev6:** `apollo-automation-m1-rev6.factory.yaml`
  - ESP32-S3 with 8MB PSRAM (octal)
  - HUB75 matrix display support
  - I²S digital microphone (GPIO 10/11/12) for audio visualization
  - Enhanced performance with PSRAM for complex pages/effects

### Adafruit Matrix Portal S3
- **S3:** `adafruit-matrix-portal-s3.factory.yaml`
  - ESP32-S3 with 2MB PSRAM (quad)
  - HUB75 matrix display support
  - Built-in accelerometer (LIS3DH) for motion-based effects
  - I²S digital microphone for audio visualization

> **Firmware vs. Config Files**: GitHub Releases provide prebuilt binaries built from the **`.factory.yaml`** files for initial flashing. After you **Adopt** the device in ESPHome Builder, it generates a device config based on the corresponding **non-factory** YAML files (e.g., `apollo-automation-m1-rev6.yaml`, `adafruit-matrix-portal-s3.yaml`).

---

## Flash prebuilt firmware (no local toolchain)
1. Go to **https://stuartparmenter.github.io/apollo-m1-playground/** and select your controller type.
2. Connect your device via USB and click "Install" to flash directly from your browser.
3. When prompted, set Wi‑Fi credentials. The device will reboot and announce itself on the network.
4. Add it to **Home Assistant** (it will be auto‑discovered) or **Adopt** it in the ESPHome Dashboard.

> **Alternative**: Download firmware from [Releases](/stuartparmenter/apollo-m1-playground/releases) and use **https://web.esphome.io/** to flash manually.

---

## Build & flash locally (developers)
Requires **ESPHome 2025.9.x** with the **ESP‑IDF** toolchain.

**pip + venv (recommended)**
```bash
python3 -m venv .venv
source .venv/bin/activate        # Linux/macOS
# .venv\Scripts\activate        # Windows PowerShell
pip install "esphome>=2025.9,<2025.10"

# Build & upload (USB or OTA)
esphome run apollo-automation-m1-rev4.factory.yaml
# or
esphome run apollo-automation-m1-rev6.factory.yaml
# or
esphome run adafruit-matrix-portal-s3.factory.yaml
```

**Docker**
```bash
docker run --rm -it -v "$PWD":/config esphome/esphome run apollo-automation-m1-rev6.factory.yaml
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
apollo-m1-playground/
├─ apollo-automation-m1-rev4.factory.yaml    # Factory config for Apollo M‑1 rev4
├─ apollo-automation-m1-rev6.factory.yaml    # Factory config for Apollo M‑1 rev6
├─ adafruit-matrix-portal-s3.factory.yaml    # Factory config for Matrix Portal S3
├─ packages/
│  ├─ common/
│  │  ├─ theme.yaml                # Shared LVGL theme (fonts + colors)
│  │  ├─ ddp.yaml                  # DDP streaming functionality
│  │  ├─ utils.yaml                # Common utilities
│  │  └─ wizmote.yaml              # WizMote remote control support
│  ├─ controllers/                 # Hardware‑specific configurations
│  └─ pages/                       # Drop‑in LVGL pages/effects
├─ fonts/                          # Icon fonts used by pages
├─ static/                         # Static assets
├─ .github/workflows/              # CI that builds release firmware
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
  > ⚠️ **Note:** Requires [media-proxy](https://github.com/stuartparmenter/media-proxy) running on your network (available as Home Assistant addon).
- **MSR‑2 Radar** — renders sweep/targets/LEDs based on HA entities.
- **Microphone / Music Visualizer** — real‑time FFT bars and peaks driven by the on‑board mic.

---

## WizMote Remote Control

The project includes support for WizMote remotes using ESPHome's built‑in ESP‑NOW protocol (requires ESPHome 2025.8+).

### Simple Setup Process
1. **Include WizMote Package**: Add `packages/common/wizmote.yaml` to your device configuration
2. **Turn on "WizMote Auto‑Discovery"** in Home Assistant
3. **Press any button on your WizMote** — MAC gets auto‑discovered and auto‑paired
4. **Done!** Discovery mode automatically turns off and WizMote is ready to use

### Button Functions
- **ON/OFF**: Display power control
- **Brightness Up/Down**: Adjust display brightness
- **Night**: Set minimum brightness
- **Buttons 1‑4**: Direct page navigation (configurable)

### Status
View pairing status via the "WizMote Status" entity in Home Assistant.

---

## External components
- **HUB75 Display Driver**: `github://stuartparmenter/ESPHome-HUB75-MatrixDisplayWrapper`
- **DDP Stream + WebSocket Control**: `github://stuartparmenter/lvgl-ddp-stream`
- **LVGL Canvas Effects**: `github://stuartparmenter/lvgl-canvas-fx`
- **Physics Engine**: `github://stuartparmenter/chipmunk2d-esphome`
- **Page Manager**: `github://stuartparmenter/lvgl-page-manager`

> For reproducible builds, prefer **tags** or **commit SHAs** rather than a moving branch.

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
