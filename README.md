# HUB75 Studio

[![License: MIT](https://img.shields.io/badge/License-MIT-green.svg)](LICENSE)
[![ESPHome](https://img.shields.io/badge/ESPHome-2025.10-blue)](https://esphome.io/)
[![Python](https://img.shields.io/badge/Python-3.11+-blue)](https://www.python.org/)

**Transform your HUB75 LED matrix into a smart display for Home Assistant.**

Flash ready‑to‑use firmware to your Apollo Automation M‑1 or Adafruit Matrix Portal S3, then enjoy a rich collection of Home Assistant–integrated applications:

Now Playing with album art, Team Tracker for live sports scores, Clock & Weather Dashboard, real‑time Audio Spectrum Visualizer, Visual Effects (fireworks, fireplace, aurora), interactive Pong, MSR‑2 Radar, Countdown Timers, QR codes, and more.

Built on **ESPHome** with **LVGL** for smooth graphics, everything integrates seamlessly with Home Assistant and can be fully customized by editing your device's YAML configuration. Multiple instances of the same page are supported (track multiple teams, rooms, or timers simultaneously).

> **Note**: This repository was formerly named `apollo-m1-playground`. GitHub automatically redirects the old URL, but please update your bookmarks and package references to use the new name.

---

## Requirements

### Hardware

**Supported Controllers:**

| Controller | Firmware File | CPU | PSRAM | Microphone | Accelerometer |
|------------|---------------|-----|-------|------------|---------------|
| **Apollo M‑1 rev4** | `apollo-automation-m1-rev4.factory.yaml` | ESP32‑S3 | None | ❌ | ❌ |
| **Apollo M‑1 rev6** | `apollo-automation-m1-rev6.factory.yaml` | ESP32‑S3 | 8MB (octal) | ✅ | ❌ |
| **Adafruit Matrix Portal S3** | `adafruit-matrix-portal-s3.factory.yaml` | ESP32‑S3 | 2MB (quad) | ❌ | ✅ |
| **Huidu HD‑WF2** | Coming soon | ESP32‑S3 | None | ❌ | ❌ |

**HUB75 LED Panels:**
- Optimized for 64×64 pixel panels
- Supports 1 or more panels in a chain (tested up to 4)
- Compatible with standard HUB75 interface panels
- Other sizes supported (32×32, 64×32, etc.) with configuration changes

### Software

**For Building from Source (Optional):**
- **Python 3.11+** required for ESPHome toolchain
- **ESPHome 2025.10.x** with ESP‑IDF framework (Arduino not supported)
- **Home Assistant** for full integration features

**For Using Prebuilt Firmware:**
- No local toolchain required — flash via web browser!
- **Home Assistant** recommended for entity integration

> **Firmware vs. Config Files**: GitHub Releases provide prebuilt binaries built from the **`.factory.yaml`** files for initial flashing. After you **Adopt** the device in ESPHome, it generates a device config based on the corresponding **non‑factory** YAML files (e.g., `apollo-automation-m1-rev6.yaml`, `adafruit-matrix-portal-s3.yaml`).

---

## Features

**Transform Your Matrix Into:**
- 🎵 **Music Dashboard** — Album art, track info, progress bar (multi‑room support)
- 🏈 **Sports Center** — Live scores for multiple teams (works with ha‑teamtracker)
- 🌤️ **Info Display** — Weather, time, person status, timers, QR codes
- 📊 **Visualizer** — Real‑time audio spectrum, presence radar
- 🎮 **Entertainment** — Pong, physics simulations, visual effects
- 🎬 **Media Player** — Stream GIFs, videos, YouTube content

**Built For Home Assistant:**
- Auto‑discovery and seamless integration
- Real‑time updates from your entities
- Control everything from HA dashboard

**Powerful Customization:**
- Run multiple instances (track multiple teams, rooms, players)
- WizMote remote control (brightness, page switching, power)
- Hardware‑specific features (microphone audio viz, accelerometer physics)
- Fully customizable via YAML configuration

**Developer Friendly:**
- Prebuilt firmware with web flasher (no toolchain required)
- Open source and extensible
- Built on ESP‑IDF with LVGL for smooth graphics
- Reproducible builds with lockfiles

📖 **[See all 15+ pages with detailed configurations →](https://github.com/stuartparmenter/hub75-studio/wiki/Pages-and-Applications)**

---

## Getting Started

**Quick Installation (Web Flasher)**
1. Visit **[HUB75 Studio Web Installer](https://stuartparmenter.github.io/hub75-studio/)**
2. Connect your device via USB and flash firmware
3. Configure Wi‑Fi and adopt in Home Assistant

📖 **[View detailed installation guide →](https://github.com/stuartparmenter/hub75-studio/wiki/Installation)**

> **Alternative**: Download prebuilt firmware from [Releases](https://github.com/stuartparmenter/hub75-studio/releases) and flash via [web.esphome.io](https://web.esphome.io/)

---

## Customization

Customize your device by editing its YAML configuration in ESPHome. Common customizations include:
- Display size and layout
- Weather entities and units
- Presence / alarm entities
- DDP / WebSocket streaming host/ports
- Device‑specific toggles for pages/effects
- WizMote remote control setup

📖 **[View detailed customization guide (including WizMote setup) →](https://github.com/stuartparmenter/hub75-studio/wiki/Customization)**

> Keep personal entity IDs and secrets in your local device config inside ESPHome; they're **not** tracked in this repo.

---

## Build from Source

**Using uv (recommended)**
```bash
# Install uv: https://docs.astral.sh/uv/getting-started/installation/

# Build & upload (USB or OTA) - dependencies auto-sync from lockfile
uv run esphome run apollo-automation-m1-rev4.factory.yaml
# or
uv run esphome run apollo-automation-m1-rev6.factory.yaml
# or
uv run esphome run adafruit-matrix-portal-s3.factory.yaml
```

**Using Docker (no local Python required)**
```bash
docker run --rm -it -v "$PWD":/config esphome/esphome run apollo-automation-m1-rev6.factory.yaml
```

---

## External Components
- **HUB75 Display Driver**: [`stuartparmenter/ESPHome-HUB75-MatrixDisplayWrapper`](https://github.com/stuartparmenter/ESPHome-HUB75-MatrixDisplayWrapper)
- **DDP Stream + WebSocket Control**: [`stuartparmenter/lvgl-ddp-stream`](https://github.com/stuartparmenter/lvgl-ddp-stream)
- **LVGL Canvas Effects**: [`stuartparmenter/lvgl-canvas-fx`](https://github.com/stuartparmenter/lvgl-canvas-fx)
- **Page Manager**: [`stuartparmenter/lvgl-page-manager`](https://github.com/stuartparmenter/lvgl-page-manager)

> For reproducible builds, prefer **tags** or **commit SHAs** rather than a moving branch.

---

## Repository Layout
```
hub75-studio/
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

## Troubleshooting

**Common Issues:**
- **Device not found in HA:** Power‑cycle, ensure Wi‑Fi credentials were set during flashing
- **Picked the wrong revision:** Re‑flash with the correct factory firmware for your hardware
- **USB permissions (Linux):** Add user to dialout/plugdev or use `udev` rules
- **Blank display:** Verify you selected the correct controller revision

📖 **[View complete troubleshooting guide →](https://github.com/stuartparmenter/hub75-studio/wiki/Troubleshooting)**

---

## Security & privacy
- Don’t commit secrets or personal entity IDs.
- Strip IPs/MACs/tokens from logs when filing issues.

---

## License
- Code & configs: **[MIT](LICENSE)**
- Fonts & icons: see **[THIRD_PARTY_LICENSES.md](THIRD_PARTY_LICENSES.md)**
