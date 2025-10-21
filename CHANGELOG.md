# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v0.2.0.html).

## [Unreleased]

## [v0.6.1] - 2025-10-21

### Changed
- Repository renamed from `apollo-m1-playground` to `hub75-studio` to better reflect the project's support for multiple HUB75 controllers beyond just Apollo Automation hardware

## [v0.6.0] - 2025-10-20

### Added
- ESPHome minimum version package -- requires ESPHome >= 2025.10.x
- DDP receiver page (enabled by default on PSRAM controllers)
- Binary sensor to detect incoming DDP data on receiver page
- Web server v3 to PSRAM configs
- Timer package for countdown timers

### Changed
- Component rename: lvgl-ddp-stream → ddp-esphome (now at v0.7.2)
- Bumped lvgl-canvas-fx to v0.3.0 (now includes chipmunk2d directly)
- Moved display wrapper to use forked display driver w/ brightness and gamma fixes
- Improved teamtracker NOT_FOUND state handling
- Simplified page with explicit w/h for mDNS discovery
- BIOS screen version label shortened (VER: → V:) for better fit
- Updated workflows and dependabot config

### Fixed
- Brightness value type (now integer)
- Added bg_opa for transparent objects
- Internal placeholder image URL for teamtracker

### Removed
- Old wledsync page (replaced by ddp-receiver)

## [v0.5.1] - 2025-09-30

### Added
- Media stream to factory firmware
- Timer package for countdown timers
- Improved now-playing logging behavior (only logs when actively playing)

### Changed
- Bumped lvgl-ddp-stream to v0.5.1

### Fixed
- Default weather icon text now uses a glyph the font has

## [v0.5.0] - 2025-09-29

### Breaking Changes
- **BREAKING:** Package variables now use lowercase names
- **BREAKING:** All pages now require `page_friendly_name` variable

### Added
- Teamtracker page for sports tracking with ha-teamtracker integration
- QR code page for displaying QR codes
- UID support for now-playing page to allow multiple instances

### Changed
- Bumped lvgl-ddp-stream to v0.5.0
- Now-playing page shows only album art on single panels
- WizMote night mode brightness changed to 20
- Teamtracker styles updated for UID support

### Fixed
- Removed extra/wrong/unnecessary update logic

## [v0.4.1] - 2025-09-21

### Added
- WizMote remote control support with auto-discovery and pairing
- New lvgl-page-manager external component for better page management
- Strapping pin warning suppression

### Changed
- Bumped lvgl-ddp-stream to v0.4.1 with crash and reconnect fixes
- Combined internal and PSRAM display on BIOS screen for single panel width
- Moved display power and brightness controls to utils
- Updated installer page to add new controller option
- Updated README

### Fixed
- Crash and reconnect issues in DDP streaming

## [v0.4.0] - 2025-09-17

### Added
- Adafruit Matrix Portal S3 controller support
- Additional diagnostic buttons and sensors
- Video source persistence and restoration

### Changed
- Updated to ESPHome 2025.9.0
- Bumped lvgl-ddp-stream to v0.4.0
- Reorganized and renamed files for clarity
- Always normalize text for better display
- Enhanced unicode simplification

## [v0.3.6] - 2025-09-15

### Breaking Changes
- **BREAKING:** Fixed all references from rev7 to rev6 (there is no rev7 yet)

### Added
- ESP32 advanced settings to mirror rev6 settings (minus PSRAM for rev4)
- Device ID to WebSocket DDP control for easier debugging

## [v0.3.5] - 2025-09-14

### Added
- DDP enabled by default in adopted configs
- Bluetooth disabled after WiFi connection to save RAM
- Refactored DDP & WebSocket control into separate package
- Text helpers converted to scripts to simplify build process

### Changed
- Bumped lvgl-ddp-stream to v0.3.1
- Cleaned up variables and substitutions

### Fixed
- Typo in comment

## [v0.3.1] - 2025-09-10

### Added
- Audio spectrum renderer migrated to separate lvgl-canvas-fx component
- Included audio spectrum by default with rev6 config

### Changed
- Updated project description

## [v0.3.0] - 2025-09-09

### Added
- GitHub workflows from ESPHome project template
- Rev4 project templates and cleaned up defaults
- Basic clock and date page that doesn't require configuration
- Factory version with improv support using dashboard_import URL
- Microphone EQ visualization page
- Microphone mute switch
- Microphone support for rev6 base config
- Aurora effects
- New dynamically switchable FX page using lvgl-canvas-fx
- Now playing page for media players
- YAML linter script to check for unique substitutions

### Changed
- Default font/background/text colors centralized
- Common helpers moved to new file
- Renamed styles.yaml to packages/common-theme.yaml for consistency
- Font URLs changed to GitHub instead of local
- Split heap reporting to show both internal & PSRAM
- Split base M-1 controller config into rev4 and rev6 variants
- Display pin mappings updated for rev6 boards
- Removed old effects pages (migrated to page-fx.yaml)
- Media player improvements with placeholder text and proper progress computation
- Video moved back to triple buffering for smoother playback
- ESPHome builder includes all optional pages for rev6
- Fixed published page workflow artifact for firmware selection
- Temporary fix for dashboard_import
- Material icons webfont now uses GitHub URL

### Fixed
- Entity picture URLs for media players
- Unicode character replacement for unsupported font characters
- Flash size set correctly for ESP32-S3-WROOM-1 MCN16R8 (16MB)
- Unique substitutions ensured across packages

### Removed
- Old fireplace and fireworks pages (deprecated in favor of lvgl-canvas-fx)
- Template files
- vars.yaml file

## [v0.2.0] - 2025-08-29

### Added
- New media player now playing page
- YAML linter script to check for unique substitutions

### Changed
- Moved to lvgl-ddp-stream 0.2.2 with no back buffers by default to save memory
- Ensured substitutions are unique across packages

## [v0.1.5] - 2025-08-28

### Added
- More comments to example config
- Helper for getting screen size
- Example of using a grid of displays instead of just horizontal
- Comment about optionally lowering color depth to save memory
- New DDP example for direct usage with WLEDVideoSync
- New fireworks package using chipmunk2d physics engine

### Changed
- Enabled fireworks-physics and disabled old fireworks
- Updated to use lvgl-ddp-stream@v0.2.1 with full screen canvas
- Used passed in canvas buffer instead of creating new one
- Used new published chipmunk2d-esphome package
- Reworked/cleaned up fireworks with better canvas size handling
- Updated HUB75 pin comments for Apollo Automation rev4 controller
- Rocket launch length now varies with canvas height
- DISPLAY_W/DISPLAY_H changed to full display dimensions
- PANEL_W/PANEL_H added for individual panel size
- Increased I2S speed for faster display throughput
- Moved to lvgl-ddp-stream v0.2.0 for better playback performance

### Fixed
- Datetime label updates once time is available instead of waiting for next minute

### Removed
- Unused functions from fireworks
- Unneeded duplicated substitution

---

## Initial Release

### [Initial Commit] - 2025-08-21
- Initial project setup with Apollo M-1 HUB75 matrix display support
- LVGL integration for graphics rendering
- Basic page system with multiple interactive displays
- ESPHome integration with ESP32-S3 support
