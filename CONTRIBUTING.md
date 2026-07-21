# Contributing to HUB75 Studio

Thanks for helping improve HUB75 Studio. The project is ESPHome YAML built on ESP-IDF with LVGL. Most contributions are new pages, added controller support, or fixes to existing configs.

## Before you start

- For anything non-trivial (a new page, a new controller, a behavior change), open an issue first so the approach can be discussed before you invest time.
- Small fixes such as typos, deprecation warnings, or one-line corrections can go straight to a pull request.

## Development setup

Requires Python 3.11+ and [uv](https://docs.astral.sh/uv/getting-started/installation/). ESPHome and its pinned dependencies install automatically from the lockfile.

```bash
# Build and flash a controller (USB or OTA)
uv run esphome run apollo-automation-m1-rev6.factory.yaml
```

No local Python toolchain? Use the Docker image:

```bash
docker run --rm -it -v "$PWD":/config esphome/esphome run apollo-automation-m1-rev6.factory.yaml
```

Builds target ESP-IDF only. Arduino is not supported.

## Repository layout

- `*.factory.yaml` files are the factory configs. GitHub Releases builds prebuilt firmware from these.
- `*.yaml` (non-factory) files are the config a device generates after you Adopt it in ESPHome.
- `packages/common/` holds the shared theme, DDP, utilities, and WizMote support.
- `packages/controllers/` holds hardware-specific configuration.
- `packages/pages/` holds the drop-in LVGL pages and effects.

Edit the package files under `packages/`, not a generated per-device config. Both the factory and non-factory entrypoints are built in CI, so a change to a shared package must build cleanly for every controller.

## Before you open a PR

1. Lint the config you touched. The linter checks includes, secrets, and substitution precedence:
   ```bash
   uv run python scripts/yaml_lint.py apollo-automation-m1-rev6.yaml
   ```
2. Build at least one controller that exercises your change, on hardware when possible.
3. Keep secrets and personal entity IDs out of the repo. Strip IPs, MACs, and tokens from any logs you paste.

## Pull requests

- One logical change per PR.
- Use a short, descriptive title, for example `Fix timer page hardcoding friendly name`.
- CI builds every controller against ESPHome stable, beta, and dev. All three must pass.
- Release notes are generated from labels, so a maintainer may add one such as `bug`, `enhancement`, or `documentation` at merge time.

## External components

Pages and effects depend on external components maintained separately:

- [lvgl-ddp-stream](https://github.com/pavlov-net/lvgl-ddp-stream)
- [lvgl-canvas-fx](https://github.com/pavlov-net/lvgl-canvas-fx)
- [lvgl-page-manager](https://github.com/pavlov-net/lvgl-page-manager)

Pin these by tag or commit SHA rather than a moving branch so builds stay reproducible.

## License

By contributing, you agree that your changes are licensed under the repository's [MIT license](LICENSE). Fonts and icons carry their own terms in [THIRD_PARTY_LICENSES.md](THIRD_PARTY_LICENSES.md).
