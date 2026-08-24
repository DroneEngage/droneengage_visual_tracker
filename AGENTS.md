# AGENTS.md — drone_engage_tracking

DroneEngage object tracker (`de_tracker`). Reads video from an input
device (physical camera or a `v4l2loopback` virtual device from an
upstream module like `de_ir_camera`), tracks objects, and writes the
result to its `output_video_device_name` virtual device for the next
module (e.g. `de_ai` / GCS). C++17, OpenCV, ONNX. See parent
`../AGENTS.md` for the **virtual video device pipeline** contract and
`de_common` vendoring.

## Build

    ./build.sh                 # DEBUG
    ./build_release.sh         # RELEASE
    ./build_ddebug.sh          # DEBUG + DDEBUG=ON

Out-of-source in `build/`. Binaries: `bin/de_tracker`,
`bin-rpi64/de_tracker` (RPi ARM64). Model: `yolo11n.onnx` (repo root).

### CMake options

- `DDEBUG` — detailed debug.
- Auto-increment build number from `.version` on RELEASE
  (MAJOR.MINOR.BUGFIX.BUILD). Bump via `MAJOR_VERSION`/`MINOR_VERSION`/
  `BUGFIX_VERSION` in `CMakeLists.txt`.

### Dependencies

OpenCV 4.5+, Threads, ONNX Runtime (for the tracker model). plog in
`3rdparty` if present.

## Config

- `de_tracker.config.module.json` — module config (WebClient UI).
- `template.json` — UI schema groups.
- `de_tracker.config.local` — instance identity.
- `de_tracker.config.module.json.bak_*` — timestamped backups (do not
  edit; the module writes these on config changes).

## Source layout & docs

`src/` — `main.cpp`, `tracker/`, `de_common/` (vendored), `defines.hpp`,
`version.hpp`. `wiki/` — `AI Interaction with Tracker.md`,
`messages-documentation.md`. `AI_ENHANCEMENT_PRIORITY1.md` — AI work
notes. `scripts/` — helper scripts. `local/` — machine-specific.
