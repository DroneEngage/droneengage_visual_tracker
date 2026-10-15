# CA-0015 — Perception demo: fake camera → de_perceive → de_comm → de_tracker lock-on

One-command, one-PC end-to-end verification of the stack contract:

```
test image (person) --v4l2loopback--> de_perceive (ONNX + tracking sink)
   --6546 obs--> de_comm --6546--> de_tracker
   |                                ^
   +-- processed frames (v4l2) -----+   modules CHAIN via virtual devices:
                                       v4l2loopback admits one capture client
                                       per node, so the tracker reads
                                       de_perceive's output, not the raw cam
                                        |  bus_agent invokes visual_tracker.lock {class:"person"}
                                        |  -> lock resolves to trk:N, onTrack emits 1043 {t,box,trk}
                                        |  image blanks a while -> lock lost -> AI re-acquires SAME trk
                                        +-- unlock -> lock none
```

## Requirements

- Linux host with kernel headers for `v4l2loopback-dkms` (auto-installed by the script),
  `ffmpeg`, and root (sudo). Verified on `dronecode.local` (Debian 12, aarch64, Pi 4 4 GB).
- Binaries laid out as `DE_CODE` siblings, or pass explicit paths:
  - `de_comm` (fresh build with nav-adapter/journal support)
  - `droneengage_tracking/bin/de_tracker`
  - `droneengage_yolo_ai_generic/bin/de_perceive` built with the `onnx` backend
    (CPU onnxruntime is enough — the CUDA provider lib is optional)
  - `droneengage_yolo_ai_generic/models/yolo26n.onnx` + `test/data/test-image-640x640.jpg`
  - `de_databus/python/de_module.py` (bus client; resolved next to the three repos)

## Run

```bash
cd droneengage_tracking/scripts/perceive_lock_demo
DE_COMM=/path/to/de_comm ./demo.sh /path/to/yolo26n.onnx   # or let --build find it
# explicit binaries:
./demo.sh /path/to/yolo26n.onnx \
  --de-comm ~/de_sim/de_comm --de-tracker .../bin/de_tracker --de-perceive .../bin/de_perceive
```

`--no-vanish` skips the vanish/reappear phase (shorter run; check 4 reports FAIL).

## What it verifies (verify_log.py)

| # | Acceptance item                                                        |
|---|------------------------------------------------------------------------|
| 1 | 6546 `ObservationFrame` {cam_id, tm_capture, obs} with a person `trk` stable across >= 30 frames |
| 2 | `visual_tracker.lock {class:"person"}` invoke returns ok (6544); 6545 state shows `lock=trk:N` |
| 3 | 1043 `TrackingTargetLocation` while locked carries `t` deltas + `box` (top-left 0..1) + `trk` |
| 4 | vanish/reappear: locked trk disappears then reappears; de_tracker logs a second seed on the SAME `trk` |
| 5 | `unlock` -> state `lock=none`                                          |

Exit 0 only when all pass. Evidence lands in the run dir:

```
/tmp/perceive_lock_demo/run-*/
  bus.jsonl            every observed intermodule message, one JSON per line
  de_tracker.log       full tracker stdout (seed lines, lock-loss re-arm)
  de_perceive.log      backend output
  de_comm.log          module-registration/forwarding log
  agent_report.txt     lock/unlock invoke results
```

## Known gaps / not covered

- **de_ir_camera hotspot path**: not run — no MI48 hardware or replay data on the
  demo host; the thermal source uses the same 6546 contract but was not exercised.
- **RKNN variant** (CA-0014): separate acceptance on RK3588; this demo is the
  reference (de_tracker + de_perceive ONNX) path.
- **Pi hardware variant**: CA-0017 replays this scenario on droneengage.local
  (imx500 camera, wrapper v5) — this script is its one-PC proof.
- The `external` OBS capture port on the tap module is configured but only
  exercised if another module publishes `OBS`; the demo taps broadcast traffic
  instead, which covers all five checks.
