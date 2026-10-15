#!/usr/bin/env bash
#
# CA-0015 perception chain demo:
#   looped clip -> v4l2loopback -> de_perceive (onnx, track stage)
#   -> de_comm -> de_tracker lock-on (visual_tracker.lock/unlock via 6543)
#
# One command. Prints PASS/FAIL per acceptance item at the end and leaves a
# bus JSONL log + per-process logs in $RUN_DIR.
#
# Host knobs (all optional):
#   RUN_DIR          working dir          (default /tmp/perceive_lock_demo)
#   DE_COMM_BIN      de_comm binary       (default $DE_CODE/droneengage_communication_pro/bin/de_comm)
#   PERCEIVE_BIN     de_perceive binary   (default $DE_CODE/droneengage_yolo_ai_generic/bin/de_perceive)
#   TRACKER_BIN      de_tracker binary    (default $DE_CODE/droneengage_tracking/bin/de_tracker)
#   MODEL            onnx model           (default $DE_CODE/droneengage_yolo_ai_generic/models/yolo11n.onnx)
#   TEST_IMAGE       source image         (default $DE_CODE/droneengage_yolo_ai_generic/test/data/test-image-640x640.jpg)
#   BUS_PORT         de_comm module port  (default 61000)
#   VDEV_IN/OUT/TRK  loopback devices     (default /dev/video100,101,102)
#   NO_CLEANUP=1     leave processes running for inspection
#
set -u
cd "$(dirname "$0")"

RUN_DIR="${RUN_DIR:-/tmp/perceive_lock_demo}"
DE_CODE="${DE_CODE:-$HOME/sim_build}"
DE_COMM_BIN="${DE_COMM_BIN:-$DE_CODE/droneengage_communication_pro/bin/de_comm}"
PERCEIVE_BIN="${PERCEIVE_BIN:-$DE_CODE/droneengage_yolo_ai_generic/bin/de_perceive}"
TRACKER_BIN="${TRACKER_BIN:-$DE_CODE/droneengage_tracking/bin/de_tracker}"
PERCEIVE_HOME="${PERCEIVE_HOME:-$DE_CODE/droneengage_yolo_ai_generic}"
TRACKER_HOME="${TRACKER_HOME:-$DE_CODE/droneengage_tracking}"
MODEL="${MODEL:-$PERCEIVE_HOME/models/yolo11n.onnx}"
TEST_IMAGE="${TEST_IMAGE:-$PERCEIVE_HOME/test/data/test-image-640x640.jpg}"
BUS_PORT="${BUS_PORT:-61000}"
VDEV_IN="${VDEV_IN:-/dev/video100}"
VDEV_OUT="${VDEV_OUT:-/dev/video101}"
VDEV_TRK="${VDEV_TRK:-/dev/video102}"
CLIP="$RUN_DIR/clip.mp4"

PIDS=()
OVERALL=0

say()  { echo "[demo] $*"; }
fail() { echo "[demo][FAIL] $*" >&2; OVERALL=1; }

cleanup() {
    say "stopping processes"
    for p in "${PIDS[@]:-}"; do
        [ -n "$p" ] && kill "$p" 2>/dev/null
    done
    sleep 0.5
    for p in "${PIDS[@]:-}"; do
        [ -n "$p" ] && kill -9 "$p" 2>/dev/null
    done
    # anything still holding this run's config is a leftover - sweep it
    pkill -9 -f "$RUN_DIR/de_comm.config.json" 2>/dev/null
    pkill -9 -f "$RUN_DIR/de_perceive.config.json" 2>/dev/null
    pkill -9 -f "$RUN_DIR/de_tracker.config.json" 2>/dev/null
    pkill -9 -f "bus_agent.py tap $RUN_DIR" 2>/dev/null
    say "logs in $RUN_DIR"
}
trap cleanup EXIT

wait_log_for() { # file, pattern, timeout_s  -> 0 when pattern appears
    local f="$1" pat="$2" t="$3"
    local end=$((SECONDS + t))
    while [ $SECONDS -lt $end ]; do
        [ -f "$f" ] && grep -q "$pat" "$f" && return 0
        sleep 0.3
    done
    return 1
}

wait_obs_person() { # timeout_s -> first trk of a confirmed person obs
    local end=$((SECONDS + $1)) trk=""
    while [ $SECONDS -lt $end ]; do
        trk=$(python3 - "$RUN_DIR/bus.jsonl" <<'EOF' 2>/dev/null
import json,sys
try:
    for line in open(sys.argv[1]):
        try: r = json.loads(line)
        except Exception: continue
        if r.get("mt") != 6546: continue
        for o in (r.get("ms") or {}).get("obs") or []:
            if o.get("cls") == "person" and "trk" in o:
                print(o["trk"]); sys.exit(0)
except FileNotFoundError:
    pass
EOF
)
        [ -n "$trk" ] && echo "$trk" && return 0
        sleep 0.5
    done
    return 1
}

# ---------------- preflight ----------------
for b in "$DE_COMM_BIN" "$PERCEIVE_BIN" "$TRACKER_BIN" "$MODEL" "$TEST_IMAGE"; do
    [ -e "$b" ] || { echo "missing: $b"; exit 2; }
done
for c in ffmpeg python3; do
    command -v "$c" >/dev/null || { echo "missing tool: $c"; exit 2; }
done
command -v sudo >/dev/null && SUDO="sudo -n" || SUDO=""

mkdir -p "$RUN_DIR"
say "RUN_DIR=$RUN_DIR"

# v4l2loopback devices (reload if present with the wrong device set)
if [ ! -e "$VDEV_IN" ] || [ ! -e "$VDEV_OUT" ] || [ ! -e "$VDEV_TRK" ]; then
    if lsmod | grep -q '^v4l2loopback'; then
        say "v4l2loopback already loaded with different device set - reloading"
        $SUDO rmmod v4l2loopback 2>/dev/null || \
            { echo "cannot reload v4l2loopback (in use) - unload it manually"; exit 2; }
        sleep 0.3
    fi
    say "loading v4l2loopback (video_nr=100,101,102)"
    $SUDO modprobe v4l2loopback video_nr=100,101,102 \
        card_label="DEMO-CAM-IN","DEMO-CAM-OUT","DEMO-TRK-OUT" \
        || { echo "v4l2loopback load failed"; exit 2; }
    sleep 0.5
fi
for d in "$VDEV_IN" "$VDEV_OUT" "$VDEV_TRK"; do
    [ -e "$d" ] || { echo "missing device $d"; exit 2; }
done

# ---------------- clip ----------------
if [ ! -s "$CLIP" ]; then
    say "generating clip (motion + timed person-vanish window)"
    ffmpeg -y -loglevel error -loop 1 -i "$TEST_IMAGE" -filter_complex \
 "[0:v]scale=800:800,zoompan=z='1.0+0.08*sin(2*PI*on/240)':x='iw/2-(iw/zoom/2)+30*sin(2*PI*on/90)':y='ih/2-(ih/zoom/2)':d=1:s=640x640:fps=15[p]; \
  [p]drawbox=x=0:y=60:w=450:h=580:c=black:t=fill:enable='lt(mod(t\,16)\,2)'[v]" \
        -map "[v]" -t 32 -c:v mpeg4 -q:v 3 -pix_fmt yuv420p "$CLIP" \
        || { echo "clip generation failed"; exit 2; }
fi

# ---------------- configs ----------------
cat > "$RUN_DIR/de_comm.config.json" <<EOF
{
    "module_id": "MT_1",
    "unit_type": "control_unit",
    "s2s_udp_listening_ip": "0.0.0.0",
    "s2s_udp_listening_port": "$BUS_PORT",
    "s2s_udp_packet_size": "8192",
    "use_unix_socket": false,
    "auth_ip": "127.0.0.1",
    "auth_port": 19408,
    "auth_verify_ssl": false,
    "use_local_comm_server": false,
    "userName": "demo@demo.droneengage.com",
    "accessCode": "demo1234",
    "unitID": "demo1",
    "groupID": "1",
    "unitDescription": "perceive-lock demo unit",
    "logger_enabled": false,
    "logger_debug": false,
    "ping_server_rate_in_ms": 1500,
    "max_allowed_ping_delay_in_ms": 5000,
    "max_offline_count": 1000,
    "led_pins_enabled": false,
    "buzzer_pins_enabled": false,
    "sv_force_paged": false,
    "sv_comm_lost_s": 10,
    "sv_battery_low_pct": 25
}
EOF

# class_names: COCO80 (person=0, dog=16) copied verbatim from the module config
read -r -d '' CLASS_NAMES <<'EOF' || true
["person","bicycle","car","motorcycle","airplane","bus","train","truck","boat","traffic light","fire hydrant","stop sign","parking meter","bench","bird","cat","dog","horse","sheep","cow","elephant","bear","zebra","giraffe","backpack","umbrella","handbag","tie","suitcase","frisbee","skis","snowboard","sports ball","kite","baseball bat","baseball glove","skateboard","surfboard","tennis racket","bottle","wine glass","cup","fork","knife","spoon","bowl","banana","apple","sandwich","orange","broccoli","carrot","hot dog","pizza","donut","cake","chair","couch","potted plant","bed","dining table","toilet","tv","laptop","mouse","remote","keyboard","cell phone","microwave","oven","toaster","sink","refrigerator","book","clock","vase","scissors","teddy bear","hair drier","toothbrush"]
EOF

cat > "$RUN_DIR/de_perceive.config.json" <<EOF
{
    "module_id": "AI-Generic-Detection",
    "s2s_udp_target_ip": "127.0.0.1",
    "s2s_udp_target_port": "$BUS_PORT",
    "s2s_udp_listening_ip": "127.0.0.1",
    "s2s_udp_listening_port": "61001",
    "s2s_udp_packet_size": "8192",
    "use_unix_socket": false,
    "auto_enable_detection": true,
    "backend": "onnx",
    "cam_id": "DEMO-CAM",
    "source_video_device": "$VDEV_IN",
    "output_video_device": "$VDEV_OUT",
    "model_path": "$MODEL",
    "class_names": $CLASS_NAMES,
    "track": {"confirm_hits": 2, "max_misses": 60, "iou_min": 0.3, "min_conf": 0.25},
    "advanced_tracking": {"frame_to_skip_between_track_process": 1,
                          "overlay_frames_between_updates": 5,
                          "de_camera_draw": false}
}
EOF

cat > "$RUN_DIR/de_tracker.config.json" <<EOF
{
    "module_id": "VIS_TRK_PRO",
    "s2s_udp_target_ip": "127.0.0.1",
    "s2s_udp_target_port": "$BUS_PORT",
    "s2s_udp_listening_ip": "127.0.0.1",
    "s2s_udp_listening_port": "61002",
    "s2s_udp_packet_size": "8192",
    "use_unix_socket": false,
    "camera": {
        "source_video_device": "$VDEV_OUT",
        "output_video_device": "$VDEV_TRK"
    },
    "tracking": {
        "tracker_algorithm_index": 7,
        "camera_orientation": 0,
        "camera_flipped": false,
        "tracking_camera_direction": 4,
        "crosshair": false,
        "de_camera_draw": false
    },
    "advanced_tracking": {
        "frames_to_skip_between_messages": 2,
        "frame_to_skip_between_track_process": 1,
        "lock_class_reacquire_frames": 40,
        "lock_reseed_cooldown_frames": 10,
        "ai_assisted_recovery_enabled": true,
        "ai_priority": true,
        "tracker_lost_timeout_ms": 10000
    }
}
EOF

cat > "$RUN_DIR/de_comm.local" <<EOF
{"module_key": "demo1-de_comm"}
EOF
cat > "$RUN_DIR/de_perceive.config.local" <<EOF
{"module_key": "demo1-de_perceive"}
EOF
cat > "$RUN_DIR/de_tracker.config.local" <<EOF
{"module_key": "demo1-de_tracker"}
EOF

# ---------------- processes ----------------
say "starting ffmpeg feeder -> $VDEV_IN"
ffmpeg -loglevel error -re -stream_loop -1 -i "$CLIP" \
    -f v4l2 -vcodec rawvideo -pix_fmt yuv420p "$VDEV_IN" \
    > "$RUN_DIR/ffmpeg.log" 2>&1 &
PIDS+=($!)
sleep 1

say "starting de_comm (module bus :$BUS_PORT)"
(cd "$RUN_DIR" && exec "$DE_COMM_BIN" -c "$RUN_DIR/de_comm.config.json" \
    -b "$RUN_DIR/de_comm.local" \
    > "$RUN_DIR/de_comm.log" 2>&1) &
PIDS+=($!)
sleep 2

say "starting de_perceive (onnx, $VDEV_IN -> $VDEV_OUT)"
(cd "$RUN_DIR" && exec "$PERCEIVE_BIN" -c "$RUN_DIR/de_perceive.config.json" \
    --bconfig "$RUN_DIR/de_perceive.config.local" \
    > "$RUN_DIR/de_perceive.log" 2>&1) &
PIDS+=($!)
sleep 2

# de_tracker reads de_perceive's output device: v4l2loopback admits only one
# capture client per device, so modules must chain, not share, the source.
say "waiting for de_perceive writer on $VDEV_OUT"
wait_log_for "$RUN_DIR/de_perceive.log" "Video writer initialized" 20 || \
    fail "de_perceive writer never initialized"
say "starting de_tracker ($VDEV_OUT -> $VDEV_TRK)"
(cd "$RUN_DIR" && exec "$TRACKER_BIN" -c "$RUN_DIR/de_tracker.config.json" \
    --bconfig "$RUN_DIR/de_tracker.config.local" \
    > "$RUN_DIR/de_tracker.log" 2>&1) &
PIDS+=($!)
sleep 2

say "starting bus tap -> bus.jsonl"
DE_COMM_PORT=$BUS_PORT python3 bus_agent.py tap "$RUN_DIR/bus.jsonl" \
    > "$RUN_DIR/tap.log" 2>&1 &
PIDS+=($!)

# ---------------- sequence ----------------
say "waiting for first confirmed person obs (6546)..."
TRK=$(wait_obs_person 90) || { fail "no person obs with trk within 90s"; }
[ -n "$TRK" ] && say "first person trk: $TRK"

say "accumulating obs stream (30+ frames, ~45s)..."
sleep 45

say "invoke visual_tracker.lock class=person"
DE_COMM_PORT=$BUS_PORT python3 bus_agent.py invoke visual_tracker lock '{"class":"person"}' 8 \
    | tee "$RUN_DIR/invoke_lock.out" || fail "lock invoke failed"

say "watching locked tracking for ~40s (covers the clip's hidden-person window)..."
sleep 40

say "invoke visual_tracker.unlock"
DE_COMM_PORT=$BUS_PORT python3 bus_agent.py invoke visual_tracker unlock '{}' 8 \
    | tee "$RUN_DIR/invoke_unlock.out" || fail "unlock invoke failed"
sleep 3

# ---------------- verify ----------------
say "verifying captured bus log"
python3 verify_log.py "$RUN_DIR" && say "DEMO: PASS" || { fail "verification failed"; }

[ "${NO_CLEANUP:-0}" = "1" ] && { say "NO_CLEANUP set - leaving processes"; trap - EXIT; }
exit $OVERALL
