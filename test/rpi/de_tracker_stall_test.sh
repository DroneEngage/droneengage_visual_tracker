#!/usr/bin/env bash
# CA-0041 module-level test: run the real de_tracker binary against a
# v4l2loopback device fed by mjpeg_feeder, and drive the lock -> track ->
# unlock -> relock cycle plus a producer-stall reseed using raw de_databus
# UDP datagrams (no de_comm needed - module ports accept JSON datagrams).
#
# Old code aborts (std::terminate -> SIGABRT) when a reseed lands while the
# capture thread is blocked in its pre-start window; the fixed code joins
# the blocked thread and resumes tracking once frames return.
#
# Usage:
#   modprobe v4l2loopback video_nr=50,102 exclusive_caps=1
#   ./de_tracker_stall_test.sh [path-to-de_tracker-bin] [run_dir]
set -u

TRACKER_BIN="${1:-$(dirname "$0")/../../bin/de_tracker}"
RUN_DIR="${2:-/tmp/ca0041_de_tracker}"
DEV_IN=/dev/video50
DEV_OUT=/dev/video102
PORT=61002
SEED='{"ty":"uv","mt":1042,"ms":{"a":1,"b":0.42,"c":0.40,"d":0.22,"e":0.24}}'
ENABLE='{"ty":"uv","mt":1042,"ms":{"a":4}}'
STOP='{"ty":"uv","mt":1042,"ms":{"a":2}}'

say() { echo "[stall-test] $*"; }
# de_databus chunks every datagram: 2-byte LE chunk number, 0xFFFF = last.
# A single \xff\xff-prefixed datagram is a complete one-chunk message.
send() { printf '\xff\xff%s' "$1" > /dev/udp/127.0.0.1/$PORT; }

mkdir -p "$RUN_DIR"
cat > "$RUN_DIR/de_tracker.config.json" <<EOF
{
    "module_id": "VIS_TRK_PRO",
    "s2s_udp_target_ip": "127.0.0.1",
    "s2s_udp_target_port": "61099",
    "s2s_udp_listening_ip": "127.0.0.1",
    "s2s_udp_listening_port": "$PORT",
    "s2s_udp_packet_size": "8192",
    "use_unix_socket": false,
    "camera": {
        "source_video_device": "$DEV_IN",
        "output_video_device": "$DEV_OUT"
    },
    "tracking": {
        "tracker_algorithm_index": 2,
        "camera_orientation": 0,
        "camera_flipped": false,
        "tracking_camera_direction": 4,
        "crosshair": false,
        "de_camera_draw": false
    },
    "advanced_tracking": {
        "frames_to_skip_between_messages": 15,
        "frame_to_skip_between_track_process": 5
    }
}
EOF
cat > "$RUN_DIR/de_tracker.local" <<EOF
{"module_key": "ca0041-test"}
EOF

FEEDER="$(dirname "$0")/mjpeg_feeder"
[ -x "$FEEDER" ] || { echo "build mjpeg_feeder first"; exit 2; }
[ -x "$TRACKER_BIN" ] || { echo "missing tracker binary: $TRACKER_BIN"; exit 2; }

cleanup() {
    [ -n "${TPID:-}" ] && kill "$TPID" 2>/dev/null
    [ -n "${FPID:-}" ] && { kill -CONT "$FPID" 2>/dev/null; kill "$FPID" 2>/dev/null; }
}
trap cleanup EXIT

say "starting feeder on $DEV_IN"
"$FEEDER" "$DEV_IN" 50 > "$RUN_DIR/feeder.log" 2>&1 &
FPID=$!
sleep 1

say "starting de_tracker ($DEV_IN -> $DEV_OUT, bus :$PORT)"
(cd "$RUN_DIR" && exec "$TRACKER_BIN" -c "$RUN_DIR/de_tracker.config.json" \
    --bconfig "$RUN_DIR/de_tracker.local" > "$RUN_DIR/de_tracker.log" 2>&1) &
TPID=$!
sleep 4
kill -0 "$TPID" || { say "de_tracker died at startup"; exit 1; }

say "PHASE 1: enable -> track region"
send "$ENABLE"; sleep 0.5
send "$SEED";  sleep 5
grep -q "DETECTED\|status" "$RUN_DIR/de_tracker.log" && say "tracking running"

say "PHASE 2: unlock (stop) -> relock (region again)"
send "$STOP"; sleep 2
send "$ENABLE"; sleep 0.5
send "$SEED"; sleep 4
kill -0 "$TPID" || { say "de_tracker died on relock"; exit 1; }

say "PHASE 3: stall producer (SIGSTOP), reseed while capture thread is blocked"
kill -STOP "$FPID"
sleep 2
send "$SEED"          # joins the session blocked mid-read, then reseeds
sleep 2
send "$SEED"          # reseed again while the new session is still blocked
sleep 2
kill -CONT "$FPID"    # frames return
say "producer resumed; waiting for tracking to come back"
sleep 8

if kill -0 "$TPID"; then
    say "PASS: de_tracker survived stall + double reseed"
    tail -5 "$RUN_DIR/de_tracker.log"
    RC=0
else
    say "FAIL: de_tracker exited"
    tail -15 "$RUN_DIR/de_tracker.log"
    RC=1
fi

send "$STOP"; sleep 1
exit $RC
