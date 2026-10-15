#!/usr/bin/env python3
"""
CA-0015 demo verifier: reads the bus JSONL tap log + the de_tracker stdout
log from RUN_DIR and prints PASS/FAIL per acceptance item.

  python3 verify_log.py <run_dir>

Checks (mapped to the task acceptance):
  1. 6546 obs frames carry cam_id + tm_capture and a person obs with a
     stable trk across >= 30 frames
  2. a CAPABILITY_RESULT (6544) ok:true for a visual_tracker.lock invoke,
     and MODULE_STATE (6545) shows lock resolved to trk:N
  3. TrackingTargetLocation (1043) messages while locked carry t deltas,
     box (top-left 0..1) and trk
  4. vanish/reappear: the locked person trk disappears from obs then
     reappears with the SAME id, and de_tracker logged a second
     "lock (trk:N)" seed for it
  5. unlock: MODULE_STATE lock returns to "none"

Exit 0 when every check passes.
"""

import json
import re
import sys
import os

results = []


def check(name, ok, detail=""):
    results.append(ok)
    print(f"[{'PASS' if ok else 'FAIL'}] {name}" + (f" — {detail}" if detail else ""))


def load_bus(path):
    rows = []
    with open(path) as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            try:
                rows.append(json.loads(line))
            except json.JSONDecodeError:
                continue
    return rows


def main():
    run_dir = sys.argv[1] if len(sys.argv) > 1 else "/tmp/perceive_lock_demo"
    bus = load_bus(os.path.join(run_dir, "bus.jsonl"))
    tracker_log = ""
    tpath = os.path.join(run_dir, "de_tracker.log")
    if os.path.isfile(tpath):
        tracker_log = open(tpath, errors="replace").read()

    obs_frames = [r for r in bus if r.get("mt") == 6546]
    states = [r for r in bus if r.get("mt") == 6545
              and (r.get("ms") or {}).get("ns") == "visual_tracker"]
    results_ = [r for r in bus if r.get("mt") == 6544]
    locations = [r for r in bus if r.get("mt") == 1043]

    # ---- 1. obs stream: cam_id/tm_capture + stable person trk >= 30 frames
    person_trk_frames = {}
    contract_ok = True
    for r in obs_frames:
        ms = r.get("ms") or {}
        if "cam_id" not in ms or "tm_capture" not in ms or "obs" not in ms:
            contract_ok = False
        for o in ms.get("obs") or []:
            if o.get("cls") == "person" and "trk" in o:
                person_trk_frames.setdefault(o["trk"], 0)
                person_trk_frames[o["trk"]] += 1
    best_trk, best_frames = (None, 0)
    if person_trk_frames:
        best_trk = max(person_trk_frames, key=person_trk_frames.get)
        best_frames = person_trk_frames[best_trk]
    check("6546 obs contract + stable person trk >= 30 frames",
          contract_ok and best_frames >= 30,
          f"{len(obs_frames)} obs frames, best person trk={best_trk} x{best_frames}")

    # ---- 2. lock invoke ok + state.lock = trk:N
    # a broadcast invoke gets a result per module; the ns owner answers
    # ok:true with a "lock" field in data
    lock_ok = any((r.get("ms") or {}).get("ok")
                  and "lock" in ((r.get("ms") or {}).get("data") or {})
                  for r in results_)
    lock_trk = None
    for r in states:
        lk = ((r.get("ms") or {}).get("s") or {}).get("lock")
        if lk and lk.startswith("trk:"):
            lock_trk = int(lk.split(":")[1])
    check("visual_tracker.lock invoke ok + state.lock resolved to trk:N",
          lock_ok and lock_trk is not None,
          f"lock trk={lock_trk}")

    # ---- 3. 1043 carries t deltas + box + trk while locked
    loc_with_all = []
    for r in locations:
        ms = r.get("ms") or {}
        t = ms.get("t")
        box = ms.get("box")
        trk = ms.get("trk")
        if (isinstance(t, list) and len(t) > 0
                and isinstance(box, list) and len(box) == 4
                and trk is not None
                and all(0.0 <= v <= 1.0 for v in box)):
            loc_with_all.append(r)
    check("1043 onTrack messages carry t deltas + box[0..1] + trk",
          len(loc_with_all) >= 3,
          f"{len(loc_with_all)}/{len(locations)} complete")

    # ---- 4. vanish/reappear: same trk back + second seed in tracker log
    reseed_ok = False
    gap_detail = ""
    if lock_trk is not None:
        # find a run of person trk presence with an absence inside
        presence = []
        for r in obs_frames:
            has = any(o.get("cls") == "person" and o.get("trk") == lock_trk
                      for o in (r.get("ms") or {}).get("obs") or [])
            presence.append(has)
        # look for True ... False ... True
        seen_present = seen_gap = False
        for p in presence:
            if p and not seen_present:
                seen_present = True
            elif not p and seen_present and not seen_gap:
                seen_gap = True
            elif p and seen_gap:
                reseed_ok = True
                break
        gap_detail = f"trk:{lock_trk} present->absent->present in obs: {reseed_ok}"
        seeds = re.findall(r"lock \(trk:(\d+)\)", tracker_log)
        same_trk_seeds = seeds.count(str(lock_trk))
        reseed_ok = reseed_ok and same_trk_seeds >= 2
        gap_detail += f"; tracker seed lines for trk:{lock_trk} = {same_trk_seeds}"
    check("vanish/reappear re-seeds on the same trk",
          reseed_ok, gap_detail)

    # ---- 5. unlock -> lock none
    unlocked = any(((r.get("ms") or {}).get("s") or {}).get("lock") == "none"
                   for r in states[::-1][:8])  # check the tail of the run
    check("unlock returns state.lock to none", unlocked)

    print()
    ok = all(results) and results
    print(f"verify: {'PASS' if ok else 'FAIL'} ({sum(results)}/{len(results)} checks)")
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
