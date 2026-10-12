# AI Interaction with Tracker

de_tracker holds a target for visual servoing. It does **not** filter AI output:
the perceiver (`de_perceive`) already publishes only confirmed tracks with
stable ids (`AI_OBSERVATIONS`, 6546). de_tracker is told which track to hold.

## Lock-on

| Command | Effect |
|---|---|
| `visual_tracker.lock {trk: N}` | hold track N only; re-seed when N vanishes and reappears |
| `visual_tracker.lock {class: "person"}` | hold the first confirmed track of the class (lowest id) |
| `TrackingTarget_ACTION_TRACKING_ENABLE` | `lock {class: ""}`: first confirmed track of any class |
| `visual_tracker.unlock`, `TRACKING_STOP` | drop the lock |
| manual region / point, `visual_tracker.track` | drop the lock, track that region |

The seed is the locked observation's `box` (`[x,y,w,h]`, normalized, top-left,
0..1); the pixel tracker (CSRT etc.) then follows it frame to frame.

## Configuration (`advanced_tracking`)

```jsonc
"lock_class_reacquire_frames": 15,  // class lock takes another track after this many absent frames
"lock_reseed_cooldown_frames": 10,  // min frames between re-seeds while the pixel tracker is not tracking
"ai_priority": true,                // re-seed when the pixel tracker drifts from the locked track
"ai_priority_distance_threshold": 0.3,
"ai_assisted_recovery_enabled": true,
"tracker_lost_timeout_ms": 10000
```

The former `ai_confidence_threshold`, `ai_stability_threshold`,
`ai_detection_buffer_size` and `ai_detection_timeout_ms` are gone: confidence
and temporal consistency are the track stage's job (`de_perceive` `"track"`).

## Tests

`make -C test test` — lock-on selection rules, the top-left box convention,
and the `visual_tracker` capability advert against `de_common` `capValidateAdvert`.
