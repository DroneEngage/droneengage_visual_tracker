# AI Interaction with Tracker - Scenarios and Behavior

## Overview

This document describes the different interaction scenarios between AI detection and the tracking system, including the new AI priority feature that allows AI to override active tracking under specific conditions.

## Configuration Parameters

### AI Priority Settings
```json
{
  "advanced_tracking": {
    "ai_priority": false,                   // Enable AI priority override
    "ai_priority_distance_threshold": 0.2,  // Distance threshold for override (20% of image diagonal)
    "ai_assisted_recovery_enabled": true,   // Enable AI recovery when tracking is lost
    "ai_confidence_threshold": 0.6,         // Minimum AI confidence for acceptance
    "ai_stability_threshold": 0.75,         // Minimum stability score for re-initialization
    "ai_detection_buffer_size": 5,          // Number of recent detections to consider
    "ai_detection_timeout_ms": 2000         // Maximum age of detections in buffer
  }
}
```

## Tracking Status States

| Status Code | State | Description |
|-------------|-------|-------------|
| 0 | `TRACKING_STOPPED` | Tracking is completely stopped |
| 1 | `TRACKING_ENABLED` | Tracking is ready to start (waiting for first detection) |
| 2 | `TRACKING_LOST` | Tracker lost the target |
| 3 | `TRACKING_DETECTED` | Tracker is actively tracking |

## AI-Tracker Interaction Scenarios

### Scenario 1: AI-Assisted Recovery (Always Available)

**Conditions**: `ai_assisted_recovery_enabled = true`

**When**: Tracker status is `TRACKING_LOST` or `TRACKING_ENABLED`

**Behavior**: AI can reinitialize tracking when it detects a stable object

**Process**:
1. AI detects object with confidence ≥ `ai_confidence_threshold`
2. System checks temporal consistency using detection buffer
3. If stability ≥ `ai_stability_threshold`, tracking is reinitialized
4. Status changes from LOST → DETECTED

**Console Output**:
```
ai_rect (stable):0.35:0.42:0.45:0.38 conf:0.85
```

### Scenario 2: AI Priority Override (When Enabled)

**Conditions**: `ai_priority = true`

**When**: Tracker status is `TRACKING_DETECTED` (actively tracking)

**Behavior**: AI can override current tracking if the detected object is significantly different

**Process**:
1. AI detects object with confidence ≥ `ai_confidence_threshold`
2. System calculates distance between current tracking center and AI detection center
3. If distance > `ai_priority_distance_threshold` × image diagonal, override is considered
4. If stability ≥ `ai_stability_threshold`, tracking switches to AI detection
5. Status remains DETECTED but tracking target changes

**Console Output**:
```
ai_rect (priority override):0.15:0.25:0.40:0.35 conf:0.92
```

### Scenario 3: First Detection After Enable

**Conditions**: After `TRACKING_ENABLE` command

**When**: `m_waiting_for_first_ai_detection = true` and status is `TRACKING_ENABLED`

**Behavior**: First AI detection immediately starts tracking

**Process**:
1. User sends `TRACKING_ENABLE` command
2. System sets `m_waiting_for_first_ai_detection = true`
3. First AI detection with sufficient confidence starts tracking immediately
4. Status changes from ENABLED → DETECTED

**Console Output**:
```
ai_rect (first detection):0.40:0.30:0.50:0.45 conf:0.88
```

### Scenario 4: Manual Tracking Commands

**Conditions**: Manual `TRACKING_POINT` or `TRACKING_REGION` commands

**When**: Any tracking status

**Behavior**: Manual commands always take precedence and are treated as manual (not AI-driven)

**Process**:
1. User sends manual tracking command
2. System immediately switches to manual tracking
3. Status changes to DETECTED
4. AI priority logic may still override if enabled

**Console Output**:
```
rect:0.25:0.35:0.30:0.30 (manual)
```

## Distance Calculation for AI Priority

The AI priority uses Euclidean distance between bounding box centers:

```cpp
// Calculate AI detection center
float ai_center_x = ai_x + ai_w / 2.0f;
float ai_center_y = ai_y + ai_h / 2.0f;

// Get current tracking center (from EMA values)
float tracking_center_x = static_cast<float>(m_ema_x) + 0.5f;
float tracking_center_y = static_cast<float>(m_ema_y) + 0.5f;

// Calculate distance
float distance = std::sqrt(
    std::pow(ai_center_x - tracking_center_x, 2) + 
    std::pow(ai_center_y - tracking_center_y, 2)
);

// Compare with threshold (percentage of image diagonal)
float image_diagonal = std::sqrt(2.0f);
float threshold_distance = image_diagonal * m_ai_priority_distance_threshold;
```

## Temporal Stability Algorithm

AI detections must be temporally consistent to be accepted:

1. **Detection Buffer**: Stores recent AI detections (configurable size)
2. **Timeout Removal**: Removes old detections beyond timeout period
3. **Stability Calculation**: Considers confidence, positional, and temporal factors
4. **Threshold Check**: Requires minimum stability score for acceptance

**Stability Factors**:
- **Confidence Consistency** (40% weight): Average confidence across buffer
- **Positional Stability** (40% weight): Standard deviation of positions
- **Temporal Consistency** (20% weight): Recency of detections

## Control Commands

### Enable/Disable AI Priority

**Enable AI Priority**:
```json
{
  "mt": 1042,
  "ms": {
    "a": 6  // TrackingTarget_ACTION_TRACKING_AI_DRIVER_ENABLED
  }
}
```

**Disable AI Priority**:
```json
{
  "mt": 1042,
  "ms": {
    "a": 7  // TrackingTarget_ACTION_TRACKING_AI_DRIVER_DISABLED
  }
}
```

## Debug Output Examples

### Normal AI-Assisted Recovery
```
TrackingStatus:2
onAITrackerBestRectWithConfidence:2 confidence:0.85
ai_rect (stable):0.35:0.42:0.45:0.38 conf:0.85
rect:0.35:0.42:0.45:0.38 (AI-driven)
TrackingStatus:3
```

### AI Priority Override
```
TrackingStatus:3
onAITrackerBestRectWithConfidence:3 confidence:0.92
AI Priority check - distance: 0.35, threshold: 0.28
ai_rect (priority override):0.15:0.25:0.40:0.35 conf:0.92
rect:0.15:0.25:0.40:0.35 (AI-driven)
```

### Manual Tracking with AI Priority Disabled
```
TrackingStatus:3
// Manual command received
rect:0.25:0.35:0.30:0.30 (manual)
TrackingStatus:3
// AI detections ignored for override when ai_priority=false
onAITrackerBestRectWithConfidence:3 confidence:0.90
```

## Edge Cases and Special Conditions

### Edge Objects (x=0 or y=0)
- Objects at frame edges are valid and should be tracked
- Previous bug: `x > 0` check incorrectly rejected edge objects
- Fixed: Validation now checks dimensions and bounds, not position

### Low Confidence Detections
- Detections below `ai_confidence_threshold` are rejected
- Console: `AI detection filtered out due to low confidence: 0.45`

### Unstable Detections
- Detections failing stability check are rejected
- Console: `AI detection rejected due to temporal instability`

### AI-Driven vs Manual Tracking
- AI-driven updates suppress "tracking off" messages for seamless recovery
- Manual commands always show "tracking off" when stopping previous tracking
- Console indicators: `(AI-driven)` vs `(manual)`

## Performance Considerations

### Detection Buffer Management
- Buffer size affects stability vs responsiveness trade-off
- Larger buffers = more stable but slower response
- Smaller buffers = faster response but less stable

### Distance Threshold Tuning
- Lower threshold = more sensitive to AI changes
- Higher threshold = requires more significant difference before override
- Recommended range: 0.1 to 0.3 (10% to 30% of image diagonal)

### Confidence vs Stability
- High confidence but unstable detections may be rejected
- Moderate confidence but stable detections are preferred
- Balance depends on use case and AI model reliability

## Troubleshooting

### Tracker Immediately Loses AI-Initiated Tracking
**Symptom**: Status changes DETECTED → LOST immediately after AI starts tracking
**Cause**: Edge objects (x=0) incorrectly rejected as invalid
**Solution**: Fixed validation check in tracker.cpp line 391

### AI Priority Not Working
**Symptom**: AI never overrides active tracking
**Causes**:
- `ai_priority` is false
- Distance threshold too high
- AI stability score too low
- AI confidence below threshold

### Too Many AI Overrides
**Symptom**: Tracking constantly switches to different AI detections
**Causes**:
- Distance threshold too low
- AI detection buffer too small
- Stability threshold too low

### AI Recovery Not Working
**Symptom**: Lost tracking never recovers with AI
**Causes**:
- `ai_assisted_recovery_enabled` is false
- AI confidence threshold too high
- AI stability threshold too high
- AI detection timeout too short
