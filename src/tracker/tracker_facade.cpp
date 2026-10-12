#include "tracker_facade.hpp"
#include "../de_common/helpers/colors.hpp"
#include "tracker_main.hpp"

using namespace de::tracker;

// Shape ids used for the tracker overlay on de_camera.
// 9001 tracking rect, 9002 crosshair, 9004 failure text.
// High values to avoid colliding with shapes set by other senders.
static constexpr uint32_t TRACKER_OVERLAY_SHAPE_ID = 9001;
static constexpr uint32_t TRACKER_OVERLAY_CROSSHAIR_ID = 9002;
static constexpr uint32_t TRACKER_OVERLAY_TEXT_ID = 9004;

void CTracker_Facade::sendTrackingTargetsLocation(
    const std::string &target_party_id, const Json_de targets_location,
    const Json_de extra) const {
  if (targets_location.empty()) {
    return;
  }

  Json_de message = {{"t", targets_location}};
  if (extra.is_object()) {
    for (auto it = extra.begin(); it != extra.end(); ++it) message[it.key()] = it.value();
  }

#ifdef DDEBUG
  std::cout << "tracking:" << targets_location.dump() << std::endl;
#endif
#ifdef DDEBUG
  std::cout << _INFO_CONSOLE_BOLD_TEXT << "onTrack >> "
            << _LOG_CONSOLE_BOLD_TEXT << targets_location.dump()
            << _NORMAL_CONSOLE_TEXT_ << std::endl;
#endif
  m_module.sendJMSG(target_party_id, message,
                    TYPE_AndruavMessage_TrackingTargetLocation, true);
}

void CTracker_Facade::sendTrackingTargetStatus(
    const std::string &target_party_id, const int status) const {
  de::tracker::CTrackerMain &m_tracker_main =
      de::tracker::CTrackerMain::getInstance();

  const uint8_t tracking_camera_direction =
      m_tracker_main.getTrackingCameraDirection();
  const bool ai_priority = m_tracker_main.getAiPriority();
  Json_de message = {
      {"a", status},
      {"b", tracking_camera_direction},
      {"c", ai_priority}
  };

  m_module.sendJMSG(target_party_id, message,
                    TYPE_AndruavMessage_TrackingTarget_STATUS, true);

#ifdef DEBUG
  std::cout << "TrackingStatus:" << status << std::endl;
#endif
}

void CTracker_Facade::sendTrackingConfig(
    const std::string &target_party_id) const {
  de::tracker::CTrackerMain &sendTrackingConfig =
      de::tracker::CTrackerMain::getInstance();

  Json_de message = {{"a", 2}};

  m_module.sendJMSG(target_party_id, message,
                    TYPE_AndruavMessage_TrackingTarget_STATUS, true);

#ifdef DEBUG
  std::cout << "TrackingStatus:" << "TO BE IMPLEMENTED" << std::endl;
#endif
}

/**
 * Sends the tracking rectangle to de_camera to be drawn by its internal
 * overlay library. Coordinates are normalized [0..1] with (0,0) top-left.
 * Sends a full SET_SHAPE (with style) on the first rect and every
 * OVERLAY_FULL_RESEND_INTERVAL updates so de_camera recovers the shape after
 * a restart or a CLEAR; in between, slim MOVE_SHAPE updates carry only the
 * geometry. MOVE_SHAPE to an unknown id is ignored by de_camera.
 */
void CTracker_Facade::sendCameraOverlayRect(
    const std::string &target_party_id, const float &x, const float &y,
    const float &w, const float &h) const {
  static constexpr uint32_t OVERLAY_FULL_RESEND_INTERVAL = 150;

  const bool send_full =
      (m_overlay_send_count % OVERLAY_FULL_RESEND_INTERVAL) == 0;
  ++m_overlay_send_count;

  Json_de message;
  if (send_full) {
    Json_de shape = {
        {"id", TRACKER_OVERLAY_SHAPE_ID},
        {"type", "rect"},
        {"x1", x},
        {"y1", y},
        {"x2", x + w},
        {"y2", y + h},
        {"thickness", 2},
        {"filled", false},
        {"color", {200, 0, 0, 255}}};

    message = {{"a", CAMERA_OVERLAY_ACTION_SET_SHAPE}, {"s", shape}};
  } else {
    message = {{"a", CAMERA_OVERLAY_ACTION_MOVE_SHAPE},
               {"i", TRACKER_OVERLAY_SHAPE_ID},
               {"x1", x},
               {"y1", y},
               {"x2", x + w},
               {"y2", y + h}};
  }

  m_module.sendJMSG(target_party_id, message,
                    TYPE_AndruavMessage_CAMERA_OVERLAY_ACTION, true);

#ifdef DEBUG_OVERLAY_MSG
  static int overlay_send_counter = 0;
  if ((overlay_send_counter++ % 30) == 0) {
    std::cout << "overlay send rect:" << x << "," << y << " " << w << "x" << h
              << std::endl;
  }
#endif
}

/**
 * Sends the center crosshair '+' to de_camera as a single "crosshair" shape:
 * x1/y1 is the centre and radius is the '+' arm half-length as a fraction of
 * frame height (same px length on both axes, matching the local drawing).
 * The geometry is static so a SET_SHAPE is sent only on the first update and
 * every OVERLAY_FULL_RESEND_INTERVAL calls so de_camera recovers the shape
 * after a restart or a CLEAR.
 */
void CTracker_Facade::sendCameraOverlayCrosshair(
    const std::string &target_party_id, const float &cx, const float &cy,
    const float &arm) const {
  static constexpr uint32_t OVERLAY_FULL_RESEND_INTERVAL = 150;

  if ((m_overlay_crosshair_send_count % OVERLAY_FULL_RESEND_INTERVAL) != 0) {
    ++m_overlay_crosshair_send_count;
    return;
  }
  ++m_overlay_crosshair_send_count;

  Json_de shape = {
      {"id", TRACKER_OVERLAY_CROSSHAIR_ID},
      {"type", "crosshair"},
      {"x1", cx},
      {"y1", cy},
      {"radius", arm},
      {"thickness", 2},
      {"color", {0, 255, 0, 255}}};

  Json_de message = {{"a", CAMERA_OVERLAY_ACTION_SET_SHAPE}, {"s", shape}};

  m_module.sendJMSG(target_party_id, message,
                    TYPE_AndruavMessage_CAMERA_OVERLAY_ACTION, true);

#ifdef DEBUG_OVERLAY_MSG
  std::cout << "overlay send crosshair:" << cx << "," << cy << std::endl;
#endif
}

/**
 * Sends a status text line to de_camera. MOVE_SHAPE cannot update text so a
 * full SET_SHAPE is sent on the first update and every
 * OVERLAY_FULL_RESEND_INTERVAL calls.
 */
void CTracker_Facade::sendCameraOverlayText(
    const std::string &target_party_id, const float &x, const float &y,
    const float &text_height, const std::string &text) const {
  static constexpr uint32_t OVERLAY_FULL_RESEND_INTERVAL = 150;

  if ((m_overlay_text_send_count % OVERLAY_FULL_RESEND_INTERVAL) != 0) {
    ++m_overlay_text_send_count;
    return;
  }
  ++m_overlay_text_send_count;

  Json_de shape = {
      {"id", TRACKER_OVERLAY_TEXT_ID},
      {"type", "text"},
      {"x1", x},
      {"y1", y},
      {"text_height", text_height},
      {"text", text},
      {"color", {200, 0, 0, 255}}};

  Json_de message = {{"a", CAMERA_OVERLAY_ACTION_SET_SHAPE}, {"s", shape}};

  m_module.sendJMSG(target_party_id, message,
                    TYPE_AndruavMessage_CAMERA_OVERLAY_ACTION, true);

#ifdef DEBUG_OVERLAY_MSG
  std::cout << "overlay send text:'" << text << "'" << std::endl;
#endif
}

void CTracker_Facade::sendCameraOverlayRemoveRect(
    const std::string &target_party_id) const {
  m_overlay_send_count = 0;

  Json_de message = {{"a", CAMERA_OVERLAY_ACTION_REMOVE_SHAPE},
                     {"i", TRACKER_OVERLAY_SHAPE_ID}};

  m_module.sendJMSG(target_party_id, message,
                    TYPE_AndruavMessage_CAMERA_OVERLAY_ACTION, true);

#ifdef DEBUG_OVERLAY_MSG
  std::cout << "overlay send REMOVE id:" << TRACKER_OVERLAY_SHAPE_ID
            << std::endl;
#endif
}

void CTracker_Facade::sendCameraOverlayRemoveText(
    const std::string &target_party_id) const {
  m_overlay_text_send_count = 0;

  Json_de message = {{"a", CAMERA_OVERLAY_ACTION_REMOVE_SHAPE},
                     {"i", TRACKER_OVERLAY_TEXT_ID}};

  m_module.sendJMSG(target_party_id, message,
                    TYPE_AndruavMessage_CAMERA_OVERLAY_ACTION, true);

#ifdef DEBUG_OVERLAY_MSG
  std::cout << "overlay send REMOVE id:" << TRACKER_OVERLAY_TEXT_ID
            << std::endl;
#endif
}

void CTracker_Facade::sendCameraOverlayRemove(
    const std::string &target_party_id) const {
  m_overlay_send_count = 0;
  m_overlay_crosshair_send_count = 0;
  m_overlay_text_send_count = 0;

  Json_de message = {
      {"a", CAMERA_OVERLAY_ACTION_REMOVE_SHAPE},
      {"i", {TRACKER_OVERLAY_SHAPE_ID, TRACKER_OVERLAY_CROSSHAIR_ID,
             TRACKER_OVERLAY_TEXT_ID}}};

  m_module.sendJMSG(target_party_id, message,
                    TYPE_AndruavMessage_CAMERA_OVERLAY_ACTION, true);

#ifdef DEBUG_OVERLAY_MSG
  std::cout << "overlay send REMOVE ids:" << TRACKER_OVERLAY_SHAPE_ID << ","
            << TRACKER_OVERLAY_CROSSHAIR_ID << "," << TRACKER_OVERLAY_TEXT_ID
            << std::endl;
#endif
}

/**
 * AI_Recognition_ACTION (1076) to the AI module (droneengage_yolo_ai) over
 * the intermodule path - used by the visual_tracker start/stop actions.
 */
void CTracker_Facade::sendAIRecognitionAction(const Json_de &cmd) const {
  m_module.sendJMSG(std::string(""), cmd,
                    TYPE_AndruavMessage_AI_Recognition_ACTION, true);
}
