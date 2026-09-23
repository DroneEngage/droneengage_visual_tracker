#include "tracker_facade.hpp"
#include "../de_common/helpers/colors.hpp"
#include "tracker_main.hpp"

using namespace de::tracker;

// Shape id used for the tracking rectangle on de_camera overlay.
// High value to avoid colliding with shapes set by other senders.
static constexpr uint32_t TRACKER_OVERLAY_SHAPE_ID = 9001;

void CTracker_Facade::sendTrackingTargetsLocation(
    const std::string &target_party_id, const Json_de targets_location) const {
  if (targets_location.empty()) {
    return;
  }

  Json_de message = {{"t", targets_location}};

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
 */
void CTracker_Facade::sendCameraOverlayRect(
    const std::string &target_party_id, const float &x, const float &y,
    const float &w, const float &h) const {
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

  Json_de message = {{"a", CAMERA_OVERLAY_ACTION_SET_SHAPE}, {"s", shape}};

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

void CTracker_Facade::sendCameraOverlayRemove(
    const std::string &target_party_id) const {
  Json_de message = {{"a", CAMERA_OVERLAY_ACTION_REMOVE_SHAPE},
                     {"i", TRACKER_OVERLAY_SHAPE_ID}};

  m_module.sendJMSG(target_party_id, message,
                    TYPE_AndruavMessage_CAMERA_OVERLAY_ACTION, true);

#ifdef DEBUG_OVERLAY_MSG
  std::cout << "overlay send REMOVE id:" << TRACKER_OVERLAY_SHAPE_ID
            << std::endl;
#endif
}