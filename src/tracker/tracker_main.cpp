#include <algorithm>
#include <iostream>
#include <stdio.h>
#include <cmath>

#include "../de_common/de_databus/configFile.hpp"
#include "../de_common/de_databus/messages.hpp"
#include "../de_common/helpers/colors.hpp"
#include "../de_common/helpers/helpers.hpp"
#include "tracker.hpp"
#include "tracker_main.hpp"
#include "tracker_capabilities.hpp"
#include "video.hpp"
#include "../version.hpp"

using namespace de::tracker;

void CTrackerMain::loopScheduler() {
  while (!m_exit_thread) {
    // timer each 10m sec.
    wait_time_nsec(0, 10000000);

    m_counter++;

    
    if (m_counter % 500 == 0) { // 5 sec
      de::CConfigFile &cConfigFile = de::CConfigFile::getInstance();
      const bool updated = cConfigFile.fileUpdated();
      if (updated) {
        cConfigFile.reloadFile();

        reloadParametersIfConfigChanged();
        std::cout << _ERROR_CONSOLE_BOLD_TEXT_ "Config file updated"
                  << _NORMAL_CONSOLE_TEXT_ << std::endl;
      }

      // visual_tracker.start needs the AI class list; keep asking until the
      // AI module answers (it may start after this module)
      bool class_list_empty;
      {
        std::lock_guard<std::mutex> lock(m_ai_class_lock);
        class_list_empty = m_ai_class_names.empty();
      }
      if (class_list_empty) requestAIClassList();
    }
  }
}

bool CTrackerMain::init() {
  m_exit_thread = false;

  bool res = readConfigParameters();
  if (res == false) {
    return false;
  }

  m_tracker = std::make_unique<CTracker>(this);

  bool ok = m_tracker.get()->init(
      m_tracker_algorithm_index, m_source_video_device,
      m_output_video_device,
      m_frames_to_skip_between_messages, m_frame_to_skip_between_track_process,
      m_desired_input_width, m_desired_input_height, m_crosshair,
      m_de_camera_draw);
  if (ok == false) {
    std::cout << _ERROR_CONSOLE_BOLD_TEXT_
              << "FATAL ERROR:" << _INFO_CONSOLE_TEXT
              << " Failed to initialize tracker. " << _NORMAL_CONSOLE_TEXT_
              << std::endl;
    return false;
  }

  m_tracker.get()->track(-1, 0, 0);

  m_scheduler_thread = std::thread{[&]() { loopScheduler(); }};

  return true;
}

void CTrackerMain::reloadParametersIfConfigChanged() {
    Json_de m_jsonConfig = CConfigFile::getInstance().GetConfigJSON();
    if (!m_jsonConfig.contains("tracking")) {
    std::cout << _ERROR_CONSOLE_BOLD_TEXT_
              << "FATAL ERROR: " << _INFO_CONSOLE_TEXT
              << CConfigFile::getInstance().getFileName()
              << " does not have field " << _ERROR_CONSOLE_TEXT_ << "[tracking]"
              << _NORMAL_CONSOLE_TEXT_ << std::endl;

    return;
  }

  Json_de tracking = m_jsonConfig["tracking"];
  if (tracking.contains("tracking_camera_direction")) {

    if (m_tracking_camera_direction > TRACKING_CAMERA_DIRECTION_UP) {
      std::cout << _ERROR_CONSOLE_BOLD_TEXT_
                << "FATAL ERROR:" << _INFO_CONSOLE_TEXT
                << " invalid tracking_camera_direction: "
                << _ERROR_CONSOLE_TEXT_ << (int)m_tracking_camera_direction
                << std::endl
                << "Assuming Default Camera Forward" << _NORMAL_CONSOLE_TEXT_
                << std::endl;
      m_tracking_camera_direction = TRACKING_CAMERA_DIRECTION_NONE;
    }
    else
    {
      m_tracking_camera_direction = tracking["tracking_camera_direction"].get<uint16_t>();
    }
  }

   std::cout << _LOG_CONSOLE_BOLD_TEXT
            << "Using tracking_camera_direction:" << _INFO_CONSOLE_BOLD_TEXT
            << m_tracking_camera_direction << _NORMAL_CONSOLE_TEXT_
            << std::endl;


  m_camera_flipped = false;
  if (tracking.contains("camera_flipped")) {
    m_camera_flipped = tracking["camera_flipped"].get<bool>();
    
    std::cout << _LOG_CONSOLE_BOLD_TEXT
                  << "Using camera_flipped:" << _INFO_CONSOLE_BOLD_TEXT
                  << m_camera_flipped << _NORMAL_CONSOLE_TEXT_
                  << std::endl;
  }

  m_crosshair = true;
  if (tracking.contains("crosshair")) {
    m_crosshair = tracking["crosshair"].get<bool>();
    
    std::cout << _LOG_CONSOLE_BOLD_TEXT
                  << "Using crosshair:" << _INFO_CONSOLE_BOLD_TEXT
                  << m_crosshair << _NORMAL_CONSOLE_TEXT_
                  << std::endl;
  }

  m_de_camera_draw = false;
  if (tracking.contains("de_camera_draw")) {
    m_de_camera_draw = tracking["de_camera_draw"].get<bool>();

    std::cout << _LOG_CONSOLE_BOLD_TEXT
                  << "Using de_camera_draw:" << _INFO_CONSOLE_BOLD_TEXT
                  << m_de_camera_draw << _NORMAL_CONSOLE_TEXT_
                  << std::endl;
  }

  // m_tracker is null during the first read inside init(); the value is
  // passed to CTracker::init there. On later config reloads propagate it.
  if (m_tracker) {
    m_tracker->setCameraDraw(m_de_camera_draw);
  }

  
  m_camera_orientation = DEF_TRACK_ORIENTATION_DEG_0;
  if (tracking.contains("camera_orientation")) {
    m_camera_orientation = tracking["camera_orientation"].get<uint16_t>();
    
    std::cout << _LOG_CONSOLE_BOLD_TEXT
                  << "Using camera_orientation:" << _INFO_CONSOLE_BOLD_TEXT
                  << m_camera_orientation << _NORMAL_CONSOLE_TEXT_
                  << std::endl;
  }
    
}

bool CTrackerMain::readConfigParameters() {
  Json_de m_jsonConfig = CConfigFile::getInstance().GetConfigJSON();
  if (!m_jsonConfig.contains("tracking")) {
    std::cout << _ERROR_CONSOLE_BOLD_TEXT_
              << "FATAL ERROR: " << _INFO_CONSOLE_TEXT
              << CConfigFile::getInstance().getFileName()
              << " does not have field " << _ERROR_CONSOLE_TEXT_ << "[tracking]"
              << _NORMAL_CONSOLE_TEXT_ << std::endl;

    return false;
  } else {

    reloadParametersIfConfigChanged();

    

    Json_de tracking = m_jsonConfig["tracking"];

    if (tracking.contains("tracker_algorithm_index")) {
      m_tracker_algorithm_index = tracking["tracker_algorithm_index"]
                                      .get<de::tracker::ENUM_TRACKER_TYPE>();
    }
  }

  m_source_video_device = "";
  m_output_video_device = "";
  m_desired_input_width = 0;
  m_desired_input_height = 0;

  if (m_jsonConfig.contains("camera")) {
    Json_de camera = m_jsonConfig["camera"];

    if (camera.contains("source_video_device_name")) {
      const int video_index = CVideo::findVideoDeviceIndex(
          camera["source_video_device_name"].get<std::string>());
      if (video_index != -1) {
        m_source_video_device = "/dev/video" + std::to_string(video_index);

        std::cout << _LOG_CONSOLE_BOLD_TEXT << "Using source_video_device_name:"
                  << _INFO_CONSOLE_BOLD_TEXT << camera["source_video_device_name"].get<std::string>()
                  << " -> " << m_source_video_device
                  << _NORMAL_CONSOLE_TEXT_ << std::endl;
      }
    }

    if (m_source_video_device.empty()) {
      if (!camera.contains("source_video_device")) {
        std::cout << _ERROR_CONSOLE_BOLD_TEXT_
                  << "FATAL ERROR: " << _INFO_CONSOLE_TEXT
                  << CConfigFile::getInstance().getFileName()
                  << " does not have field " << _ERROR_CONSOLE_TEXT_
                  << "[source_video_device]" << _NORMAL_CONSOLE_TEXT_
                  << std::endl;

        return false;
      } else {
        m_source_video_device =
            camera["source_video_device"].get<std::string>();

        std::cout << _LOG_CONSOLE_BOLD_TEXT
                  << "Using source_video_device:" << _INFO_CONSOLE_BOLD_TEXT
                  << m_source_video_device << _NORMAL_CONSOLE_TEXT_
                  << std::endl;
      }
    }

    if (camera.contains("output_video_device_name")) {
      const int video_index = CVideo::findVideoDeviceIndex(
          camera["output_video_device_name"].get<std::string>());
      if (video_index != -1) {
        m_output_video_device = "/dev/video" + std::to_string(video_index);

        std::cout << _SUCCESS_CONSOLE_BOLD_TEXT_
                  << "Using output_video_device_name:"
                  << _INFO_CONSOLE_BOLD_TEXT << camera["output_video_device_name"].get<std::string>()
                  << " -> " << m_output_video_device
                  << _NORMAL_CONSOLE_TEXT_ << std::endl;
      }
    }

    if (m_output_video_device.empty()) {
      if (!camera.contains("output_video_device")) {
        std::cout << _ERROR_CONSOLE_BOLD_TEXT_
                  << "FATAL ERROR:" << _INFO_CONSOLE_TEXT
                  << " No output_video_device specified in config.json"
                  << _NORMAL_CONSOLE_TEXT_ << std::endl;
        return false;
      } else {
        m_output_video_device =
            camera["output_video_device"].get<std::string>();

        std::cout << _SUCCESS_CONSOLE_BOLD_TEXT_
                  << "Using output_video_device:" << _INFO_CONSOLE_BOLD_TEXT
                  << m_output_video_device << _NORMAL_CONSOLE_TEXT_
                  << std::endl;
      }
    }

    // Optional desired input resolution
    if (camera.contains("desired_input_width") &&
        camera.contains("desired_input_height")) {
      m_desired_input_width = camera["desired_input_width"].get<int>();
      m_desired_input_height = camera["desired_input_height"].get<int>();
      if (m_desired_input_width < 0 || m_desired_input_height < 0) {
        m_desired_input_width = 0;
        m_desired_input_height = 0;
      }
    }

    // If desired size not provided, display max available video size
    if (m_desired_input_width == 0 && m_desired_input_height == 0 &&
        !m_source_video_device.empty()) {
      unsigned int max_w = 0, max_h = 0;
      if (CVideo::getMaxSupportedResolution(m_source_video_device, max_w,
                                            max_h)) {
        std::cout << _SUCCESS_CONSOLE_BOLD_TEXT_
                  << "Camera max supported resolution: " << _INFO_CONSOLE_TEXT
                  << max_w << "x" << max_h << _NORMAL_CONSOLE_TEXT_
                  << std::endl;
      }
    }
  } else {
    std::cout << _ERROR_CONSOLE_BOLD_TEXT_
              << "FATAL ERROR:" << _INFO_CONSOLE_TEXT
              << " No camera specified in config.json" << _NORMAL_CONSOLE_TEXT_
              << std::endl;
    return false;
  }

  m_frames_to_skip_between_messages = FRAMES_TO_SKIP_BETWEEN_MESSAGES;
  m_frame_to_skip_between_track_process = FRAMES_TO_SKIP_BETWEEN_TRACK_PROCESS;

  if (!m_jsonConfig.contains("advanced_tracking")) {
    std::cout << _INFO_CONSOLE_BOLD_TEXT
              << "Field not found in config.json: " << _INFO_CONSOLE_TEXT
              << "[advanced_tracking]" << _NORMAL_CONSOLE_TEXT_ << std::endl;
    std::cout << _SUCCESS_CONSOLE_BOLD_TEXT_ << "Default Values will be used  "
              << _NORMAL_CONSOLE_TEXT_ << std::endl;

  } else {
    Json_de advanced_tracking = m_jsonConfig["advanced_tracking"];
    if (advanced_tracking.contains("frames_to_skip_between_messages")) {
      m_frames_to_skip_between_messages =
          advanced_tracking["frames_to_skip_between_messages"].get<uint16_t>();
    }
    m_frames_to_skip_between_messages = m_frames_to_skip_between_messages == 0
                                            ? 1
                                            : m_frames_to_skip_between_messages;

    if (advanced_tracking.contains("frame_to_skip_between_track_process")) {
      m_frame_to_skip_between_track_process =
          advanced_tracking["frame_to_skip_between_track_process"]
              .get<uint16_t>();
    }
    m_frame_to_skip_between_track_process =
        m_frame_to_skip_between_track_process == 0
            ? 1
            : m_frame_to_skip_between_track_process;

    if (advanced_tracking.contains("ema_alpha_base")) {
      m_ema_alpha_base = advanced_tracking["ema_alpha_base"].get<double>();
    }

    // Lock-on parameters
    if (advanced_tracking.contains("lock_class_reacquire_frames")) {
      m_lock_class_reacquire_frames = advanced_tracking["lock_class_reacquire_frames"].get<int>();
    }

    if (advanced_tracking.contains("lock_reseed_cooldown_frames")) {
      m_lock_reseed_cooldown_frames = advanced_tracking["lock_reseed_cooldown_frames"].get<int>();
    }

    // Load tracker recovery parameters
    if (advanced_tracking.contains("ai_assisted_recovery_enabled")) {
      m_ai_assisted_recovery_enabled = advanced_tracking["ai_assisted_recovery_enabled"].get<bool>();
    }
    
    if (advanced_tracking.contains("ai_priority")) {
      m_ai_priority = advanced_tracking["ai_priority"].get<bool>();
    }
    
    if (advanced_tracking.contains("ai_priority_distance_threshold")) {
      m_ai_priority_distance_threshold = advanced_tracking["ai_priority_distance_threshold"].get<float>();
    }
    
    if (advanced_tracking.contains("tracker_lost_timeout_ms")) {
      m_tracker_lost_timeout_ms = advanced_tracking["tracker_lost_timeout_ms"].get<int>();
    }

    std::cout << _SUCCESS_CONSOLE_BOLD_TEXT_
              << "Field Found: advanced_tracking field found:  "
              << _INFO_CONSOLE_TEXT
              << "Following values will be used:" << _NORMAL_CONSOLE_TEXT_
              << std::endl;
  }

  std::cout << _INFO_CONSOLE_TEXT
            << "frames_to_skip_between_messages: " << _LOG_CONSOLE_BOLD_TEXT
            << m_frames_to_skip_between_messages << _INFO_CONSOLE_TEXT
            << ", frame_to_skip_between_track_process: "
            << _LOG_CONSOLE_BOLD_TEXT << m_frame_to_skip_between_track_process
            << _INFO_CONSOLE_TEXT
            << ", ema_alpha_base: " << _LOG_CONSOLE_BOLD_TEXT
            << m_ema_alpha_base << _INFO_CONSOLE_TEXT
            << ", lock_class_reacquire_frames: " << _LOG_CONSOLE_BOLD_TEXT
            << m_lock_class_reacquire_frames << _INFO_CONSOLE_TEXT
            << ", lock_reseed_cooldown_frames: " << _LOG_CONSOLE_BOLD_TEXT
            << m_lock_reseed_cooldown_frames << _INFO_CONSOLE_TEXT
            << ", ai_assisted_recovery_enabled: " << _LOG_CONSOLE_BOLD_TEXT
            << (m_ai_assisted_recovery_enabled ? "true" : "false") << _INFO_CONSOLE_TEXT
            << ", ai_priority: " << _LOG_CONSOLE_BOLD_TEXT
            << (m_ai_priority ? "true" : "false") << _INFO_CONSOLE_TEXT
            << ", ai_priority_distance_threshold: " << _LOG_CONSOLE_BOLD_TEXT
            << m_ai_priority_distance_threshold << _INFO_CONSOLE_TEXT
            << ", tracker_lost_timeout_ms: " << _LOG_CONSOLE_BOLD_TEXT
            << m_tracker_lost_timeout_ms << _NORMAL_CONSOLE_TEXT_ << std::endl;

  return true;
}

bool CTrackerMain::uninit() {
  // exit thread.
  if (m_exit_thread == true) {
    std::cout << "m_exit_thread == true" << std::endl;
    return true;
  }

  m_exit_thread = true;

  if (m_scheduler_thread.joinable()) {
    m_scheduler_thread.join();
  }

  if (m_tracker) {
    m_tracker.get()->uninit();
  }

  return true;
}

void CTrackerMain::startTrackingRect(const float x, const float y,
                                     const float w, const float h, bool is_ai_driven) {
#ifdef DEBUG
  std::cout << _INFO_CONSOLE_BOLD_TEXT << "rect:" << x << ":" << y << ":" << w
            << ":" << h << (is_ai_driven ? " (AI-driven)" : " (manual)") << std::endl;
#endif
  (void)is_ai_driven;

  if (m_tracker_status == TrackingTarget_STATUS_TRACKING_STOPPED)
    return;

  // an explicit region overrides any lock-on
  unlock();

  m_tracker.get()->stop();
  m_tracker.get()->trackRect(x, y, w, h, false);  // Manual

  m_tracker_facade.sendTrackingTargetStatus(std::string(""), m_tracker_status);
}

/**
 * (Re)start the pixel tracker on a box taken from a locked observation.
 * x,y top-left, all normalized 0..1.
 */
void CTrackerMain::seedTracker(const float x, const float y, const float w, const float h) {
  if (m_tracker_status == TrackingTarget_STATUS_TRACKING_STOPPED)
    return;

  m_tracker.get()->stop();
  m_tracker.get()->trackRect(x, y, w, h, true);  // observation-driven

  m_tracker_facade.sendTrackingTargetStatus(std::string(""), m_tracker_status);
}

void CTrackerMain::enableTracking() {
  // this state means I will accept start tracking point.
  // this is not a real start for the tracker core.
  m_tracker_status = TrackingTarget_STATUS_TRACKING_ENABLED;
  
  // accept the first confirmed track of any class as the lock target
  lockClass("");

  // ACK
  m_tracker_facade.sendTrackingTargetStatus(std::string(""), m_tracker_status);
  publishCapabilityState();
}

void CTrackerMain::pauseTracking() { m_tracker.get()->pause(); }

void CTrackerMain::stopTracking() {
  m_tracker_status = TrackingTarget_STATUS_TRACKING_STOPPED;
  m_tracker.get()->stop();
  
  unlock();

  m_tracker_facade.sendTrackingTargetStatus(std::string(""), m_tracker_status);
  publishCapabilityState();
}

/**
 * Called when there is a a tracked object.
 * input x,y,w,h:[0 to 1.0]
 * output from [-0.5 to 0.5]
 * (0,0) top left
 * center = [(x + w )/2 , (y + h)/2]
 */
void CTrackerMain::onTrack(const float &x, const float &y, const float &width,
                           const float &height,
                           const bool should_skip_message) {

  if (m_tracking_camera_direction == TRACKING_CAMERA_DIRECTION_NONE) {
    return;
  }

  double center_x = -0.5 + x + width / 2.0f;
  double center_y = -0.5 + y + height / 2.0f;

  const double rx = center_x; // raw normalized [-0.5..0.5]
  const double ry = center_y;

  if (!m_ema_init) {
    m_ema_x = rx;
    m_ema_y = ry;
    m_ema_init = true;
  } else {
    // optional adaptive alpha: more responsive on big moves
    const double mag = std::max(std::abs(rx - m_ema_x), std::abs(ry - m_ema_y));
    const double alpha = std::clamp(m_ema_alpha_base + 0.5 * mag, 0.1, 0.8);
    m_ema_x = alpha * rx + (1.0 - alpha) * m_ema_x;
    m_ema_y = alpha * ry + (1.0 - alpha) * m_ema_y;
  }

  double delta_x, delta_y, delta_z;
  delta_z = 0.0;

  switch (m_camera_orientation) {
  case DEF_TRACK_ORIENTATION_DEG_0:
    delta_x = m_ema_x;
    delta_y = m_ema_y;
    break;
  case DEF_TRACK_ORIENTATION_DEG_90:
    delta_x = m_ema_y;
    delta_y = -m_ema_x;
    break;

  case DEF_TRACK_ORIENTATION_DEG_180:
    delta_x = -m_ema_x;
    delta_y = -m_ema_y;
    break;

  case DEF_TRACK_ORIENTATION_DEG_270:
    delta_x = -m_ema_y;
    delta_y = -m_ema_x;
    break;

  default:
    std::cerr << _ERROR_CONSOLE_BOLD_TEXT_
              << "Invalid camera orientation: " << m_camera_orientation
              << _NORMAL_CONSOLE_TEXT_ << std::endl;
    break;
  }

  if (m_camera_flipped) {
    delta_x = -delta_x;
  }

  if (m_tracking_camera_direction == TRACKING_CAMERA_DIRECTION_BACK) {
    // camera installed backward
    delta_x = -delta_x;
    delta_y = -delta_y;
  }

#ifdef DDEBUG
  std::cout << "Track Object:" << center_x << ":" << center_y << std::endl;
#endif

  // Apply precision limiting
  delta_x = roundToPrecision(delta_x, 3);
  delta_y = roundToPrecision(delta_y, 3);

  Json_de targets = Json_de::array();

  switch (m_tracking_camera_direction) {
  case TRACKING_CAMERA_DIRECTION_FRONT:
  case TRACKING_CAMERA_DIRECTION_BACK:
    targets.push_back({{"x", delta_x}, {"y", -delta_y}});
    break;

  case TRACKING_CAMERA_DIRECTION_DOWN:
    delta_z = delta_y;
    targets.push_back({{"x", delta_x}, {"y", delta_z}});
    break;

  case TRACKING_CAMERA_DIRECTION_UP:
    delta_z = -delta_y;
    targets.push_back({{"x", delta_x}, {"y", delta_z}});
    break;

  default:
    return;
  }

#ifdef DDEBUG
  std::cout << _INFO_CONSOLE_BOLD_TEXT << "onTrack >> "
            << _LOG_CONSOLE_BOLD_TEXT << "  x:" << _INFO_CONSOLE_BOLD_TEXT << x
            << _LOG_CONSOLE_BOLD_TEXT << "  y:" << _INFO_CONSOLE_BOLD_TEXT << y
            << _LOG_CONSOLE_BOLD_TEXT << "  w:" << _INFO_CONSOLE_BOLD_TEXT
            << width << _LOG_CONSOLE_BOLD_TEXT
            << "  h:" << _INFO_CONSOLE_BOLD_TEXT << height
            << _LOG_CONSOLE_BOLD_TEXT << "  dx:" << _INFO_CONSOLE_BOLD_TEXT
            << delta_x << _LOG_CONSOLE_BOLD_TEXT
            << "  dy:" << _INFO_CONSOLE_BOLD_TEXT << delta_y
            << _LOG_CONSOLE_BOLD_TEXT << "  dz:" << _INFO_CONSOLE_BOLD_TEXT
            << delta_z << _NORMAL_CONSOLE_TEXT_ << std::endl;
#endif

  if (!should_skip_message) {
    // deltas stay centred (-0.5..0.5) for DE-Pilot; the tracked box rides along
    // in the stack-wide convention: [x,y,w,h] normalized, top-left, 0..1
    Json_de extra = Json_de::object({{"box", de::tracker::boxToWire(x, y, width, height)}});
    {
      std::lock_guard<std::mutex> lock(m_lock_mutex);
      if (m_lock.locked() && m_lock.trk() >= 0) extra["trk"] = m_lock.trk();
    }
    m_tracker_facade.sendTrackingTargetsLocation(std::string(""), targets, extra);
  }
}

/**
 * Called by CTracker when de_camera_draw is enabled and the tracked object
 * rectangle is updated. Coordinates are normalized [0..1], (0,0) top-left.
 */
void CTrackerMain::onCameraOverlayRect(const float &x, const float &y,
                                       const float &width,
                                       const float &height) {
  m_tracker_facade.sendCameraOverlayRect(std::string(""), x, y, width, height);
}

/**
 * Called by CTracker when de_camera_draw is enabled and the center crosshair
 * should be (re)sent. Coordinates are normalized [0..1], (0,0) top-left.
 */
void CTrackerMain::onCameraOverlayCrosshair(const float &cx, const float &cy,
                                            const float &arm) {
  m_tracker_facade.sendCameraOverlayCrosshair(std::string(""), cx, cy, arm);
}

/**
 * Called by CTracker when de_camera_draw is enabled and a status text should
 * be shown on the de_camera overlay.
 */
void CTrackerMain::onCameraOverlayText(const float &x, const float &y,
                                       const float &text_height,
                                       const std::string &text) {
  m_tracker_facade.sendCameraOverlayText(std::string(""), x, y, text_height,
                                         text);
}

/**
 * Called by CTracker when only the tracking rectangle should be removed from
 * the de_camera overlay (track lost while the session keeps running).
 */
void CTrackerMain::onCameraOverlayRemoveRect() {
  m_tracker_facade.sendCameraOverlayRemoveRect(std::string(""));
}

/**
 * Called by CTracker when the failure/status text should be removed from the
 * de_camera overlay (track recovered).
 */
void CTrackerMain::onCameraOverlayRemoveText() {
  m_tracker_facade.sendCameraOverlayRemoveText(std::string(""));
}

/**
 * Called by CTracker when the tracking rectangle should be removed from the
 * de_camera overlay (track lost, paused, stopped, or draw mode disabled).
 */
void CTrackerMain::onCameraOverlayRemove() {
  m_tracker_facade.sendCameraOverlayRemove(std::string(""));
}

/**
 * Called once trackig status changed.
 */
void CTrackerMain::onTrackStatusChanged(const int &status) {
  m_tracker_status = status;

  m_tracker_facade.sendTrackingTargetStatus(std::string(""), status);
  publishCapabilityState();

  // Handle tracker status changes with improved recovery logic
  if (status == TrackingTarget_STATUS_TRACKING_LOST) {
    onTrackerLost();
  } else if (status == TrackingTarget_STATUS_TRACKING_DETECTED) {
    onTrackerRecovered();
  }

  // Remove the problematic "Brake Mode" logic - tracker should continue operating
  // and wait for AI assistance or manual recovery instead of shutting down

#ifdef DDEBUG
  std::cout << _INFO_CONSOLE_BOLD_TEXT
            << "onTrackStatusChanged:" << _LOG_CONSOLE_BOLD_TEXT
            << std::to_string(m_tracker_status) << _NORMAL_CONSOLE_TEXT_
            << std::endl;
#endif
}

void CTrackerMain::onTrackerLost() {
  m_tracker_lost_timestamp = std::chrono::steady_clock::now();
  
#ifdef DDEBUG
  std::cout << _INFO_CONSOLE_BOLD_TEXT << "Tracker lost - waiting for recovery..." 
            << _NORMAL_CONSOLE_TEXT_ << std::endl;
#endif
}

void CTrackerMain::onTrackerRecovered() {
#ifdef DDEBUG
  std::cout << _INFO_CONSOLE_BOLD_TEXT << "Tracker recovered successfully!" 
            << _NORMAL_CONSOLE_TEXT_ << std::endl;
#endif
}

bool CTrackerMain::shouldContinueTracking() {
  // Always continue tracking unless explicitly stopped
  if (m_tracker_status == TrackingTarget_STATUS_TRACKING_STOPPED) {
    return false;
  }
  
  // If AI assisted recovery is disabled and tracker is lost, check timeout
  if (!m_ai_assisted_recovery_enabled && 
      m_tracker_status == TrackingTarget_STATUS_TRACKING_LOST) {
    
    auto now = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - m_tracker_lost_timestamp);
    
    if (elapsed.count() > m_tracker_lost_timeout_ms) {
      std::cout << _INFO_CONSOLE_BOLD_TEXT 
                << "Tracker lost timeout exceeded - manual intervention required" 
                << _NORMAL_CONSOLE_TEXT_ << std::endl;
                // WE SHOULD SEND MESSAGE TO GCS HERE
      return false;
    }
  }
  
  return true;
}

void CTrackerMain::lockTrk(const int64_t trk) {
  {
    std::lock_guard<std::mutex> lock(m_lock_mutex);
    m_lock.lockTrk(trk);
  }
  publishCapabilityState();
}

void CTrackerMain::lockClass(const std::string &cls) {
  {
    std::lock_guard<std::mutex> lock(m_lock_mutex);
    m_lock.lockClass(cls);
  }
  publishCapabilityState();
}

void CTrackerMain::unlock() {
  bool changed = false;
  {
    std::lock_guard<std::mutex> lock(m_lock_mutex);
    changed = m_lock.locked();
    m_lock.unlock();
  }
  if (changed) publishCapabilityState();
}

de::tracker::LockConfig CTrackerMain::currentLockConfig() const {
  de::tracker::LockConfig c;
  c.class_reacquire_frames = m_lock_class_reacquire_frames;
  c.reseed_cooldown_frames = m_lock_reseed_cooldown_frames;
  c.drift_correction = m_ai_priority;
  c.drift_distance = m_ai_priority_distance_threshold;
  return c;
}

/**
 * Observations are already confirmed and carry stable track ids, so nothing
 * is filtered here: the lock decides which one to hold and seeds the pixel
 * tracker from its box.
 */
void CTrackerMain::onObservations(const de::ai::ObservationFrame &frame) {
  if (m_tracker_status == TrackingTarget_STATUS_TRACKING_STOPPED)
    return;

  de::tracker::PixelTrackerState state;
  state.tracking = (m_tracker_status == TrackingTarget_STATUS_TRACKING_DETECTED);
  state.has_center = m_ema_init;
  state.cx = m_ema_x + 0.5;  // EMA is kept in [-0.5,0.5]
  state.cy = m_ema_y + 0.5;

  de::tracker::LockBox seed;
  bool reseed = false;
  std::string before, after;
  {
    std::lock_guard<std::mutex> lock(m_lock_mutex);
    m_lock.setConfig(currentLockConfig());
    before = m_lock.describe();
    reseed = m_lock.onObservations(frame, state, seed);
    after = m_lock.describe();
  }

  if (reseed) {
    std::cout << _INFO_CONSOLE_BOLD_TEXT << "lock (" << after << "):" << seed.x << ":" << seed.y << ":"
              << seed.w << ":" << seed.h << _NORMAL_CONSOLE_TEXT_ << std::endl;
    seedTracker(static_cast<float>(seed.x), static_cast<float>(seed.y),
                static_cast<float>(seed.w), static_cast<float>(seed.h));
  }

  if (before != after) publishCapabilityState();
}

// ============================================================================
// Phase 3: Capability registry (de.cap/1) - "visual_tracker" namespace
// ============================================================================

const char *CTrackerMain::trackingStateName(const int status) {
  switch (status) {
  case TrackingTarget_STATUS_TRACKING_LOST:
    return "lost";
  case TrackingTarget_STATUS_TRACKING_DETECTED:
    return "detected";
  case TrackingTarget_STATUS_TRACKING_ENABLED:
    return "enabled";
  case TrackingTarget_STATUS_TRACKING_STOPPED:
    return "stopped";
  default:
    return "config";
  }
}

const char *CTrackerMain::aiStateName(const int status) {
  switch (status) {
  case TrackingTarget_STATUS_AI_Recognition_DISABLED:
    return "disabled";
  case TrackingTarget_STATUS_AI_Recognition_ENABLED:
    return "enabled";
  case TrackingTarget_STATUS_AI_Recognition_LOST:
    return "lost";
  case TrackingTarget_STATUS_AI_Recognition_DETECTED:
    return "detected";
  default:
    return "disabled";
  }
}

void CTrackerMain::publishCapabilityState() {
  std::string lock_state;
  {
    std::lock_guard<std::mutex> lock(m_lock_mutex);
    lock_state = m_lock.describe();
  }
  m_tracker_facade.publishState(
      "visual_tracker",
      Json_de::object(
          {{"tracking", trackingStateName(m_tracker_status)},
           {"ai", aiStateName(m_ai_tracker_status)},
           {"lock", lock_state}}));
}

/**
 * de_tracker owns the whole "visual_tracker" namespace (one owner per unit,
 * README E4): "track" is served here, "start"/"stop" are forwarded to the
 * AI module (droneengage_yolo_ai) as AI_Recognition_ACTION (1076). The
 * advert is re-sent when the AI class list becomes known so "start {class}"
 * turns into an enum; the new hash in the ID message makes the registry
 * re-fetch it.
 */
void CTrackerMain::setupCapabilities() {
  std::vector<std::string> class_names;
  {
    std::lock_guard<std::mutex> lock(m_ai_class_lock);
    class_names = m_ai_class_names;
  }

  const Json_de advert = de::tracker::buildVisualTrackerAdvert(version_string, class_names);

  if (!m_tracker_facade.setCapabilities({advert.dump()})) {
    std::cout << _ERROR_CONSOLE_BOLD_TEXT_
              << "capability advert rejected for ns visual_tracker"
              << _NORMAL_CONSOLE_TEXT_ << std::endl;
    return;
  }

  if (!m_caps_advertised) {
    m_tracker_facade.onInvoke([this](const std::string &id, const std::string &ns,
                                     const std::string &act, const Json_de &params,
                                     std::string &err) -> Json_de {
      return onCapabilityInvoke(id, ns, act, params, err);
    });
    m_caps_advertised = true;
  }
}

/**
 * Class names from AI_Recognition_STATUS a=CLASS_LIST (1077). Re-advertises
 * only when the list actually changed.
 */
void CTrackerMain::setAIClassList(const std::vector<std::string> &class_names) {
  {
    std::lock_guard<std::mutex> lock(m_ai_class_lock);
    if (class_names == m_ai_class_names) return;
    m_ai_class_names = class_names;
  }
  setupCapabilities();
}

/**
 * Ask the AI module for its class list (it answers with 1077 a=CLASS_LIST).
 */
void CTrackerMain::requestAIClassList() {
  m_tracker_facade.sendAIRecognitionAction(
      Json_de::object({{"a", TrackingTarget_ACTION_AI_Recognition_CLASS_LIST}}));
}

/**
 * CAPABILITY_INVOKE handler (worker thread). Params arrive already validated
 * against the advert.
 * - start {class}: AI_Recognition_ACTION ENABLE + SEARCH i=[index] (1076).
 * - stop {}: AI_Recognition_ACTION DISABLE (1076).
 * - track {target:{x,y,w,h}}: TrackingTarget_ACTION_TRACKING_REGION.
 */
Json_de CTrackerMain::onCapabilityInvoke(const std::string &id,
                                         const std::string &ns,
                                         const std::string &act,
                                         const Json_de &params,
                                         std::string &err) {
  (void)id;
  (void)ns;

  if (act == "start") {
    const std::string cls = params.value("class", "");

    // resolve the class name to its index in the AI module's list
    int index = -1;
    bool list_known = false;
    {
      std::lock_guard<std::mutex> lock(m_ai_class_lock);
      list_known = !m_ai_class_names.empty();
      for (size_t i = 0; i < m_ai_class_names.size(); ++i) {
        if (m_ai_class_names[i] == cls) {
          index = (int)i;
          break;
        }
      }
    }
    if (!list_known) {
      requestAIClassList();
      err = "AI class list not known yet (is droneengage_yolo_ai running?)";
      return Json_de::object();
    }
    if (index < 0) {
      err = "unknown class '" + cls + "'";
      return Json_de::object();
    }

    // same sequence the GCS sends: ENABLE then SEARCH
    m_tracker_facade.sendAIRecognitionAction(
        Json_de::object({{"a", TrackingTarget_ACTION_AI_Recognition_ENABLE}}));
    m_tracker_facade.sendAIRecognitionAction(Json_de::object(
        {{"a", TrackingTarget_ACTION_AI_Recognition_SEARCH},
         {"i", Json_de::array({index})}}));
    // the AI module re-broadcasts its class list so de_mavlink's detect
    // projector can map the selected index to a name
    requestAIClassList();
    return Json_de::object({{"started", true}, {"class", cls}, {"index", index}});
  }

  if (act == "stop") {
    m_tracker_facade.sendAIRecognitionAction(
        Json_de::object({{"a", TrackingTarget_ACTION_AI_Recognition_DISABLE}}));
    return Json_de::object({{"stopped", true}});
  }

  if (act == "lock") {
    const bool has_trk = params.contains("trk") && params["trk"].is_number_integer();
    const bool has_cls = params.contains("class") && params["class"].is_string();
    if (has_trk == has_cls) {
      err = "lock needs exactly one of trk or class";
      return Json_de::object();
    }

    // locking is the same request as enabling the tracker
    if (m_tracker_status == TrackingTarget_STATUS_TRACKING_STOPPED) {
      m_tracker_status = TrackingTarget_STATUS_TRACKING_ENABLED;
      m_tracker_facade.sendTrackingTargetStatus(std::string(""), m_tracker_status);
    }

    if (has_trk) lockTrk(params["trk"].get<int64_t>());
    else lockClass(params["class"].get<std::string>());

    return Json_de::object({{"lock", [this]() { std::lock_guard<std::mutex> l(m_lock_mutex); return m_lock.describe(); }()}});
  }

  if (act == "unlock") {
    unlock();
    return Json_de::object({{"lock", "none"}});
  }

  if (act != "track") {
    err = "unknown action visual_tracker." + act;
    return Json_de::object();
  }

  const Json_de &target = params["target"];
  const float x = target["x"].get<float>();
  const float y = target["y"].get<float>();
  const float w = target["w"].get<float>();
  const float h = target["h"].get<float>();

  // startTrackingRect() is a no-op while the tracker is stopped
  if (m_tracker_status == TrackingTarget_STATUS_TRACKING_STOPPED)
    enableTracking();

  startTrackingRect(x, y, w, h, getAiPriority());

  return Json_de::object({{"tracking", trackingStateName(m_tracker_status)}});
}
