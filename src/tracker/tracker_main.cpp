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
#include "video.hpp"

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
      m_desired_input_width, m_desired_input_height, m_crosshair);
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
                  << _INFO_CONSOLE_BOLD_TEXT << m_source_video_device
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
                  << _INFO_CONSOLE_BOLD_TEXT << m_output_video_device
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

    // Load AI enhancement parameters
    if (advanced_tracking.contains("ai_confidence_threshold")) {
      MIN_AI_CONFIDENCE_THRESHOLD = advanced_tracking["ai_confidence_threshold"].get<float>();
    }
    
    if (advanced_tracking.contains("ai_stability_threshold")) {
      MIN_DETECTION_STABILITY_THRESHOLD = advanced_tracking["ai_stability_threshold"].get<float>();
    }
    
    if (advanced_tracking.contains("ai_detection_buffer_size")) {
      AI_DETECTION_BUFFER_SIZE = advanced_tracking["ai_detection_buffer_size"].get<int>();
    }
    
    if (advanced_tracking.contains("ai_detection_timeout_ms")) {
      DETECTION_TIMEOUT_MS = advanced_tracking["ai_detection_timeout_ms"].get<int>();
    }
    
    // Load tracker recovery parameters
    if (advanced_tracking.contains("ai_assisted_recovery_enabled")) {
      m_ai_assisted_recovery_enabled = advanced_tracking["ai_assisted_recovery_enabled"].get<bool>();
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
            << ", ai_confidence_threshold: " << _LOG_CONSOLE_BOLD_TEXT
            << MIN_AI_CONFIDENCE_THRESHOLD << _INFO_CONSOLE_TEXT
            << ", ai_stability_threshold: " << _LOG_CONSOLE_BOLD_TEXT
            << MIN_DETECTION_STABILITY_THRESHOLD << _INFO_CONSOLE_TEXT
            << ", ai_assisted_recovery_enabled: " << _LOG_CONSOLE_BOLD_TEXT
            << (m_ai_assisted_recovery_enabled ? "true" : "false") << _INFO_CONSOLE_TEXT
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

  m_scheduler_thread.join();

  m_tracker.get()->uninit();

  return true;
}

void CTrackerMain::startTrackingRect(const float x, const float y,
                                     const float w, const float h) {
#ifdef DEBUG
  std::cout << _INFO_CONSOLE_BOLD_TEXT << "rect:" << x << ":" << y << ":" << w
            << ":" << h << std::endl;
#endif

  if (m_tracker_status == TrackingTarget_STATUS_TRACKING_STOPPED)
    return;

  // Reset AI detection waiting state when manual tracking starts
  m_waiting_for_first_ai_detection = false;

  m_tracker.get()->stop();

  m_tracker.get()->trackRect(x, y, w, h);

  m_tracker_facade.sendTrackingTargetStatus(std::string(""), m_tracker_status);
}

void CTrackerMain::enableTracking() {
  // this state means I will accept start tracking point.
  // this is not a real start for the tracker core.
  m_tracker_status = TrackingTarget_STATUS_TRACKING_ENABLED;
  
  // Clear AI detection buffer when enabling tracking to start fresh
  m_ai_detection_buffer.clear();
  m_waiting_for_first_ai_detection = true;

  // ACK
  m_tracker_facade.sendTrackingTargetStatus(std::string(""), m_tracker_status);
}

void CTrackerMain::pauseTracking() { m_tracker.get()->pause(); }

void CTrackerMain::stopTracking() {
  m_tracker_status = TrackingTarget_STATUS_TRACKING_STOPPED;
  m_tracker.get()->stop();
  
  // Reset AI detection waiting state
  m_waiting_for_first_ai_detection = false;
  m_ai_detection_buffer.clear();

  m_tracker_facade.sendTrackingTargetStatus(std::string(""), m_tracker_status);
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
    m_tracker_facade.sendTrackingTargetsLocation(std::string(""), targets);
  }
}

/**
 * Called once trackig status changed.
 */
void CTrackerMain::onTrackStatusChanged(const int &status) {
  m_tracker_status = status;

  m_tracker_facade.sendTrackingTargetStatus(std::string(""), status);

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
      return false;
    }
  }
  
  return true;
}

void CTrackerMain::onAITrackerBestRect(const float x, const float y,
                                       const float w, const float h) {
  // Legacy method - assume high confidence for backward compatibility
  onAITrackerBestRectWithConfidence(x, y, w, h, 1.0f);
}

void CTrackerMain::onAITrackerBestRectWithConfidence(const float x, const float y,
                                                     const float w, const float h, const float confidence) {
  m_ai_tracker_status = TrackingTarget_STATUS_AI_Recognition_DETECTED;

#ifdef DEBUG
  std::cout << "onAITrackerBestRectWithConfidence:" << m_tracker_status 
            << " confidence:" << confidence << std::endl;
#endif

  // Apply confidence threshold filtering
  if (confidence < MIN_AI_CONFIDENCE_THRESHOLD) {
#ifdef DDEBUG
    std::cout << _INFO_CONSOLE_BOLD_TEXT << "AI detection filtered out due to low confidence: " 
              << _LOG_CONSOLE_BOLD_TEXT << confidence << _NORMAL_CONSOLE_TEXT_ << std::endl;
#endif
    return;
  }

  // If we're waiting for the first AI detection after enableTracking(), start immediately
  if (m_waiting_for_first_ai_detection && (m_tracker_status == TrackingTarget_STATUS_TRACKING_ENABLED)) {
    std::cout << _INFO_CONSOLE_BOLD_TEXT << "ai_rect (first detection):" << x << ":" << y << ":"
              << w << ":" << h << " conf:" << confidence << std::endl;
    startTrackingRect(x, y, w, h);
    m_waiting_for_first_ai_detection = false;
    return;
  }

  // Create detection record
  AIDetection detection(x, y, w, h, confidence);
  
  // Check if we should reinitialize tracker based on temporal consistency
  if (shouldReinitializeTracker(detection)) {
    if ((m_tracker_status == TrackingTarget_STATUS_TRACKING_LOST) ||
        (m_tracker_status == TrackingTarget_STATUS_TRACKING_ENABLED)) {
      std::cout << _INFO_CONSOLE_BOLD_TEXT << "ai_rect (stable):" << x << ":" << y << ":"
                << w << ":" << h << " conf:" << confidence << std::endl;
      startTrackingRect(x, y, w, h);
    }
  } else {
#ifdef DDEBUG
    std::cout << _INFO_CONSOLE_BOLD_TEXT << "AI detection rejected due to temporal instability" 
              << _NORMAL_CONSOLE_TEXT_ << std::endl;
#endif
  }
}

bool CTrackerMain::shouldReinitializeTracker(const AIDetection& detection) {
  // Add new detection to buffer
  m_ai_detection_buffer.push_back(detection);
  
  // Remove old detections beyond buffer size
  while (m_ai_detection_buffer.size() > AI_DETECTION_BUFFER_SIZE) {
    m_ai_detection_buffer.erase(m_ai_detection_buffer.begin());
  }
  
  // Remove old detections beyond timeout
  auto now = std::chrono::steady_clock::now();
  for (auto it = m_ai_detection_buffer.begin(); it != m_ai_detection_buffer.end(); ) {
    auto age = std::chrono::duration_cast<std::chrono::milliseconds>(now - it->timestamp);
    if (age.count() > DETECTION_TIMEOUT_MS) {
      it = m_ai_detection_buffer.erase(it);
    } else {
      ++it;
    }
  }
  
  // Check if we have enough detections for stability assessment
  if (m_ai_detection_buffer.size() < 3) {
    return false;  // Need at least 3 detections for stability
  }
  
  // Calculate detection stability
  float stability = calculateDetectionStability();
  
#ifdef DDEBUG
  std::cout << _INFO_CONSOLE_BOLD_TEXT << "Detection stability: " 
            << _LOG_CONSOLE_BOLD_TEXT << stability << _NORMAL_CONSOLE_TEXT_ << std::endl;
#endif
  
  return stability >= MIN_DETECTION_STABILITY_THRESHOLD;
}

bool CTrackerMain::hasConsistentDetections() {
  if (m_ai_detection_buffer.size() < 3) {
    return false;
  }
  
  // Check if all recent detections meet minimum confidence
  for (const auto& det : m_ai_detection_buffer) {
    if (det.confidence < MIN_AI_CONFIDENCE_THRESHOLD) {
      return false;
    }
  }
  
  // Check positional consistency (detections should be relatively close)
  const AIDetection& latest = m_ai_detection_buffer.back();
  for (size_t i = 0; i < m_ai_detection_buffer.size() - 1; ++i) {
    const AIDetection& prev = m_ai_detection_buffer[i];
    
    // Calculate center points
    float latest_center_x = latest.x + latest.w / 2.0f;
    float latest_center_y = latest.y + latest.h / 2.0f;
    float prev_center_x = prev.x + prev.w / 2.0f;
    float prev_center_y = prev.y + prev.h / 2.0f;
    
    // Calculate distance between centers
    float distance = std::sqrt(
      std::pow(latest_center_x - prev_center_x, 2) + 
      std::pow(latest_center_y - prev_center_y, 2)
    );
    
    // If distance is too large, detections are inconsistent
    if (distance > 0.2f) {  // 20% of image dimension
      return false;
    }
  }
  
  return true;
}

float CTrackerMain::calculateDetectionStability() {
  if (m_ai_detection_buffer.size() < 2) {
    return 0.0f;
  }
  
  float stability_score = 0.0f;
  const AIDetection& latest = m_ai_detection_buffer.back();
  
  // Factor 1: Confidence consistency (40% weight)
  float avg_confidence = 0.0f;
  float confidence_variance = 0.0f;
  
  for (const auto& det : m_ai_detection_buffer) {
    avg_confidence += det.confidence;
  }
  avg_confidence /= m_ai_detection_buffer.size();
  
  for (const auto& det : m_ai_detection_buffer) {
    confidence_variance += std::pow(det.confidence - avg_confidence, 2);
  }
  confidence_variance /= m_ai_detection_buffer.size();
  
  float confidence_stability = 1.0f - std::min(confidence_variance, 1.0f);
  stability_score += 0.4f * confidence_stability;
  
  // Factor 2: Positional consistency (40% weight)
  float positional_variance = 0.0f;
  float latest_center_x = latest.x + latest.w / 2.0f;
  float latest_center_y = latest.y + latest.h / 2.0f;
  
  for (size_t i = 0; i < m_ai_detection_buffer.size() - 1; ++i) {
    const AIDetection& prev = m_ai_detection_buffer[i];
    float prev_center_x = prev.x + prev.w / 2.0f;
    float prev_center_y = prev.y + prev.h / 2.0f;
    
    float distance = std::sqrt(
      std::pow(latest_center_x - prev_center_x, 2) + 
      std::pow(latest_center_y - prev_center_y, 2)
    );
    positional_variance += distance;
  }
  
  positional_variance /= (m_ai_detection_buffer.size() - 1);
  float positional_stability = std::max(0.0f, 1.0f - positional_variance * 5.0f);  // Scale factor
  stability_score += 0.4f * positional_stability;
  
  // Factor 3: Temporal consistency (20% weight)
  auto now = std::chrono::steady_clock::now();
  auto age_ms = std::chrono::duration_cast<std::chrono::milliseconds>(now - latest.timestamp);
  float temporal_factor = 1.0f - std::min(age_ms.count() / 1000.0f, 1.0f);  // Decay over 1 second
  stability_score += 0.2f * temporal_factor;
  
  return std::min(stability_score, 1.0f);
}
