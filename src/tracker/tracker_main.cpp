#include <stdio.h>
#include <iostream>

#include "../de_common/helpers/colors.hpp"
#include "../de_common/helpers/helpers.hpp"
#include "../de_common/de_databus/configFile.hpp"
#include "../de_common/de_databus/messages.hpp"
#include "tracker.hpp"
#include "tracker_main.hpp"
#include "video.hpp"



using namespace de::tracker;



bool CTrackerMain::init()
{

    uint16_t camera_orientation = 0;
    bool camera_forward = true;
    de::tracker::ENUM_TRACKER_TYPE tracker_algorithm_index = de::tracker::ENUM_TRACKER_TYPE::TRACKER_CSRT;

    Json_de m_jsonConfig = CConfigFile::getInstance().GetConfigJSON();
    if (!m_jsonConfig.contains("tracking"))
    {
        std::cout << _ERROR_CONSOLE_BOLD_TEXT_ << "FATAL ERROR: " << _INFO_CONSOLE_TEXT << CConfigFile::getInstance().getFileName() 
                << " does not have field " << _ERROR_CONSOLE_TEXT_ << "[tracking]" <<  _NORMAL_CONSOLE_TEXT_ 
                << std::endl;
	
        return false;
    }
    else
    {

        Json_de tracking = m_jsonConfig["tracking"];
        
        camera_orientation = tracking["camera_orientation"].get<uint16_t>();
        camera_forward = tracking["camera_forward"].get<bool>();

        if (tracking.contains("tracker_algorithm_index"))
        {
            tracker_algorithm_index = tracking["tracker_algorithm_index"].get<de::tracker::ENUM_TRACKER_TYPE>();
        }
    }


    

    

    std::string source_video_device = "";
    std::string output_video_device = "";
    int desired_input_width = 0;
    int desired_input_height = 0;

    if (m_jsonConfig.contains("camera"))
    {
        Json_de camera = m_jsonConfig["camera"];

        if (camera.contains("source_video_device_name"))
            {
                const int video_index = CVideo::findVideoDeviceIndex(camera["source_video_device_name"].get<std::string>());
                if (video_index != -1) 
                {
                    source_video_device = "/dev/video" + std::to_string(video_index);

                    std::cout << _LOG_CONSOLE_BOLD_TEXT << "Using source_video_device_name:" << _INFO_CONSOLE_BOLD_TEXT << source_video_device 
                            << _NORMAL_CONSOLE_TEXT_
                            << std::endl;
                }

          
            }

            if (source_video_device.empty())
            {
                if (!camera.contains("source_video_device"))
                {
                    std::cout << _ERROR_CONSOLE_BOLD_TEXT_ << "FATAL ERROR: " << _INFO_CONSOLE_TEXT << CConfigFile::getInstance().getFileName() 
                            << " does not have field " << _ERROR_CONSOLE_TEXT_ << "[source_video_device]" <<  _NORMAL_CONSOLE_TEXT_ 
                            << std::endl;
                
                    return false;
                }
                else
                {
                    source_video_device = camera["source_video_device"].get<std::string>();

                    std::cout << _LOG_CONSOLE_BOLD_TEXT << "Using source_video_device:" << _INFO_CONSOLE_BOLD_TEXT << source_video_device 
                            << _NORMAL_CONSOLE_TEXT_
                            << std::endl;
                }
            }

            
            if (camera.contains("output_video_device_name"))
            {
                const int video_index = CVideo::findVideoDeviceIndex(camera["output_video_device_name"].get<std::string>());
                if (video_index != -1) 
                {
                    output_video_device = "/dev/video" + std::to_string(video_index);

                    std::cout << _SUCCESS_CONSOLE_BOLD_TEXT_ << "Using output_video_device_name:" << _INFO_CONSOLE_BOLD_TEXT << output_video_device 
                            << _NORMAL_CONSOLE_TEXT_
                            << std::endl;

                }
            }

            
            if (output_video_device.empty())
            {
                if (!camera.contains("output_video_device"))
                {
                    std::cout << _ERROR_CONSOLE_BOLD_TEXT_ << "FATAL ERROR:" << _INFO_CONSOLE_TEXT << " No output_video_device specified in config.json" <<  _NORMAL_CONSOLE_TEXT_ << std::endl;
                    return false;
                }
                else
                {
                    output_video_device = camera["output_video_device"].get<std::string>();

                    std::cout << _SUCCESS_CONSOLE_BOLD_TEXT_ << "Using output_video_device:" << _INFO_CONSOLE_BOLD_TEXT << output_video_device 
                            <<   _NORMAL_CONSOLE_TEXT_
                            << std::endl;
                }
            }


            // Optional desired input resolution
                if (camera.contains("desired_input_width") && camera.contains("desired_input_height"))
                {
                    desired_input_width = camera["desired_input_width"].get<int>();
                    desired_input_height = camera["desired_input_height"].get<int>();
                    if (desired_input_width < 0 || desired_input_height < 0)
                    {
                        desired_input_width = 0;
                        desired_input_height = 0;
                    }
                }


                // If desired size not provided, display max available video size
                if (desired_input_width == 0 && desired_input_height == 0 && !source_video_device.empty())
                {
                    unsigned int max_w = 0, max_h = 0;
                    if (CVideo::getMaxSupportedResolution(source_video_device, max_w, max_h))
                    {
                        std::cout << _SUCCESS_CONSOLE_BOLD_TEXT_ << "Camera max supported resolution: "
                                << _INFO_CONSOLE_TEXT << max_w << "x" << max_h << _NORMAL_CONSOLE_TEXT_ << std::endl;
                    }
                }
    }
    else
    {
        std::cout << _ERROR_CONSOLE_BOLD_TEXT_ << "FATAL ERROR:" << _INFO_CONSOLE_TEXT << " No camera specified in config.json" <<  _NORMAL_CONSOLE_TEXT_ << std::endl;
        return false; 
    }
    
    uint16_t frames_to_skip_between_messages = FRAMES_TO_SKIP_BETWEEN_MESSAGES;
    uint16_t frame_to_skip_between_track_process = FRAMES_TO_SKIP_BETWEEN_TRACK_PROCESS;
    

    if (!m_jsonConfig.contains("advanced_tracking"))
    {
        std::cout << _INFO_CONSOLE_BOLD_TEXT << "Field not found in config.json: " << _INFO_CONSOLE_TEXT << "[advanced_tracking]" << _NORMAL_CONSOLE_TEXT_ << std::endl;
        std::cout << _SUCCESS_CONSOLE_BOLD_TEXT_ << "Default Values will be used  " << _NORMAL_CONSOLE_TEXT_ << std::endl;

    }
    else
    {
        Json_de advanced_tracking = m_jsonConfig["advanced_tracking"];
        if (advanced_tracking.contains("frames_to_skip_between_messages"))
        {
            frames_to_skip_between_messages = advanced_tracking["frames_to_skip_between_messages"].get<uint16_t>();
        }

        if (advanced_tracking.contains("frame_to_skip_between_track_process"))
        {
            frame_to_skip_between_track_process = advanced_tracking["frame_to_skip_between_track_process"].get<uint16_t>();
        }

        std::cout << _SUCCESS_CONSOLE_BOLD_TEXT_ << "Field Found: advanced_tracking field found:  " << _INFO_CONSOLE_TEXT << "Following values will be used:" << _NORMAL_CONSOLE_TEXT_ << std::endl;
    }

    std::cout << _INFO_CONSOLE_TEXT << "frames_to_skip_between_messages: " << _LOG_CONSOLE_BOLD_TEXT << frames_to_skip_between_messages 
            << _INFO_CONSOLE_TEXT << ", frame_to_skip_between_track_process: " << _LOG_CONSOLE_BOLD_TEXT << frame_to_skip_between_track_process 
            << _NORMAL_CONSOLE_TEXT_ << std::endl;

    
    
    

    m_tracker = std::make_unique<CTracker>(this);
    

    

    bool res = m_tracker.get()->init(tracker_algorithm_index, source_video_device
        , camera_orientation, camera_forward, output_video_device
        , frames_to_skip_between_messages, frame_to_skip_between_track_process
        , desired_input_width, desired_input_height) ;
    if (res == false)
    {
        std::cout << _ERROR_CONSOLE_BOLD_TEXT_ << "FATAL ERROR:" << _INFO_CONSOLE_TEXT << " Failed to initialize tracker. " <<  _NORMAL_CONSOLE_TEXT_ << std::endl;
        return false;
	}


    
    m_tracker.get()->track(-1,0, 0);
    
    return true;
}



bool CTrackerMain::uninit()
{
    m_tracker.get()->uninit();

    return true;
}

void CTrackerMain::startTrackingRect(const float x, const float y, const float w, const float h)
{
    #ifdef DEBUG
    std::cout << _INFO_CONSOLE_BOLD_TEXT << "rect:" << x << ":" << y << ":" << w << ":" << h << std::endl;
    #endif
        
    if (m_tracker_status == TrackingTarget_STATUS_TRACKING_STOPPED) return ;
    
    m_tracker.get()->stop();

    m_tracker.get()->trackRect(x,y,w,h);

    m_trackerFacade.sendTrackingTargetStatus (
        std::string(""),
        m_tracker_status
    );
}


void CTrackerMain::enableTracking()
{
    // this state means I will accept start tracking point.
    // this is not a real start for the tracker core.
    m_tracker_status = TrackingTarget_STATUS_TRACKING_ENABLED;
    
    // ACK
    m_trackerFacade.sendTrackingTargetStatus (
        std::string(""),
        m_tracker_status
    );
}

void CTrackerMain::pauseTracking()
{
    m_tracker.get()->pause();
}


void CTrackerMain::stopTracking()
{
    m_tracker_status = TrackingTarget_STATUS_TRACKING_STOPPED;
    m_tracker.get()->stop();

    m_trackerFacade.sendTrackingTargetStatus (
        std::string(""),
        m_tracker_status
    );
}



/**
 * Called when there is a a tracked object.
 * input x,y,w,h:[0 to 1.0]
 * output from [-0.5 to 0.5]
 * (0,0) top left
 * center = [(x + w )/2 , (y + h)/2]
 */
void CTrackerMain::onTrack (const float& x, const float& y, const float& width, const float& height, const uint16_t camera_orientation, const bool camera_forward) 
{

   const double center_x = -0.5 +  x + width /2.0f; 
   const double center_y = -0.5 + y + height /2.0f; 
      
   double delta_x,delta_y,delta_z;

    switch (camera_orientation)
    {
    case DEF_TRACK_ORIENTATION_DEG_0:
            delta_x =  center_x;
            delta_y =  center_y;
        break;
    case DEF_TRACK_ORIENTATION_DEG_90:
            delta_x =   center_y;
            delta_y =  -center_x;
        break;
    
    case DEF_TRACK_ORIENTATION_DEG_180:
            delta_x = -center_x;
            delta_y = -center_y;
        break;

    case DEF_TRACK_ORIENTATION_DEG_270:
            delta_x =  -center_y;
            delta_y =  -center_x;
        break;
    
    default:
        std::cerr << _ERROR_CONSOLE_BOLD_TEXT_ << "Invalid camera orientation: " 
                  << camera_orientation << _NORMAL_CONSOLE_TEXT_ << std::endl;
        break;
    }

#ifdef DEBUG
    std::cout << "Track Object:" << center_x << ":" << center_y << std::endl;
#endif

    // Apply precision limiting
    delta_x = roundToPrecision(delta_x, 3);
    delta_y = roundToPrecision(delta_y, 3);


    Json_de targets = Json_de::array();
    
    if (camera_forward)
    {
        targets.push_back({
            {"x",delta_x},
            {"y",-delta_y}
        });
    }
    else
    {
        delta_z = delta_y;

        targets.push_back({
            {"x",delta_x},
            {"z",delta_z}
        });
    }

    #ifdef DDEBUG
        std::cout << _INFO_CONSOLE_BOLD_TEXT << "onTrack >> " 
        << _LOG_CONSOLE_BOLD_TEXT << "  x:" << _INFO_CONSOLE_BOLD_TEXT << x
        << _LOG_CONSOLE_BOLD_TEXT << "  y:" << _INFO_CONSOLE_BOLD_TEXT << y
        << _LOG_CONSOLE_BOLD_TEXT << "  w:" << _INFO_CONSOLE_BOLD_TEXT << width
        << _LOG_CONSOLE_BOLD_TEXT << "  h:" << _INFO_CONSOLE_BOLD_TEXT << height
        << _LOG_CONSOLE_BOLD_TEXT << "  dx:" << _INFO_CONSOLE_BOLD_TEXT << delta_x
        << _LOG_CONSOLE_BOLD_TEXT << "  dy:" << _INFO_CONSOLE_BOLD_TEXT << delta_y
        << _LOG_CONSOLE_BOLD_TEXT << "  dz:" << _INFO_CONSOLE_BOLD_TEXT << delta_z
        << _NORMAL_CONSOLE_TEXT_ << std::endl;
    #endif

    
    m_trackerFacade.sendTrackingTargetsLocation (
        std::string(""),
        targets
    );
}

/**
 * Called once trackig status changed.
 */
void CTrackerMain::onTrackStatusChanged (const int& status)  
{
    m_tracker_status = status;
    
    m_trackerFacade.sendTrackingTargetStatus (
        std::string(""),
        status
    );
    
    if (((m_tracker_status == TrackingTarget_STATUS_TRACKING_LOST)
        || (m_tracker_status == TrackingTarget_STATUS_TRACKING_ENABLED))
    && (m_ai_tracker_status != TrackingTarget_ACTION_AI_Recognition_ENABLE)
        )
        {
            // NO AI to Help
            // Brake Mode
            std::cout << _ERROR_CONSOLE_BOLD_TEXT_ << "CANNOT CONTINUE TRACKING..." << _NORMAL_CONSOLE_TEXT_ << std::endl;
        }

    #ifdef DDEBUG
    std::cout << _INFO_CONSOLE_BOLD_TEXT << "onTrackStatusChanged:" << _LOG_CONSOLE_BOLD_TEXT << std::to_string(m_tracker_status) << _NORMAL_CONSOLE_TEXT_ << std::endl;
    #endif
}

void CTrackerMain::onAITrackerBestRect(const float x, const float y, const float w, const float h)
{
    m_ai_tracker_status = TrackingTarget_STATUS_AI_Recognition_DETECTED;

#ifdef DEBUG
    std::cout << "onAITrackerBestRect:" << m_tracker_status << std::endl;
#endif
    if ((m_tracker_status == TrackingTarget_STATUS_TRACKING_LOST)
    || (m_tracker_status == TrackingTarget_STATUS_TRACKING_ENABLED))
    {
        std::cout << _INFO_CONSOLE_BOLD_TEXT << "ai_rect:" << x << ":" << y << ":" << w << ":" << h << std::endl;
        startTrackingRect(x, y, w, h);
    }
}
            
