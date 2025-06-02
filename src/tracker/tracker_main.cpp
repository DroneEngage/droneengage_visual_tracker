#include <stdio.h>
#include <iostream>

#include "../helpers/colors.hpp"
#include "../helpers/helpers.hpp"
#include "../de_common/configFile.hpp"
#include "../de_common/messages.hpp"
#include "tracker.hpp"
#include "tracker_main.hpp"




using namespace de::tracker;



bool CTrackerMain::init()
{
    Json_de m_jsonConfig = CConfigFile::getInstance().GetConfigJSON();
    
    if (!m_jsonConfig.contains("tracker_algorithm_index"))
    {
        std::cout << _ERROR_CONSOLE_BOLD_TEXT_ << "FATAL ERROR: " << _INFO_CONSOLE_TEXT << CConfigFile::getInstance().getFileName() << " does not have field " << _ERROR_CONSOLE_TEXT_ "[tracker_algorithm_index]" <<  _NORMAL_CONSOLE_TEXT_ << std::endl;
	
        exit(1);
    }

    if (!m_jsonConfig.contains("source_video_device"))
    {
        std::cout << _ERROR_CONSOLE_BOLD_TEXT_ << "FATAL ERROR: " << _INFO_CONSOLE_TEXT << CConfigFile::getInstance().getFileName() << " does not have field " << _ERROR_CONSOLE_TEXT_ "[source_video_device]" <<  _NORMAL_CONSOLE_TEXT_ << std::endl;
	
        exit(1);
    }

    m_tracker = std::make_unique<CTracker>(CTracker(this));
    std::string output_video_device = "";
    if (m_jsonConfig.contains("output_video_device"))
    {
        output_video_device = m_jsonConfig["output_video_device"];
    }
    
    uint16_t camera_orientation = m_jsonConfig["camera_orientation"].get<uint16_t>();
    bool camera_forward = m_jsonConfig["camera_forward"].get<bool>();

    bool res = m_tracker.get()->init(m_jsonConfig["tracker_algorithm_index"], m_jsonConfig["source_video_device"], camera_orientation, camera_forward, output_video_device);
    if (res == false)
    {
        std::cout << _ERROR_CONSOLE_BOLD_TEXT_ << "FATAL ERROR:" << _INFO_CONSOLE_TEXT << " Failed to initialize tracker. " <<  _NORMAL_CONSOLE_TEXT_ << std::endl;
        exit(1);
	}


    if (output_video_device!="")
    {
        // if Target video virtual device is specified then stream
        // to a virtual video driver even without tracking
        bool display_video = false;
        const Json_de& jsonConfig = de::CConfigFile::getInstance().GetConfigJSON();
        if (jsonConfig.contains("display_video"))
        {
            display_video = jsonConfig["display_video"].get<bool>();
        }
        // x = -1 start streaming with no tracking
        m_tracker.get()->track(-1,0, 0, display_video);
    }
    return true;
}



bool CTrackerMain::uninit()
{
    m_tracker.get()->uninit();

    return true;
}


void CTrackerMain::startTracking(const float x, const float y, const float radius)
{
    m_tracker.get()->stop();

    bool display_video = false;
    const Json_de& jsonConfig = de::CConfigFile::getInstance().GetConfigJSON();
    if (jsonConfig.contains("display_video"))
    {
        display_video = jsonConfig["display_video"].get<bool>();
    }

    m_tracker.get()->track(x,y,radius, display_video);
}

void CTrackerMain::stopTracking()
{
    m_tracker.get()->stop();
}

/**
 * Called when there is a a tracked object.
 * output from 0 to 1.0
 * (0,0) top left
 * center = [(x + w )/2 , (y + h)/2]
 */
void CTrackerMain::onTrack (const float& x, const float& y, const float& width, const float& height, const uint16_t camera_orientation, const bool camera_forward) 
{

   const double center_x = 0.5 -  x + width /2.0f; 
   const double center_y = 0.5 - y + height /2.0f; 
      
   double delta_x,delta_y,delta_z;

    switch (camera_orientation)
    {
    case DEF_TRACK_ORIENTATION_DEG_0:
            delta_x =  center_x;
            delta_y =  center_y;
        break;
    case DEF_TRACK_ORIENTATION_DEG_90:
            delta_x = -center_y;
            delta_y =  center_x;
        break;
    
    case DEF_TRACK_ORIENTATION_DEG_180:
            delta_x = -center_x;
            delta_y = -center_y;
        break;

    case DEF_TRACK_ORIENTATION_DEG_270:
            delta_x =  center_y;
            delta_y = -center_x;
        break;
    
    default:
        break;
    }


    // Apply precision limiting
    delta_x = roundToPrecision(delta_x, 6);
    delta_y = roundToPrecision(delta_y, 6);


    Json_de targets = Json_de::array();
    
    if (camera_forward)
    {
        targets.push_back({
            {"x",delta_x},
            {"y",delta_y}
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

    #ifdef DEBUG
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
void CTrackerMain::onTrackStatusChanged (const bool& track)  
{
    m_trackerFacade.sendTrackingTargetStatus (
        std::string(""),
        track
    );
    
    std::cout << _INFO_CONSOLE_BOLD_TEXT << "onTrackStatusChanged:" << _LOG_CONSOLE_BOLD_TEXT << std::to_string(track) << _NORMAL_CONSOLE_TEXT_ << std::endl;
}
