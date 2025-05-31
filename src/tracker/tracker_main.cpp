#include <stdio.h>
#include <iostream>

#include "../helpers/colors.hpp"
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
    
    bool res = m_tracker.get()->init(m_jsonConfig["tracker_algorithm_index"], m_jsonConfig["source_video_device"], output_video_device);
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
void CTrackerMain::onTrack (const float& x, const float& y, const float& width, const float& height) 
{

    #ifdef DEBUG
        std::cout << _INFO_CONSOLE_BOLD_TEXT << "onTrack >> " 
        << _LOG_CONSOLE_BOLD_TEXT << " x:" << _INFO_CONSOLE_BOLD_TEXT << x
        << _LOG_CONSOLE_BOLD_TEXT << "  y:" << _INFO_CONSOLE_BOLD_TEXT << y
        << _LOG_CONSOLE_BOLD_TEXT << "  w:" << _INFO_CONSOLE_BOLD_TEXT << width
        << _LOG_CONSOLE_BOLD_TEXT << "  h:" << _INFO_CONSOLE_BOLD_TEXT << height
        << _NORMAL_CONSOLE_TEXT_ << std::endl;
    #endif
   static int counter=0;
    Json_de targets = Json_de::array();
    targets.push_back({
        {"a",x},
        {"b",y},
        {"c",width},
        {"d",height},
        {"n","target"}
    });

    m_trackerFacade.sendTrackingTargetsLocation(
        std::string(""),
        targets
    );
}

/**
 * Called once trackig status changed.
 */
void CTrackerMain::onTrackStatusChanged (const bool& track)  
{
    std::cout << _INFO_CONSOLE_BOLD_TEXT << "onTrackStatusChanged:" << _LOG_CONSOLE_BOLD_TEXT << std::to_string(track) << _NORMAL_CONSOLE_TEXT_ << std::endl;
}
