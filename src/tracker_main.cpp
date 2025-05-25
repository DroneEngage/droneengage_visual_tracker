#include <stdio.h>
#include <iostream>

#include "./helpers/colors.hpp"
#include "./uavos_common/configFile.hpp"
#include "./uavos_common/messages.hpp"
#include "tracker.hpp"
#include "tracker_main.hpp"





using namespace uavos::tracker;



bool CTrackerMain::init()
{
    Json m_jsonConfig = CConfigFile::getInstance().GetConfigJSON();
    
    if (!m_jsonConfig.contains("tracker_algorithm_index"))
    {
        std::cout << _ERROR_CONSOLE_BOLD_TEXT_ << "FATAL ERROR: " << _INFO_CONSOLE_TEXT << CConfigFile::getInstance().getFileName() << " does not have field " << _ERROR_CONSOLE_TEXT_ "[tracker_algorithm_index]" <<  _NORMAL_CONSOLE_TEXT_ << std::endl;
	
        exit(1);
    }

    if (!m_jsonConfig.contains("video_device"))
    {
        std::cout << _ERROR_CONSOLE_BOLD_TEXT_ << "FATAL ERROR: " << _INFO_CONSOLE_TEXT << CConfigFile::getInstance().getFileName() << " does not have field " << _ERROR_CONSOLE_TEXT_ "[video_device]" <<  _NORMAL_CONSOLE_TEXT_ << std::endl;
	
        exit(1);
    }

    m_tracker = std::make_unique<CTracker>(CTracker(this));

    bool res = m_tracker.get()->init(m_jsonConfig["tracker_algorithm_index"], m_jsonConfig["video_device"]);
    if (res == false)
    {
        std::cout << _ERROR_CONSOLE_BOLD_TEXT_ << "FATAL ERROR:" << _INFO_CONSOLE_TEXT << " Failed to initialize tracker. " <<  _NORMAL_CONSOLE_TEXT_ << std::endl;
        exit(1);
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

    m_tracker.get()->track(x,y,radius, true);
}

void CTrackerMain::stopTracking()
{
    m_tracker.get()->stop();
}


void CTrackerMain::onTrack (const float& x, const float& y, const float& width, const float& height) 
{
   static int counter=0;
    Json targets = Json::array();
    targets.push_back({
        {"a",x},
        {"b",y},
        {"c",width},
        {"d",height},
        {"n","target"}
    });

    Json message =
        {
            {"t", targets}
            // you can add r as radius
        };

    if (m_sendJMSG != NULL)
    { 
        // counter= counter +1;
        // if (counter%5!=0) 
        // {
        //     std::cout << "skip" << std::endl;
        //     return ;
        // }
        m_sendJMSG (std::string(""), message, TYPE_AndruavMessage_TrackingTargetLocation, true);
    }
}


void CTrackerMain::onTrackStatusChanged (const bool& track)  
{

}
