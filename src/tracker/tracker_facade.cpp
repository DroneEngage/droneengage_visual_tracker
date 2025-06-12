#include "../helpers/colors.hpp"
#include "tracker_facade.hpp"




using namespace de::tracker;

void CTracker_Facade::sendTrackingTargetsLocation( const std::string& target_party_id, const Json_de targets_location) const
{
    if (targets_location.empty())
    {
        return;
    }

    Json_de message =
    {
        {"t", targets_location}
    };

    #ifdef DEBUG
        std::cout << "tracking:" << targets_location.dump() << std::endl;
    #endif
    #ifdef DEBUG
        std::cout << _INFO_CONSOLE_BOLD_TEXT << "onTrack >> " 
        << _LOG_CONSOLE_BOLD_TEXT << targets_location.dump() << _NORMAL_CONSOLE_TEXT_ << std::endl;
    #endif
    m_module.sendJMSG(target_party_id, message, TYPE_AndruavMessage_TrackingTargetLocation, true);
}


void CTracker_Facade::sendTrackingTargetStatus(const std::string& target_party_id, const bool enabled) const
{
    int status = TrackingTarget_STATUS_TRACKING_LOST;
    if (enabled)
    {
        status  = TrackingTarget_STATUS_TRACKING_DETECTED;
    }
    Json_de message =
    {
        {"a", status}
    };

    m_module.sendJMSG(target_party_id, message, TYPE_AndruavMessage_TargetTracking_STATUS, true);
}