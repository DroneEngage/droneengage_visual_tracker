#include <iostream>
#include "../de_common/messages.hpp"
#include "tracker_andruav_message_parser.hpp"

using namespace de::tracker;

/**
 * @brief Parse messages receuved from uavos_comm"
 * 
 * @param andruav_message message received from uavos_comm
 */
void CTrackerAndruavMessageParser::parseMessage (Json_de &andruav_message, const char * full_message, const int & full_message_length)
{
    const int messageType = andruav_message[ANDRUAV_PROTOCOL_MESSAGE_TYPE].get<int>();
    bool is_binary = !(full_message[full_message_length-1]==125 || (full_message[full_message_length-2]==125));   // "}".charCodeAt(0)  IS TEXT / BINARY Msg  
    
    if (messageType == TYPE_AndruavMessage_RemoteExecute)
    {
        parseRemoteExecute(andruav_message);

        return ;
    }

    else
    {
        Json_de cmd = andruav_message[ANDRUAV_PROTOCOL_MESSAGE_CMD];
        
        switch (messageType)
        {

            case TYPE_AndruavMessage_TrackingTarget_ACTION:
            {
                
                if (!cmd.contains("a") || !cmd["a"].is_number_integer()) return ;

                switch (cmd["a"].get<int>())
                {

                    case TrackingTarget_ACTION_TRACKING_POINT:
                    {
                    // a: center X
                    // b: center Y
                    // r: radius

                        if (!validateField(cmd, "b", Json_de::value_t::number_float)) return ;
                        if (!validateField(cmd, "c", Json_de::value_t::number_float)) return ;
                        if (!validateField(cmd, "r", Json_de::value_t::number_float)) return ;
                        
                        const float x = cmd["b"].get<float>();
                        const float y = cmd["c"].get<float>();
                        const float r = cmd["r"].get<float>();
                        
                        // minimum track dimension
                        if (r <= 0.01f) break;
                       
                        m_trackerMain.startTrackingRect(x, y, r, r);
                    }
                    break;

                    case TrackingTarget_ACTION_TRACKING_REGION:
                    {
                    
                        // b: 
                        // c:
                        // d:
                        // e:

                        if (!validateField(cmd, "b", Json_de::value_t::number_float)) return ;
                        if (!validateField(cmd, "c", Json_de::value_t::number_float)) return ;
                        if (!validateField(cmd, "d", Json_de::value_t::number_float)) return ;
                        if (!validateField(cmd, "e", Json_de::value_t::number_float)) return ;

                        const float x = cmd["b"].get<float>();
                        const float y = cmd["c"].get<float>();
                        const float w = cmd["d"].get<float>();
                        const float h = cmd["e"].get<float>();
                        
                        // minimum track dimension
                        if (w <= 0.01f) break;
                        if (h <= 0.01f) break;
                       
                        m_trackerMain.startTrackingRect(x, y, w, h);
                    }
                    break;
                    
                    case TrackingTarget_ACTION_TRACKING_STOP:
                        m_trackerMain.stopTracking();
                    break;

                    case TrackingTarget_ACTION_TRACKING_PAUSE:
                        m_trackerMain.pauseTracking();
                    break;
                }
            }
            break;

        }
    }
}


/**
 * @brief part of parseMessage that is responsible only for
 * parsing remote execute command.
 * 
 * @param andruav_message 
 */
void CTrackerAndruavMessageParser::parseRemoteExecute (Json_de &andruav_message)
{
    const Json_de cmd = andruav_message[ANDRUAV_PROTOCOL_MESSAGE_CMD];
    
    if (!validateField(cmd, "C", Json_de::value_t::number_unsigned)) return ;
                
    const int remoteCommand = cmd["C"].get<int>();
    std::cout << "cmd: " << remoteCommand << std::endl;
    
}

