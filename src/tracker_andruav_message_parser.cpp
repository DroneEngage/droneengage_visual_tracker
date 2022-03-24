#include <iostream>
#include "./uavos_common/messages.hpp"
#include "tracker_andruav_message_parser.hpp"

using namespace uavos::tracker;

/**
 * @brief Parse messages receuved from uavos_comm"
 * 
 * @param andruav_message message received from uavos_comm
 */
void CTrackerAndruavMessageParser::parseMessage (Json &andruav_message, const char * full_message, const int & full_message_length)
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
        Json message = andruav_message[ANDRUAV_PROTOCOL_MESSAGE_CMD];
        
        switch (messageType)
        {

            case TYPE_AndruavMessage_Arm:
            {
                // A  : bool arm/disarm
                // [D]: bool force 
                if (!validateField(message, "A", Json::value_t::boolean)) return ;
                
                bool arm = message["A"].get<bool>();
                bool force = false;
                if (message.contains("D") == true)
                {
                    if (!validateField(message, "D", Json::value_t::boolean)) return ;
                
                    force = message["D"].get<bool>();
                }
                //mavlinksdk::CMavlinkCommand::getInstance().doArmDisarm(arm,force);
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
void CTrackerAndruavMessageParser::parseRemoteExecute (Json &andruav_message)
{
    const Json cmd = andruav_message[ANDRUAV_PROTOCOL_MESSAGE_CMD];
    
    if (!validateField(cmd, "C", Json::value_t::number_unsigned)) return ;
                
    const int remoteCommand = cmd["C"].get<int>();
    std::cout << "cmd: " << remoteCommand << std::endl;
    switch (remoteCommand)
    {

    }
}

