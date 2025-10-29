#include <iostream>
#include "../de_common/de_databus/messages.hpp"
#include "tracker_andruav_message_parser.hpp"

using namespace de::tracker;

/**
 * @brief Parse messages receuved from uavos_comm"
 *
 * @param andruav_message message received from uavos_comm
 */
void CTrackerAndruavMessageParser::parseCommand(Json_de &andruav_message, const char *full_message, const int &full_message_length, int messageType, uint32_t permission)
{
    const Json_de cmd = andruav_message[ANDRUAV_PROTOCOL_MESSAGE_CMD];

    switch (messageType)
    {

    case TYPE_AndruavMessage_TrackingTarget_ACTION:
    {

        if (!cmd.contains("a") || !cmd["a"].is_number_integer())
            return;

        switch (cmd["a"].get<int>())
        {

        case TrackingTarget_ACTION_TRACKING_POINT:
        {
            // a: center X
            // b: center Y
            // r: radius

            if (!validateField(cmd, "b", Json_de::value_t::number_float))
                return;
            if (!validateField(cmd, "c", Json_de::value_t::number_float))
                return;
            if (!validateField(cmd, "r", Json_de::value_t::number_float))
                return;

            const float x = cmd["b"].get<float>();
            const float y = cmd["c"].get<float>();
            const float r = cmd["r"].get<float>();

            // minimum track dimension
            if (r <= 0.01f)
                break;

            m_trackerMain.startTrackingRect(x, y, r, r);
        }
        break;

        case TrackingTarget_ACTION_TRACKING_REGION:
        {

            // b:
            // c:
            // d:
            // e:

            if (!validateField(cmd, "b", Json_de::value_t::number_float))
                return;
            if (!validateField(cmd, "c", Json_de::value_t::number_float))
                return;
            if (!validateField(cmd, "d", Json_de::value_t::number_float))
                return;
            if (!validateField(cmd, "e", Json_de::value_t::number_float))
                return;

            const float x = cmd["b"].get<float>();
            const float y = cmd["c"].get<float>();
            const float w = cmd["d"].get<float>();
            const float h = cmd["e"].get<float>();

            // minimum track dimension
            if (w <= 0.01f)
                break;
            if (h <= 0.01f)
                break;

            m_trackerMain.startTrackingRect(x, y, w, h);
        }
        break;

        case TrackingTarget_ACTION_TRACKING_STOP:
            m_trackerMain.stopTracking();
            break;

        case TrackingTarget_ACTION_TRACKING_PAUSE:
            m_trackerMain.pauseTracking();
            break;

        case TrackingTarget_ACTION_TRACKING_ENABLE:
            m_trackerMain.enableTracking();
            break;
        }
    }
    break;

    case TYPE_AndruavMessage_AI_Recognition_STATUS:
    {
        if (!cmd.contains("a") || !cmd["a"].is_number_integer())
            return;
        const int status = cmd["a"].get<int>();

        m_trackerMain.setAITrackerStatus(status);

        switch (status)
        {
        case TrackingTarget_STATUS_AI_Recognition_LOST:
            break;
        case TrackingTarget_STATUS_AI_Recognition_DETECTED:
            break;
        case TrackingTarget_STATUS_AI_Recognition_ENABLED:
            break;
        case TrackingTarget_STATUS_AI_Recognition_DISABLED:
            break;
        }
    }
    break;

    case TYPE_AndruavMessage_AI_Recognition_TargetLocation:
    {

        if (cmd.contains("t") && cmd["t"].is_array())
        {
            const Json_de obj = cmd["t"];

#ifdef DDEBUG
            std::cout << "ALL AI Object:" << obj.dump() << std::endl;
#endif
        }

        if (cmd.contains("b") && cmd["b"].is_object())
        {
            const Json_de obj = cmd["b"];

            const float x = obj["x"].get<float>();
            const float y = obj["y"].get<float>();
            const float w = obj["w"].get<float>();
            const float h = obj["h"].get<float>();

            const float centerX = -0.5 + x + w / 2.0;
            const float centerY = -0.5 + y + h / 2.0;

#ifdef DDEBUG
            std::cout << "Best AI Object:" << obj.dump() << std::endl;
            std::cout << "Best AI Object:" << centerX << ":" << centerY << std::endl;
#endif

            m_trackerMain.onAITrackerBestRect(x, y, w, h);
        }
    }
    break;
    }
}

/**
 * @brief part of parseMessage that is responsible only for
 * parsing remote execute command.
 *
 * @param andruav_message
 */
void CTrackerAndruavMessageParser::parseRemoteExecute(Json_de &andruav_message)
{
    const Json_de cmd = andruav_message[ANDRUAV_PROTOCOL_MESSAGE_CMD];

    if (!validateField(cmd, "C", Json_de::value_t::number_unsigned))
        return;

    const int remoteCommand = cmd["C"].get<int>();
    std::cout << "cmd: " << remoteCommand << std::endl;
}
