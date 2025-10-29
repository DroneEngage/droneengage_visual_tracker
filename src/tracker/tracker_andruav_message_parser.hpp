#ifndef TRACKER_ANDRUAV_MESSAGE_PARSER_H
#define TRACKER_ANDRUAV_MESSAGE_PARSER_H

/**
 * @file tracker_andruav_message_parser.hpp
 * @author Mohammad S. Hefny (mohammad.hefny@gmail.com)
 * @brief
 * @version 0.1
 * @date 2022-03-24
 *
 * @copyright Copyright (c) 2022
 *
 */

#include "../de_common/helpers/json_nlohmann.hpp"
using Json_de = nlohmann::json;

#include "../de_common/de_databus/de_message_parser_base.hpp"

#include "tracker_main.hpp"

namespace de
{
    namespace tracker
    {

        class CTrackerAndruavMessageParser : public de::comm::CAndruavMessageParserBase
        {
        public:
            static CTrackerAndruavMessageParser& getInstance()
            {
                static CTrackerAndruavMessageParser instance;
                return instance;
            }

            CTrackerAndruavMessageParser(CTrackerAndruavMessageParser const &) = delete;
            void operator=(CTrackerAndruavMessageParser const &) = delete;

        private:
            CTrackerAndruavMessageParser() {}

        public:
            ~CTrackerAndruavMessageParser() {}

        protected:
            void parseRemoteExecute(Json_de &andruav_message) override;
            void parseCommand(Json_de &andruav_message, const char *full_message, const int &full_message_length, int messageType, uint32_t permission) override;

    
            inline bool validateField(const Json_de &message, const char *field_name, const Json_de::value_t field_type)
            {
                if (
                    (message.contains(field_name) == false) || (message[field_name].type() != field_type))
                    return false;

                return true;
            }

        private:
            de::tracker::CTrackerMain &m_trackerMain = de::tracker::CTrackerMain::getInstance();
        };

    };
};

#endif