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

#include "../helpers/json.hpp"
using Json_de = nlohmann::json;

#include "tracker_main.hpp"

namespace de
{
namespace tracker
{


class CTrackerAndruavMessageParser
{
     public:

        CTrackerAndruavMessageParser()
        {

        };


        public:

            void parseMessage (Json_de &andruav_message, const char * message, const int & message_length);
            
        protected:
            
            void parseRemoteExecute (Json_de &andruav_message);
            
            inline bool validateField (const Json_de& message, const char *field_name, const Json_de::value_t field_type)
            {
                if (
                    (message.contains(field_name) == false) 
                    || (message[field_name].type() != field_type)
                    ) 
                    return false;

                return true;
            }

        private:
            de::tracker::CTrackerMain&  m_trackerMain = de::tracker::CTrackerMain::getInstance();
};

};
};

#endif