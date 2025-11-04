#ifndef TRACKER_MAIN_H
#define TRACKER_MAIN_H

#include "../de_common/de_databus/de_module.hpp"
#include "../de_common/de_databus/de_common_callback.hpp"
#include <string>

#include "tracker.hpp"
#include "tracker_facade.hpp"



#include "../de_common/helpers/json_nlohmann.hpp"
using Json_de = nlohmann::json;

namespace de
{
namespace tracker
{

    class CTrackerMain: public de::tracker::CCallBack_Tracker, de::comm::CCommon_Callback
    {

        public:
            
            //https://stackoverflow.com/questions/1008019/c-singleton-design-pattern
            static CTrackerMain& getInstance()
            {
                static CTrackerMain instance;

                return instance;
            }

            CTrackerMain(CTrackerMain const&)           = delete;
            void operator=(CTrackerMain const&)         = delete;

        
            // Note: Scott Meyers mentions in his Effective Modern
            //       C++ book, that deleted functions should generally
            //       be public as it results in better error messages
            //       due to the compilers behavior to check accessibility
            //       before deleted status

        private:

            CTrackerMain()
            {
            }

        public:
            
            ~CTrackerMain()
            {
                if (m_exit_thread == false)
                {
                    uninit();
                }
            };
                

        public:
            
            bool init() ;
            bool uninit() ;
            
        
        public:
            
            void enableTracking();
            void startTrackingRect(const float x, const float y, const float w, const float h);
            void pauseTracking();
            void stopTracking();

        public:
            
            void onAITrackerBestRect(const float x, const float y, const float w, const float h);
            
            
        public:
            inline void setAITrackerStatus(const int status)
            {
                m_ai_tracker_status = status;
            }

            inline int getAITrackerStatus()
            {
                return m_ai_tracker_status;
            }

        public:
            //CCommon_Callback
            void OnConnectionStatusChangedWithAndruavServer (const int status) {};
        
        public:
            //CCallBack_Tracker
            void onTrack (const float& x, const float& y, const float& width, const float& height, const uint16_t camera_orientation, const bool camera_forward, const bool should_skip_message) override ;
            void onTrackStatusChanged (const int& track) override ;

        
        private:
            bool readConfigParameters();

        private:
        
            int m_tracker_status = TrackingTarget_STATUS_TRACKING_STOPPED;
            int m_ai_tracker_status = TrackingTarget_STATUS_AI_Recognition_DISABLED;
            
            bool m_exit_thread = true;
            
            bool m_ema_init = false;
            double m_ema_x = 0, m_ema_y = 0;
            double m_ema_alpha_base = 0.3; // tune: 0.1..0.5
            std::unique_ptr<de::tracker::CTracker> m_tracker;
            de::tracker::CTracker_Facade& m_trackerFacade = de::tracker::CTracker_Facade::getInstance();

            // Parsed configuration values
            uint16_t m_camera_orientation = DEF_TRACK_ORIENTATION_DEG_0;
            bool m_camera_forward = true;
            de::tracker::ENUM_TRACKER_TYPE m_tracker_algorithm_index = de::tracker::ENUM_TRACKER_TYPE::TRACKER_CSRT;

            std::string m_source_video_device;
            std::string m_output_video_device;
            int m_desired_input_width = 0;
            int m_desired_input_height = 0;

            uint16_t m_frames_to_skip_between_messages = FRAMES_TO_SKIP_BETWEEN_MESSAGES;
            uint16_t m_frame_to_skip_between_track_process = FRAMES_TO_SKIP_BETWEEN_TRACK_PROCESS;
    };

};
};


#endif