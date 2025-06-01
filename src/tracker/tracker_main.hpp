#ifndef TRACKER_MAIN_H
#define TRACKER_MAIN_H

#include "../de_common/de_module.hpp"
#include "../de_common/de_common_callback.hpp"
#include "tracker.hpp"
#include "tracker_facade.hpp"

#include "../helpers/json_nlohmann.hpp"
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
            
            void startTracking(const float x, const float y, const float radius);
            void stopTracking();


        public:
            //CCommon_Callback
            void OnConnectionStatusChangedWithAndruavServer (const int status) {};
        
        public:
            //CCallBack_Tracker
            void onTrack (const float& x, const float& y, const float& width, const float& height, const uint16_t camera_orientation, const bool camera_forward) override ;
            void onTrackStatusChanged (const bool& track) override ;

        private:
            
            
            bool m_exit_thread;

            std::unique_ptr<de::tracker::CTracker> m_tracker;
            de::tracker::CTracker_Facade& m_trackerFacade = de::tracker::CTracker_Facade::getInstance();
    };

};
};


#endif