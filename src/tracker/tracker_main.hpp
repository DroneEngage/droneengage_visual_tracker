#ifndef TRACKER_MAIN_H
#define TRACKER_MAIN_H
#include <thread>
#include <mutex>
#include "../de_common/de_databus/de_module.hpp"
#include "../de_common/de_databus/de_common_callback.hpp"
#include <string>

#include "tracker.hpp"
#include "lock_on.hpp"
#include "tracker_facade.hpp"


#include "../de_common/de_databus/messages.hpp"


#include "../de_common/helpers/json_nlohmann.hpp"
#include <vector>
#include <chrono>
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
            
            void loopScheduler();

        public:

            void setupCapabilities();
            Json_de onCapabilityInvoke (const std::string& id, const std::string& ns,
                                        const std::string& act, const Json_de& params,
                                        std::string& err);

        public:
            
            void enableTracking();
            void startTrackingRect(const float x, const float y, const float w, const float h, bool is_ai_driven = false);
            void pauseTracking();
            void stopTracking();

        public:

            // confirmed observations from the perceiver (obs, 6546)
            void onObservations(const de::ai::ObservationFrame& frame);

            // lock-on (visual_tracker.lock / .unlock); manual rect commands unlock
            void lockTrk(const int64_t trk);
            void lockClass(const std::string& cls);
            void unlock();

            bool shouldContinueTracking();
            void onTrackerLost();
            void onTrackerRecovered();
            
        public:
            inline void setAITrackerStatus(const int status)
            {
                m_ai_tracker_status = status;
                m_tracker_facade.sendTrackingTargetStatus(std::string(""), m_tracker_status);
                publishCapabilityState();
            }

            void setAIClassList(const std::vector<std::string>& class_names);

            inline int getAITrackerStatus()
            {
                return m_ai_tracker_status;
            }
            inline bool getCameraFlipped() const
            {
                return m_camera_flipped;
            }
            inline bool getCrosshair() const
            {
                return m_crosshair;
            }
            

        public:
            //CCommon_Callback
            void OnConnectionStatusChangedWithAndruavServer (const int status) {};
        
        public:
            //CCallBack_Tracker
            void onTrack (const float& x, const float& y, const float& width, const float& height, const bool should_skip_message) override ;
            void onTrackStatusChanged (const int& track) override ;
            void onCameraOverlayRect (const float& x, const float& y, const float& width, const float& height) override ;
            void onCameraOverlayCrosshair (const float& cx, const float& cy, const float& arm) override ;
            void onCameraOverlayText (const float& x, const float& y, const float& text_height, const std::string& text) override ;
            void onCameraOverlayRemoveRect () override ;
            void onCameraOverlayRemoveText () override ;
            void onCameraOverlayRemove () override ;

            inline uint8_t getTrackingCameraDirection() const
            {
                return m_tracking_camera_direction;
            }
            inline void  setAiPriority(const bool priority)
            {
                m_ai_priority = priority;
                m_tracker_facade.sendTrackingTargetStatus(std::string(""), m_tracker_status);

            }
            inline bool getAiPriority() const
            {
                return m_ai_priority;
            }
            // resolved at config load (CA-0024): the CAMERA_OVERLAY_ACTION
            // target `t`; "" = legacy draw-on-all-cameras
            inline const std::string& getOverlayTarget() const
            {
                return m_overlay_target;
            }
        private:
            bool readConfigParameters();
            void reloadParametersIfConfigChanged();
            void publishCapabilityState();
            static const char *trackingStateName(const int status);
            static const char *aiStateName(const int status);
            void requestAIClassList();
            void seedTracker(const float x, const float y, const float w, const float h);
            de::tracker::LockConfig currentLockConfig() const;

        private:
        
            bool m_exit_thread = true;
            std::thread m_scheduler_thread;
            u_int64_t m_counter =0;
            
            int m_tracker_status = TrackingTarget_STATUS_TRACKING_STOPPED;
            int m_ai_tracker_status = TrackingTarget_STATUS_AI_Recognition_DISABLED;

            // class list learned from AI_Recognition_STATUS a=CLASS_LIST
            // (1077); feeds the visual_tracker.start {class} enum
            std::mutex m_ai_class_lock;
            std::vector<std::string> m_ai_class_names;
            bool m_caps_advertised = false;
            
            // lock-on target selection (obs carry stable, already confirmed track ids)
            std::mutex m_lock_mutex;
            de::tracker::CLockOn m_lock;
            int m_lock_class_reacquire_frames = 15;
            int m_lock_reseed_cooldown_frames = 10;

            // Tracker recovery behavior
            bool m_ai_assisted_recovery_enabled = true;
            bool m_ai_priority = false;                   // Prioritize AI detections over current tracking
            float m_ai_priority_distance_threshold = 0.2f; // Distance threshold for "out of square" (percentage of image diagonal)
            int m_tracker_lost_timeout_ms = 10000;  // 10 seconds timeout before requiring manual intervention
            std::chrono::steady_clock::time_point m_tracker_lost_timestamp;
            
            bool m_ema_init = false;
            double m_ema_x = 0, m_ema_y = 0;
            double m_ema_alpha_base = 0.3; // tune: 0.1..0.5
            std::unique_ptr<de::tracker::CTracker> m_tracker;
            de::tracker::CTracker_Facade& m_tracker_facade = de::tracker::CTracker_Facade::getInstance();

            // Parsed configuration values
            uint16_t m_camera_orientation = DEF_TRACK_ORIENTATION_DEG_0;
            // camera image is flipped
            bool m_camera_flipped = false;
            uint8_t m_tracking_camera_direction = TRACKING_CAMERA_DIRECTION_NONE;
            de::tracker::ENUM_TRACKER_TYPE m_tracker_algorithm_index = de::tracker::ENUM_TRACKER_TYPE::TRACKER_CSRT;
            bool m_crosshair = true;
            bool m_de_camera_draw = false;

            std::string m_source_video_device;
            std::string m_output_video_device;
            std::string m_overlay_target;
            int m_desired_input_width = 0;
            int m_desired_input_height = 0;

            uint16_t m_frames_to_skip_between_messages = FRAMES_TO_SKIP_BETWEEN_MESSAGES;
            uint16_t m_frame_to_skip_between_track_process = FRAMES_TO_SKIP_BETWEEN_TRACK_PROCESS;
    };

};
};


#endif