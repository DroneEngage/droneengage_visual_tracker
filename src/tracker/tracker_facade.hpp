#ifndef TRACKER_FACADE_MODULE_H
#define TRACKER_FACADE_MODULE_H


#include "../de_common/helpers/json_nlohmann.hpp"

using Json_de = nlohmann::json;

#include "../de_common/de_databus/de_facade_base.hpp"
#include "../de_common/de_databus/messages.hpp"




namespace de
{
namespace tracker
{
    class CTracker_Facade : public de::comm::CFacade_Base
    {
        public:
            //https://stackoverflow.com/questions/1008019/c-singleton-design-pattern
            static CTracker_Facade& getInstance()
            {
                static CTracker_Facade instance;

                return instance;
            }

            CTracker_Facade(CTracker_Facade const&)         = delete;
            void operator=(CTracker_Facade const&)          = delete;

        
        private:

            CTracker_Facade()
            {
            };

        public:
            
            ~CTracker_Facade ()
            {
                
            };
                

        public:
           
            void sendTrackingTargetsLocation(const std::string& target_party_id, const Json_de targets_location) const;
            void sendTrackingTargetStatus(const std::string& target_party_id, const int status) const;
            void sendTrackingConfig(const std::string& target_party_id) const;
            void sendCameraOverlayRect(const std::string& target_party_id, const float& x, const float& y, const float& w, const float& h) const;
            void sendCameraOverlayCrosshair(const std::string& target_party_id, const float& cx, const float& cy, const float& arm) const;
            void sendCameraOverlayText(const std::string& target_party_id, const float& x, const float& y, const float& text_height, const std::string& text) const;
            void sendCameraOverlayRemoveRect(const std::string& target_party_id) const;
            void sendCameraOverlayRemoveText(const std::string& target_party_id) const;
            void sendCameraOverlayRemove(const std::string& target_party_id) const;
            void sendAIRecognitionAction(const Json_de& cmd) const;
        protected:

            mutable uint32_t m_overlay_send_count = 0;
            mutable uint32_t m_overlay_crosshair_send_count = 0;
            mutable uint32_t m_overlay_text_send_count = 0;

    };
}
}


#endif