#ifndef TRACKER_MAIN_H
#define TRACKER_MAIN_H

#include "./uavos_common/uavos_module.hpp"
#include "tracker.hpp"

namespace uavos
{
namespace tracker
{

    class CTrackerMain: public uavos::CMODULE, uavos::tracker::CCallBack_Tracker
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
                // Define module features
                m_module_features.push_back("TRK");

                m_module_class="computing";
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
            
            bool init() override;
            bool uninit() override;
            
        
        public:
            
            void startTracking(const float x, const float y, const float radius);
            void stopTracking();

            /**
             * @details register callback function to send message using it.
             * 
             * @param sendJMSG of type @link SEND_JMSG_CALLBACK @endlink 
             */
            void registerSendJMSG (SEND_JMSG_CALLBACK sendJMSG) override
            {
                m_sendJMSG = sendJMSG;
            };            

            /**
             * @details register callback function to send message using it.
             * 
             * @param sendBMSG of type @link SEND_BMSG_CALLBACK @endlink 
             */
            void registerSendBMSG (SEND_BMSG_CALLBACK sendBMSG) override
            {
                m_sendBMSG = sendBMSG;
            };            

            /**
             * @details register call back to send InterModule remote execute message.
             * 
             * @param sendMREMSG of type @link SEND_MREMSG_CALLBACK @endlink 
             */
            void registerSendMREMSG (SEND_MREMSG_CALLBACK sendMREMSG) override
            {
                m_sendMREMSG = sendMREMSG;
            };      
        
        public:
            //CCacdllBack_Tracker
            void onTrack (const float& x, const float& y, const float& width, const float& height) override ;
            void onTrackStatusChanged (const bool& track) override ;

        private:
            
            
            bool m_exit_thread;

            SEND_JMSG_CALLBACK      m_sendJMSG = NULL;
            SEND_BMSG_CALLBACK      m_sendBMSG = NULL;
            SEND_MREMSG_CALLBACK    m_sendMREMSG = NULL;

            std::unique_ptr<uavos::tracker::CTracker> m_tracker;
    };

};
};


#endif