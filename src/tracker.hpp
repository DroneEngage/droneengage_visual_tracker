/**
 * @file tracker.hpp
 * @author Mohammad S. Hefny (mohammad.hefny@gmail.com)
 * @brief 
 * @version 0.1
 * @date 2022-03-20
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#ifndef TRACKER_H
#define TRACKER_H

#include <opencv2/opencv.hpp>
#include <opencv2/tracking.hpp>
#include <opencv2/tracking/tracking_legacy.hpp>


#include <opencv2/core/ocl.hpp>


namespace uavos
{
namespace tracker
{

    enum ENUM_TRACKER_TYPE 
    {
        TRACKER_BOOSTING    = 0,
        TRACKER_MIL         = 1,  
        TRACKER_KCF         = 2,
        TRACKER_TLD         = 3,  
        TRACKER_MEDIANFLOW  = 4,
        TRACKER_GOTURN      = 5,  
        TRACKER_MOSSE       = 6,
        TRACKER_CSRT        = 7,  
    };
    
    class CTracker 
    {

        public:

            static CTracker& getInstance()
            {
                static CTracker instance;

                return instance;
            }

            CTracker(CTracker const&)           = delete;
            void operator=(CTracker const&)                 = delete;

        private:

            CTracker() {};


        public:
            
            ~CTracker() 
            {

            };

            

        public:
            void init (const enum ENUM_TRACKER_TYPE tracker_type);
            void uninit();
            void track(const std::string& video_path, const bool display);
            const std::string getActiveTracker() 
            {
                return trackerTypes[(int)m_active_tracker];
            };

        protected:


        private:
            bool m_process = false;
            bool m_islegacy = false;
            enum ENUM_TRACKER_TYPE m_active_tracker;
            // List of tracker types in OpenCV 3.4.1
            const std::string trackerTypes[8] = {"BOOSTING", "MIL", "KCF", "TLD","MEDIANFLOW", "GOTURN", "MOSSE", "CSRT"};
            cv::Ptr<cv::Tracker> m_tracker;
	        cv::Ptr<cv::legacy::Tracker> m_legacy_tracker;
    };

}
}


#endif