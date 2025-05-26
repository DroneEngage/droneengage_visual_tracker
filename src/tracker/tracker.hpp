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

#include <thread>
#include <opencv2/opencv.hpp>
#include <opencv2/tracking.hpp>
#include <opencv2/tracking/tracking_legacy.hpp>


#include <opencv2/core/ocl.hpp>


namespace de
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
    

    class CCallBack_Tracker
    {
        public:
            virtual void onTrack (const float& x, const float& y, const float& width, const float& height) = 0;
            virtual void onTrackStatusChanged (const bool& track) = 0;
    };

    class CTracker 
    {

        public:

            CTracker(CCallBack_Tracker *callback_tracker):m_valid_track(false),m_callback_tracker(callback_tracker), m_target_video_path(""), m_target_video_active(false){
                
            };


            ~CTracker() 
            {
                uninit();
            };

            

        public:
            bool init (const enum ENUM_TRACKER_TYPE tracker_type, const std::string& video_path, const std::string& target_video_device);
            bool uninit();
            void track(const float x, const float y, const float radius, const bool display);
            void track2(const float x, const float y, const float radius, const bool display);
            void stop();
            const std::string getActiveTracker() 
            {
                return trackerTypes[(int)m_active_tracker];
            };

            bool isTrackingValid() const 
            { 
                return m_valid_track;
            };


        protected:
            bool getVideoResolution(const std::string& video_device_path, unsigned int& width, unsigned int& height);
            bool initTargetVirtualVideoDevice(const std::string& target_video_device);

        protected:
            
            float revScaleX(const float& x) const
            {
                return (x / m_image_width);
            };

            float revScaleY(const float& y) const
            {
                return (y / m_image_height);
            };
            

        private:
            bool m_process = false;
            bool m_valid_track;

            bool m_islegacy = false;
            std::string m_video_path;
            enum ENUM_TRACKER_TYPE m_active_tracker;
            // List of tracker types in OpenCV 3.4.1
            const std::string trackerTypes[8] = {"BOOSTING", "MIL", "KCF", "TLD","MEDIANFLOW", "GOTURN", "MOSSE", "CSRT"};
            cv::Ptr<cv::Tracker> m_tracker;
	        cv::Ptr<cv::legacy::Tracker> m_legacy_tracker;

            int m_image_width  = 640;
            int m_image_height = 480;
            int m_image_fps = 5;
            
            cv::VideoCapture video_capture_cap;
            CCallBack_Tracker * m_callback_tracker;
            
            std::string m_target_video_path;
            
            bool m_target_video_active = false;
            int m_video_fd = -1;
            int m_yuv_frame_size = 0;

            bool m_virtual_device_opened = false;
    };

}
}


#endif