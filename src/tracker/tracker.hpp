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


#define DEF_TRACK_ORIENTATION_DEG_0     0
#define DEF_TRACK_ORIENTATION_DEG_90    1
#define DEF_TRACK_ORIENTATION_DEG_180   2
#define DEF_TRACK_ORIENTATION_DEG_270   3


// Delay in processing OnTrack by the unit is high, so sending a full rate messages
// is not needed especially that mavlink module has its own timing and discards messsages 
// with small timespan.
// The reason I dont skip the tracking process itself is to increase the probability of locking on the object.
constexpr uint16_t FRAMES_TO_SKIP_BETWEEN_MESSAGES = 15;
constexpr uint16_t FRAMES_TO_SKIP_BETWEEN_TRACK_PROCESS = 5;

typedef struct buffer {
    void* start;
    size_t length;
} BUFFER;



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
            virtual void onTrack (const float& x, const float& y, const float& width, const float& height, const uint16_t camera_orientation, const bool camera_forward) = 0;
            virtual void onTrackStatusChanged (const int& track) = 0;
    };

    class CTracker 
    {

        public:

            CTracker(CCallBack_Tracker *callback_tracker):m_valid_track(false),m_callback_tracker(callback_tracker), m_output_video_path(""), m_output_video_active(false){
                
            };


            ~CTracker() 
            {
                uninit();
                //destroyVirtualVideoDevice(); // Clean up V4L2 resources
            };

            

        public:
            bool init (const enum ENUM_TRACKER_TYPE tracker_type, const std::string& video_path
                , const uint16_t camera_orientation , const bool camera_forward, const std::string& output_video_device
                , uint16_t frames_to_skip_between_messages, uint16_t frame_to_skip_between_track_process);
            bool uninit();
            void track(const float x, const float y, const float radius);
            void trackRect(const float x, const float y, const float w, const float h);
            void track2Rect(const float x, const float y, const float w, const float h);
            void pause();
            void stop();
            const std::string getActiveTracker() 
            {
                return trackerTypes[(int)m_active_tracker];
            };

            bool isTrackingValid() const 
            { 
                return m_valid_track;
            };


        public:

            inline const uint16_t isCameraForward()
            {
                return m_camera_forward;
            }

            inline const bool getCameraOrientation()
            {
                return m_camera_orientation;
            }

        protected:
            bool getVideoResolution(const std::string& video_device_path, unsigned int& width, unsigned int& height);
            bool initTargetVirtualVideoDevice(const std::string& output_video_device);
            void destroyVirtualVideoDevice();

        protected:
            /**
             * output from 0 to 1.0
             * (0,0) top left
             */
            inline float revScaleX(const float& x) const
            {
                return (x / m_image_width);
            };

            inline float revScaleY(const float& y) const
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
            
            
            cv::VideoCapture video_capture;
            CCallBack_Tracker * m_callback_tracker;
            
            std::string m_output_video_path;
            
            bool m_output_video_active = false;
            int m_video_fd = -1;
            int m_yuv_frame_size = 0;

            bool m_virtual_device_opened = false;
            bool m_is_tracking_active_initial = false;
            
            bool m_camera_forward    = false;
            uint16_t m_camera_orientation   = DEF_TRACK_ORIENTATION_DEG_0;

            uint32_t m_target_fps = 30; 

            BUFFER * m_buffers = nullptr;
            unsigned int m_buffer_count = 0;
            unsigned int m_current_buffer_index = 0;


            uint16_t m_frames_to_skip_between_messages  = FRAMES_TO_SKIP_BETWEEN_MESSAGES;
            uint16_t m_frame_to_skip_between_track_process = FRAMES_TO_SKIP_BETWEEN_TRACK_PROCESS;
    };

}
}


#endif