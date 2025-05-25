#include <opencv2/opencv.hpp>
#include <opencv2/tracking.hpp>
#include <opencv2/core/ocl.hpp>

#include "./helpers/colors.hpp"
#include "tracker.hpp"




using namespace uavos::tracker;

std::thread m_framesThread;
            

bool CTracker::init(const enum ENUM_TRACKER_TYPE tracker_type, const std::string& video_path)
{
    m_process = false;
    m_active_tracker = tracker_type;
    switch(m_active_tracker)
    {
        case ENUM_TRACKER_TYPE::TRACKER_BOOSTING:
            m_islegacy = true;
            m_legacy_tracker =cv::legacy::TrackerBoosting::create();
            break;
        
        case ENUM_TRACKER_TYPE::TRACKER_MIL:
            m_islegacy = false;
            m_tracker = cv::TrackerMIL::create();
            break;
        
        case ENUM_TRACKER_TYPE::TRACKER_KCF:
            m_islegacy = false;
            m_tracker = cv::TrackerKCF::create();
            break;
        
        case ENUM_TRACKER_TYPE::TRACKER_TLD:
            m_islegacy = true;
            m_legacy_tracker =cv::legacy::TrackerTLD::create();
            break;

        case ENUM_TRACKER_TYPE::TRACKER_MEDIANFLOW:
            m_islegacy = true;
            m_legacy_tracker = cv::legacy::TrackerMedianFlow::create();
            break;
        
        case ENUM_TRACKER_TYPE::TRACKER_GOTURN:
            m_islegacy = false;
            //m_tracker = cv::TrackerGOTURN::create();
            break;
        
        case ENUM_TRACKER_TYPE::TRACKER_MOSSE:
            //m_tracker = cv::TrackerMOSSE::create();
            break;
        
        case ENUM_TRACKER_TYPE::TRACKER_CSRT:
            m_islegacy = false;
            m_tracker = cv::TrackerCSRT::create();
            break;
    }

    m_video_path = video_path;

    video_capture_cap.open(video_path);
    if (!video_capture_cap.isOpened()) 
    {
        //TODO: send error message to WebClient please.

        std::cout << _ERROR_CONSOLE_BOLD_TEXT_ << "FATAL ERROR:" << _INFO_CONSOLE_TEXT << " could not open camera at " << _ERROR_CONSOLE_TEXT_ << video_path <<  _NORMAL_CONSOLE_TEXT_ << std::endl;
	
        return false;
    }

                
    video_capture_cap.set(
            cv::CAP_PROP_FRAME_WIDTH,
            m_image_width);
    
    video_capture_cap.set(
            cv::CAP_PROP_FRAME_HEIGHT,
            m_image_height);
       
    video_capture_cap.set(
            cv::CAP_PROP_FPS,
            m_image_fps); 

    std::cout << _SUCCESS_CONSOLE_BOLD_TEXT_ << "Tracker:" << getActiveTracker() << _SUCCESS_CONSOLE_TEXT_ << " has been initialized." << _NORMAL_CONSOLE_TEXT_ << std::endl;

    return true;
}



bool CTracker::uninit()
{
    stop();
    return true;
}

void CTracker::stop()
{
    m_process = false;
    m_valid_track = false;
    if (m_callback_tracker!= nullptr) 
    {
        m_callback_tracker->onTrackStatusChanged(false);
    }

    if (m_framesThread.joinable())   m_framesThread.join();
}

void CTracker::track(const float x, const float y, const float radius, const bool display)
{
    m_valid_track = false;

    std::cout << "X,y,r:" << std::to_string(x) << "," << std::to_string(y) <<  "," << std::to_string(radius) << _NORMAL_CONSOLE_TEXT_ << std::endl;
    
    if (m_video_path == std::string("")) 
    {
        //TODO: semd error message.
        return ;
    }
    
    
    
    m_framesThread = std::thread([x,y,radius,display,this](){
    
    cv::Mat frame;
    
    if (!video_capture_cap.isOpened()) 
    {
        //TODO: send error message to WebClient please.

        std::cout << _ERROR_CONSOLE_BOLD_TEXT_ << "FATAL ERROR:" << _INFO_CONSOLE_TEXT << " could not open camera at " << _ERROR_CONSOLE_TEXT_ << m_video_path <<  _NORMAL_CONSOLE_TEXT_ << std::endl;
	
        return ;
    }
    
    video_capture_cap >> frame;
    m_image_width = frame.cols;
    m_image_height = frame.rows;

    std::cout << _LOG_CONSOLE_TEXT_BOLD_ << "frame: " << _INFO_CONSOLE_TEXT << m_image_width << "x" << m_image_height << _NORMAL_CONSOLE_TEXT_ << std::endl;
    // Define initial bounding box 
    float scaled_x = x * m_image_width;
    float scaled_y = y * m_image_height;
    
    if (scaled_x<0) 
    {
        scaled_x = 0;
    } else if (scaled_x+radius>m_image_width)  
    {
        scaled_x =m_image_width-radius;
    }

    if (scaled_y<0) 
    {
        scaled_y= 0;
    } else if (scaled_y+radius>m_image_height) 
    {
        scaled_y =m_image_height-radius;
    }
    
    std::cout << "scaled_x,scaled_y:" << std::to_string(scaled_x) << ":"<< std::to_string(scaled_y) << std::endl;

    cv::Rect2d bbox_2d(scaled_x, scaled_y, radius, radius); 
    cv::Rect bbox(scaled_x, scaled_y, radius, radius); 

    // if (display)
    // {
    //     // Uncomment the line below to select a different bounding box 
    //     bbox_2d = cv::selectROI(frame, false); 
    //     // Display bounding box. 
    //     cv::rectangle(frame, bbox_2d, cv::Scalar( 0,255,255 ), 2, 1 ); 

    //     cv::imshow("Tracking", frame); 
    // }

    if (m_islegacy)
    {
        m_legacy_tracker->init(frame, bbox_2d);
    }
    else
    {
        m_tracker->init(frame, bbox);
    }
    
    
    m_process = true;

    while (m_process)
    {
        bool valid_track;
        video_capture_cap >> frame;
        if (!frame.empty())
        {
                            
            if (m_islegacy)
            {
                valid_track = m_legacy_tracker->update(frame, bbox_2d);
            }
            else
            {
                valid_track = m_tracker->update(frame, bbox);
            }
                
            if (valid_track != m_valid_track)
            {
                m_valid_track = valid_track;
                if (m_callback_tracker!= nullptr) m_callback_tracker->onTrackStatusChanged(m_valid_track);
            }
        }
        else
        {
            #ifdef DEBUG
		        std::cout << "empty frame" << std::endl;
            #endif
            continue;
        }
            
        if (m_valid_track)
        {
            // Tracking success : Draw the tracked object
            if (m_islegacy)
            {
                if (m_callback_tracker!= nullptr) m_callback_tracker->onTrack(revScaleX(bbox_2d.x), revScaleY(bbox_2d.y), revScaleX(bbox_2d.width), revScaleY(bbox_2d.height));
                
                if (display) cv::rectangle(frame, bbox_2d, cv::Scalar( 0,255,255 ), 2, 1 );
                #ifdef DEBUG
                    std::cout << "Tracking at " << bbox_2d << std::endl;
                #endif
            }
            else
            {
               if (m_callback_tracker!= nullptr) m_callback_tracker->onTrack(revScaleX(bbox.x), revScaleY(bbox.y), revScaleX(bbox.width), revScaleY(bbox.height));

                #ifdef DEBUG
                    std::cout << "Tracking at " << std::to_string(bbox.x) << "  --    " << std::to_string(bbox.y) << " xxx "
                    << std::to_string(bbox.width) << "  --    " << std::to_string(bbox.height) << std::endl;
                #endif
                if (display) cv::rectangle(frame, bbox, cv::Scalar( 0,255,255 ), 2, 1 );
            }
        }
        else
        {
            if (m_callback_tracker!= nullptr) m_callback_tracker->onTrackStatusChanged(false);

            // Tracking failure detected.
            if (display) 
            {
                cv::putText(frame, "Tracking failure detected", cv::Point(100,80), cv::FONT_HERSHEY_SIMPLEX, 0.75, cv::Scalar(0,0,255),2);
            }
        }
        

        if (display) 
        {
            // Display frame.
            cv::imshow("Tracking", frame);
            cv::waitKey(1);
        }   

    }
    std::cout << _LOG_CONSOLE_TEXT_BOLD_ << "tracking off" << _NORMAL_CONSOLE_TEXT_ << std::endl;
    });
}