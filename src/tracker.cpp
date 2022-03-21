#include <opencv2/opencv.hpp>
#include <opencv2/tracking.hpp>
#include <opencv2/core/ocl.hpp>
#include "tracker.hpp"




using namespace uavos::tracker;



void CTracker::init(const enum ENUM_TRACKER_TYPE tracker_type)
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

    std::cout << "Tracker:" << getActiveTracker() << " has been initialized." << std::endl;
}



void CTracker::uninit()
{
    m_process = false;
}


void CTracker::track(const std::string& video_path, const bool display)
{
    bool valid_track;
    m_process = true;

    cv::VideoCapture video_capture_cap;
    
    video_capture_cap.open(video_path);
    if (video_capture_cap.isOpened()) {
            
        video_capture_cap.set(
            cv::CAP_PROP_FRAME_WIDTH,
            640);
        
        video_capture_cap.set(
            cv::CAP_PROP_FRAME_HEIGHT,
            480);
        
        video_capture_cap.set(
            cv::CAP_PROP_FPS,
            10); 
         }
    cv::Mat frame;
    
    video_capture_cap >> frame;

    // Define initial bounding box 
    cv::Rect2d bbox_2d(287, 23, 86, 320); 
    cv::Rect bbox(287, 23, 86, 320); 

    // Uncomment the line below to select a different bounding box 
    bbox_2d = cv::selectROI(frame, false); 
    // Display bounding box. 
    cv::rectangle(frame, bbox_2d, cv::Scalar( 1, 0, 0 ), 2, 1 ); 

    cv::imshow("Tracking", frame); 
    if (m_islegacy)
    {
        m_legacy_tracker->init(frame, bbox_2d);
    }
    else
    {
        m_tracker->init(frame, bbox);
    }

    while (m_process)
    {
        video_capture_cap >> frame;
        if (!frame.empty())
        {
            #ifdef DEBUG
		        std::cout << "push frame" << std::endl;
                std::cout << "GOT A FRAME" << std::endl;                                   
            #endif
            // Process the frame
            if (!frame.empty())
            {
                #ifdef DEBUG
		            std::cout << "PROCESS A FRAME" << std::endl;
                #endif
                            
                if (m_islegacy)
                {
                    valid_track = m_legacy_tracker->update(frame, bbox_2d);
                }
                else
                {
                    valid_track = m_tracker->update(frame, bbox);
                    
                    
                }
            }
            
            if (valid_track)
            {
                // Tracking success : Draw the tracked object
                if (m_islegacy)
                {
                    cv::rectangle(frame, bbox_2d, cv::Scalar( 1, 0, 0 ), 2, 1 );
                }
                else
                {
                    cv::rectangle(frame, bbox, cv::Scalar( 1, 0, 0 ), 2, 1 );
                }
            }
            else
            {
                // Tracking failure detected.
                cv::putText(frame, "Tracking failure detected", cv::Point(100,80), cv::FONT_HERSHEY_SIMPLEX, 0.75, cv::Scalar(0,0,255),2);
            }
        

        // Display frame.
        cv::imshow("Tracking", frame);
		cv::waitKey(1);   
        }
        else
        {
            #ifdef DEBUG
		        std::cout << "empty frame" << std::endl;
            #endif
        }

        
        
            
    }
}