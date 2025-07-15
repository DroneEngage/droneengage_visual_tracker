#include <chrono> // For high-resolution timing
#include <thread> // For std::this_thread::sleep_for
#include <sys/mman.h>  // For mmap, munmap, PROT_READ, PROT_WRITE, MAP_SHARED, MAP_FAILED
#include <opencv2/opencv.hpp>
#include <opencv2/tracking.hpp>
#include <opencv2/core/ocl.hpp>

#include "../helpers/colors.hpp"
#include "video.hpp"
#include "tracker.hpp"

// Headers for V4L2
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/videodev2.h>
#include "../de_common/messages.hpp"
using namespace de::tracker;

std::thread m_framesThread;



// Original rule intent: FRAMES_TO_SKIP_BETWEEN_MESSAGES > FRAMES_TO_SKIP_BETWEEN_TRACK_PROCESS
static_assert(FRAMES_TO_SKIP_BETWEEN_MESSAGES > FRAMES_TO_SKIP_BETWEEN_TRACK_PROCESS,
              "FRAMES_TO_SKIP_BETWEEN_MESSAGES must be greater than FRAMES_TO_SKIP_BETWEEN_TRACK_PROCESS");

bool CTracker::initTargetVirtualVideoDevice(const std::string &output_video_device)
{
    m_virtual_device_opened = false;

    if (output_video_device != std::string(""))
    {
        m_output_video_path = output_video_device;
        m_output_video_active = true;
    }
    else
    {
        m_output_video_active = false;
        // not an error
        return true;
    }
    
    cv::Mat frame;
    video_capture >> frame;
    cv::Mat yuv_frame;
    cv::cvtColor(frame, yuv_frame, cv::COLOR_BGR2YUV_I420);

    // Open the virtual video device
    m_video_fd = open(m_output_video_path.c_str(), O_RDWR | O_NONBLOCK);

    std::cout << _SUCCESS_CONSOLE_BOLD_TEXT_ << "Video Output:" << _LOG_CONSOLE_BOLD_TEXT << m_output_video_path << _NORMAL_CONSOLE_TEXT_ << std::endl;

    if (m_video_fd < 0)
    {
        std::cout << "Error: Could not open virtual video device " << m_output_video_path << ": " << strerror(errno) << std::endl;
        return false;
    }

    std::cout << _SUCCESS_CONSOLE_BOLD_TEXT_<< "Successfully opened virtual video device: " << _LOG_CONSOLE_BOLD_TEXT << m_output_video_path << std::endl;

    struct v4l2_format fmt = {0};
    fmt.type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
    fmt.fmt.pix.width = m_image_width;
    fmt.fmt.pix.height = m_image_height;
    fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_YUV420;
    fmt.fmt.pix.field = V4L2_FIELD_NONE;
    fmt.fmt.pix.bytesperline = m_image_width;
    fmt.fmt.pix.sizeimage = (m_image_width * m_image_height * 3) / 2;

    if (CVideo::xioctl(m_video_fd, VIDIOC_S_FMT, &fmt) < 0)
    {
        fprintf(stderr, "Failed to set video format on %s: %s\n", m_output_video_path.c_str(), strerror(errno));
        close(m_video_fd);
        video_capture.release();
        return false;
    }
    fprintf(stderr, "Successfully set format for %s: %dx%d, pixformat YUV420\n",
            m_output_video_path.c_str(), fmt.fmt.pix.width, fmt.fmt.pix.height);

    // Request buffers for memory mapping
    struct v4l2_requestbuffers req = {0};
    req.count = 4;
    req.type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
    req.memory = V4L2_MEMORY_MMAP;

    if (CVideo::xioctl(m_video_fd, VIDIOC_REQBUFS, &req) < 0)
    {
        fprintf(stderr, "Failed to request buffers on %s: %s\n", m_output_video_path.c_str(), strerror(errno));
        close(m_video_fd);
        video_capture.release();
        return false;
    }

    if (req.count < 2)
    {
        std::cout << "Insufficient buffer memory on " <<  m_output_video_path << std::endl;
        close(m_video_fd);
        video_capture.release();
        return false;
    }

    // Map the buffers
    m_buffers = new (std::nothrow) buffer[req.count];
    
    if (!m_buffers) {
        std::cout << "Error: Failed to allocate memory for V4L2 buffers." << std::endl;
        close(m_video_fd);
        m_video_fd = -1;
        return false;
    }

    for (unsigned int i = 0; i < req.count; ++i)
    {
        m_buffers[i].start = nullptr; // Initialize to nullptr
        m_buffers[i].length = 0;      // Initialize to 0

        struct v4l2_buffer buf = {0};
        buf.type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
        buf.memory = V4L2_MEMORY_MMAP;
        buf.index = i;

        if (CVideo::xioctl(m_video_fd, VIDIOC_QUERYBUF, &buf) < 0)
        {
            fprintf(stderr, "Failed to query buffer %u on %s: %s\n", i, m_output_video_path.c_str(), strerror(errno));
            close(m_video_fd);
            video_capture.release();
            return false;
        }

        m_buffers[i].length = buf.length;
        m_buffers[i].start = mmap(NULL, buf.length,
                                 PROT_READ | PROT_WRITE,
                                 MAP_SHARED,
                                 m_video_fd, buf.m.offset);

        if (m_buffers[i].start == MAP_FAILED)
        {
            fprintf(stderr, "Failed to map buffer %u on %s: %s\n", i, m_output_video_path.c_str(), strerror(errno));
            close(m_video_fd);
            video_capture.release();
            return false;
        }
    }

    m_yuv_frame_size = m_image_width * m_image_height * 3 / 2;
    m_buffer_count = req.count;
    m_current_buffer_index = 0;

    m_virtual_device_opened = true;
    return true;
}

bool CTracker::init(const enum ENUM_TRACKER_TYPE tracker_type, const std::string& video_path
    , const uint16_t camera_orientation, const bool camera_forward, const std::string& output_video_device
    , uint16_t frames_to_skip_between_messages, uint16_t frame_to_skip_between_track_process)
{

    // Enable OpenCV OpenCL acceleration if available for operations like cvtColor
    cv::ocl::setUseOpenCL(true);

    if (frames_to_skip_between_messages < frame_to_skip_between_track_process)
    {
        std::cout << _ERROR_CONSOLE_BOLD_TEXT_ << "FATAL ERROR:" << _INFO_CONSOLE_TEXT << " frames_to_skip_between_messages should be larger than frame_to_skip_between_track_process. " <<  _NORMAL_CONSOLE_TEXT_ << std::endl;
        exit(1);
    }

    m_camera_forward = camera_forward;
    m_camera_orientation = camera_orientation;
    
    m_process = false;
    m_active_tracker = tracker_type;
    switch (m_active_tracker)
    {
    case ENUM_TRACKER_TYPE::TRACKER_BOOSTING:
        m_islegacy = true;
        m_legacy_tracker = cv::legacy::TrackerBoosting::create();
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
        m_legacy_tracker = cv::legacy::TrackerTLD::create();
        break;

    case ENUM_TRACKER_TYPE::TRACKER_MEDIANFLOW:
        m_islegacy = true;
        m_legacy_tracker = cv::legacy::TrackerMedianFlow::create();
        break;

    case ENUM_TRACKER_TYPE::TRACKER_GOTURN:
        m_islegacy = false;
        // m_tracker = cv::TrackerGOTURN::create();
        break;

    case ENUM_TRACKER_TYPE::TRACKER_MOSSE:
        // m_tracker = cv::TrackerMOSSE::create();
        break;

    case ENUM_TRACKER_TYPE::TRACKER_CSRT:
        m_islegacy = false;
        m_tracker = cv::TrackerCSRT::create();
        break;
    }

    m_video_path = video_path;
    
    unsigned int detected_width = 0;
    unsigned int detected_height = 0;

    if (!CVideo::getVideoResolution(m_video_path, detected_width, detected_height))
    {
        std::cout << _ERROR_CONSOLE_BOLD_TEXT_ << "FATAL ERROR:" << _BK_RED_WHITE_TEXT_ << " could not access camera at " << _ERROR_CONSOLE_TEXT_ << video_path << _NORMAL_CONSOLE_TEXT_ << std::endl;

        return false;
    }
    
    m_image_width = detected_width;
    m_image_height = detected_height;
    
    video_capture.open(video_path);
    if (!video_capture.isOpened())
    {
        // TODO: send error message to WebClient please.

        std::cout << _ERROR_CONSOLE_BOLD_TEXT_ << "FATAL ERROR:" << _BK_RED_WHITE_TEXT_ << " could not open camera at " << _ERROR_CONSOLE_TEXT_ << video_path << _NORMAL_CONSOLE_TEXT_ << std::endl;

        return false;
    }
    
    std::cout << _SUCCESS_CONSOLE_BOLD_TEXT_ << "Video Capture:" << _LOG_CONSOLE_BOLD_TEXT << video_path << _NORMAL_CONSOLE_TEXT_ << std::endl;
   
    // Read the first frame to get its actual width and height
    
    video_capture.set(cv::CAP_PROP_POS_FRAMES, 0); // Rewind to start of video for actual tracking

    video_capture.set(
        cv::CAP_PROP_FRAME_WIDTH,
        m_image_width);

    video_capture.set(
        cv::CAP_PROP_FRAME_HEIGHT,
        m_image_height);

    video_capture.set(
        cv::CAP_PROP_FPS,
        m_image_fps);

    video_capture.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));


    // --- V4L2 Output Device Initialization ---
    if (!initTargetVirtualVideoDevice(output_video_device))
    {
        video_capture.release();
        return false;
    }

    std::cout << _SUCCESS_CONSOLE_BOLD_TEXT_ << "Tracker:" << getActiveTracker() << _SUCCESS_CONSOLE_TEXT_ << " has been initialized." << _NORMAL_CONSOLE_TEXT_ << std::endl;

    return true;
}
void CTracker::destroyVirtualVideoDevice()
{
    if (m_buffers) // Only proceed if m_buffers was allocated
    {
        for (unsigned int i = 0; i < m_buffer_count; ++i)
        {
            if (m_buffers[i].start != MAP_FAILED && m_buffers[i].start != nullptr)
            {
                if (munmap(m_buffers[i].start, m_buffers[i].length) == -1)
                {
                    std::cerr << "Error: Failed to munmap buffer " << i << ": " << strerror(errno) << std::endl;
                    // Log the error but continue to try and unmap other buffers
                }
            }
        }
        delete[] m_buffers;
        m_buffers = nullptr; // Prevent dangling pointer and double deallocation
        m_buffer_count = 0;
    }

    if (m_video_fd != -1) // Close file descriptor if it's open
    {
        close(m_video_fd);
        m_video_fd = -1; // Indicate file descriptor is closed
        std::cout << "Virtual video device closed." << std::endl;
    }
    m_virtual_device_opened = false;
}
bool CTracker::uninit()
{
    stop();
    
    
    return true;
}


void CTracker::pause()
{
    m_is_tracking_active_initial = false;

    if (m_callback_tracker != nullptr)
    {
        m_callback_tracker->onTrackStatusChanged(TrackingTarget_STATUS_TRACKING_STOPPED);
    }
}

void CTracker::stop()
{
    if (!m_process) return ; 
    m_process = false;
    m_valid_track = false;
    if (m_callback_tracker != nullptr)
    {
        m_callback_tracker->onTrackStatusChanged(TrackingTarget_STATUS_TRACKING_STOPPED);
    }

    if (m_framesThread.joinable())
        m_framesThread.join();
}

void CTracker::trackRect(const float x, const float y, const float w, const float h)
{
    m_valid_track = false;

    std::cout << _INFO_CONSOLE_BOLD_TEXT << "X:" << _LOG_CONSOLE_TEXT << std::to_string(x) << _INFO_CONSOLE_BOLD_TEXT << ",Y:" << _LOG_CONSOLE_TEXT << std::to_string(y) << _INFO_CONSOLE_BOLD_TEXT << ",W:" << _LOG_CONSOLE_TEXT << std::to_string(w)  << ",H:" << _LOG_CONSOLE_TEXT << std::to_string(h) << _NORMAL_CONSOLE_TEXT_ << std::endl;

    if (m_video_path == std::string(""))
    {
        // TODO: send error message.
        return;
    }

    m_framesThread = std::thread([x, y, w, h, this]()
                                 { this->track2Rect(x, y, w, h); });
}

void CTracker::track(const float x, const float y, const float radius)
{
    m_valid_track = false;

    std::cout << _INFO_CONSOLE_BOLD_TEXT << "X:" << _LOG_CONSOLE_TEXT << std::to_string(x) << _INFO_CONSOLE_BOLD_TEXT << ",Y:" << _LOG_CONSOLE_TEXT << std::to_string(y) << _INFO_CONSOLE_BOLD_TEXT << ",R:" << _LOG_CONSOLE_TEXT << std::to_string(radius) << _NORMAL_CONSOLE_TEXT_ << std::endl;

    if (m_video_path == std::string(""))
    {
        // TODO: send error message.
        return;
    }

    m_framesThread = std::thread([x, y, radius, this]()
                                 { this->track2Rect(x, y, radius, radius); });
}


void CTracker::track2Rect(const float x, const float y, const float w, const float h)
{
    // Pre-allocate Mats to avoid re-allocations in the loop
    // 'frame' will store the captured video frame (BGR)
    // 'yuv_frame' will store the converted YUV frame for streaming
    cv::Mat frame;
    cv::Mat yuv_frame;

    // --- Frame Rate Control Variables ---
    const std::chrono::milliseconds target_frame_time_ms(1000 / m_target_fps);

    // --- Initial Setup and Validation ---
    if (!video_capture.isOpened())
    {
        std::cerr << _ERROR_CONSOLE_BOLD_TEXT_ << "FATAL ERROR:" << _INFO_CONSOLE_TEXT << " could not open camera at "
                  << _ERROR_CONSOLE_TEXT_ << m_video_path << _NORMAL_CONSOLE_TEXT_ << std::endl;
        return;
    }

    // Capture the first frame to get dimensions. This only needs to happen once.
    video_capture >> frame;
    if (frame.empty())
    {
        std::cerr << _ERROR_CONSOLE_BOLD_TEXT_ << "ERROR:" << _INFO_CONSOLE_TEXT << " First frame is empty from "
                  << _ERROR_CONSOLE_TEXT_ << m_video_path << _NORMAL_CONSOLE_TEXT_ << std::endl;
        return;
    }

    const int center_x = frame.cols / 2;
    const int center_y = frame.rows / 2;
    // Define the length of the cross arms (you can adjust this)
    const int cross_arm_length = 30; // pixels

    float scaled_width = w * m_image_width;
    float scaled_height = h * m_image_height;
    // Clamp coordinates within frame boundaries using std::clamp for conciseness and safety
    // Assuming radius is the side length, so bbox_x + radius should be <= width
    float scaled_x = std::clamp(x * m_image_width, 0.0f, m_image_width - scaled_width);
    float scaled_y = std::clamp(y * m_image_height, 0.0f, m_image_height - scaled_height);

    cv::Rect2d bbox_2d(scaled_x, scaled_y, scaled_width, scaled_height);
    cv::Rect bbox(bbox_2d);

    m_is_tracking_active_initial = (x > 0);
    if (m_is_tracking_active_initial) {
        // Re-initialize tracker with the new bounding box on the first frame
        if (m_islegacy) m_legacy_tracker->init(frame, bbox_2d);
        else m_tracker->init(frame, bbox);
        
        if (m_callback_tracker) m_callback_tracker->onTrackStatusChanged(TrackingTarget_STATUS_TRACKING_DETECTED);
    } else {
        if (m_callback_tracker) m_callback_tracker->onTrackStatusChanged(TrackingTarget_STATUS_TRACKING_STOPPED);
    }

    m_process = true;
    bool track_success = m_is_tracking_active_initial;
    uint64_t frame_counter = 0;

    bool current_valid_track_status = false; // Initialize to false, assuming no track initially
    
    // --- Main Tracking and Streaming Loop ---
    while (m_process)
    {
        auto start_time = std::chrono::high_resolution_clock::now();

        video_capture >> frame;
        if (frame.empty())
        {
            std::cerr << _ERROR_CONSOLE_BOLD_TEXT_ << "WARNING:" << _INFO_CONSOLE_TEXT << " Captured an empty frame. Stopping tracking."
                      << _NORMAL_CONSOLE_TEXT_ << std::endl;
            if (m_callback_tracker) m_callback_tracker->onTrackStatusChanged(TrackingTarget_STATUS_TRACKING_STOPPED);
            break;
        }

        // --- Tracking Logic ---
        if (m_is_tracking_active_initial)// Use the initial tracking state
        {
            const bool should_skip_track_process = (frame_counter % FRAMES_TO_SKIP_BETWEEN_TRACK_PROCESS) != 0;

            // --- REFINED: Update tracking status only when processed ---
            if (!should_skip_track_process) {
                track_success = m_islegacy ? m_legacy_tracker->update(frame, bbox_2d) : m_tracker->update(frame, bbox);
                
                // Report status change immediately after an update attempt
                if (m_callback_tracker) {
                    if (track_success != current_valid_track_status)
                    {  
                        current_valid_track_status = track_success;
                        m_callback_tracker->onTrackStatusChanged(track_success ? TrackingTarget_STATUS_TRACKING_DETECTED : TrackingTarget_STATUS_TRACKING_LOST);
                    }
                }
            }

            if (track_success)
            {
                // Draw bounding box on every frame regardless of update skip
                if (m_islegacy) cv::rectangle(frame, bbox_2d, cv::Scalar(0, 255, 255), 2, 1);
                else cv::rectangle(frame, bbox, cv::Scalar(0, 255, 255), 2, 1);
                
                const bool should_skip_message = (frame_counter % FRAMES_TO_SKIP_BETWEEN_MESSAGES) != 0;
                if (!should_skip_message && m_callback_tracker)
                {
                    if (m_islegacy) {
                        m_callback_tracker->onTrack(revScaleX(bbox_2d.x), revScaleY(bbox_2d.y),
                                                   revScaleX(bbox_2d.width), revScaleY(bbox_2d.height),
                                                   m_camera_orientation, m_camera_forward);
                    } else {
                        m_callback_tracker->onTrack(revScaleX(bbox.x), revScaleY(bbox.y),
                                                   revScaleX(bbox.width), revScaleY(bbox.height),
                                                   m_camera_orientation, m_camera_forward);
                    }
                }
            }
            else
            {
                cv::putText(frame, "Tracking failure detected", cv::Point(100, 80), cv::FONT_HERSHEY_SIMPLEX, 0.75, cv::Scalar(0, 0, 255), 2);
            }
             // Always draw crosshair if tracking was started
            cv::line(frame, cv::Point(center_x - cross_arm_length, center_y), cv::Point(center_x + cross_arm_length, center_y), cv::Scalar(0, 255, 0), 2);
            cv::line(frame, cv::Point(center_x, center_y - cross_arm_length), cv::Point(center_x, center_y + cross_arm_length), cv::Scalar(0, 255, 0), 2);
        }

        // --- OPTIMIZATION: Use V4L2 buffer queuing instead of write() ---
        if (m_virtual_device_opened)
        {
            cv::cvtColor(frame, yuv_frame, cv::COLOR_BGR2YUV_I420);
            
            struct v4l2_buffer buf = {};
            buf.type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
            buf.memory = V4L2_MEMORY_MMAP;

            // Dequeue a buffer (non-blocking)
            if (CVideo::xioctl(m_video_fd, VIDIOC_DQBUF, &buf) == 0)
            {
                // Copy frame data into the dequeued buffer
                memcpy(m_buffers[buf.index].start, yuv_frame.data, m_yuv_frame_size);
                
                // Queue the buffer back to the driver
                if (CVideo::xioctl(m_video_fd, VIDIOC_QBUF, &buf) < 0) {
                    perror("VIDIOC_QBUF");
                    m_process = false; // Stop on critical error
                }
            }
            else if (errno != EAGAIN) {
                perror("VIDIOC_DQBUF");
                m_process = false; // Stop on critical error
            }
            // If errno is EAGAIN, it means the output queue is full. We simply drop the frame,
            // which is the desired behavior to prevent the pipeline from blocking.
        }

        auto end_time = std::chrono::high_resolution_clock::now();
        auto elapsed_time = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);

        if (elapsed_time < target_frame_time_ms)
        {
            #ifdef DDEBUG
                        std::cout << "Elapsed: " << elapsed_time.count() << "ms, Sleeping for: " << time_to_sleep.count() << "ms" << std::endl;
            #endif
            std::this_thread::sleep_for(target_frame_time_ms - elapsed_time);
        }
        #ifdef DDEBUG
        else
        {
            std::cout << _INFO_CONSOLE_BOLD_TEXT << "Warning: Frame processing took " << _LOG_CONSOLE_BOLD_TEXT << elapsed_time.count() << "ms " << _INFO_CONSOLE_BOLD_TEXT << ", exceeding target "
                    << _LOG_CONSOLE_BOLD_TEXT << target_frame_time_ms.count() << "ms." << _INFO_CONSOLE_BOLD_TEXT << " Cannot maintain 30 FPS." << _NORMAL_CONSOLE_TEXT_ << std::endl;
        }
            #endif
        ++frame_counter;
    }// End of while (m_process) loop

    std::cout << _LOG_CONSOLE_BOLD_TEXT << "tracking off" << _NORMAL_CONSOLE_TEXT_ << std::endl;
}