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
    if (m_video_fd < 0)
    {
        std::cout << "Error: Could not open virtual video device " << m_output_video_path << ": " << strerror(errno) << std::endl;
        return false;
    }

    std::cout << "Successfully opened virtual video device: " << m_output_video_path << std::endl;

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

bool CTracker::init(const enum ENUM_TRACKER_TYPE tracker_type, const std::string& video_path, const uint16_t camera_orientation, const bool camera_forward, const std::string& output_video_device)
{

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
    else
    {
        std::cout << _SUCCESS_CONSOLE_BOLD_TEXT_ << "Video Capture:" << _INFO_CONSOLE_TEXT << video_path << _NORMAL_CONSOLE_TEXT_ << std::endl;
    }

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

    std::cout << "X,y,r:" << std::to_string(x) << "," << std::to_string(y) << "," << std::to_string(w) << "," << std::to_string(h) << _NORMAL_CONSOLE_TEXT_ << std::endl;

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

    std::cout << "X,y,r:" << std::to_string(x) << "," << std::to_string(y) << "," << std::to_string(radius) << _NORMAL_CONSOLE_TEXT_ << std::endl;

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
    cv::Mat yuv_frame; // Declare outside the loop

    // Only declare these if they are truly needed within the loop or outside.
    // If bbox_2d and bbox are only used for initialization and passed by reference
    // to update, their creation here is fine.
    cv::Rect2d bbox_2d;
    cv::Rect bbox;


    // --- Frame Rate Control Variables ---
    const std::chrono::milliseconds target_frame_time_ms(1000 / m_target_fps); 
    

    // --- Initial Setup and Validation ---
    if (!video_capture.isOpened())
    {
        // TODO: send error message to WebClient please.
        std::cerr << _ERROR_CONSOLE_BOLD_TEXT_ << "FATAL ERROR:" << _INFO_CONSOLE_TEXT << " could not open camera at "
                  << _ERROR_CONSOLE_TEXT_ << m_video_path << _NORMAL_CONSOLE_TEXT_ << std::endl;
        // Consider a callback for fatal errors if a WebClient is truly involved
        // if (m_callback_tracker) m_callback_tracker->onFatalError("Could not open camera");
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

    m_image_width = frame.cols;
    m_image_height = frame.rows;

#ifdef DDEBUG
    std::cout << _LOG_CONSOLE_BOLD_TEXT << "frame: " << _INFO_CONSOLE_TEXT << m_image_width << "x" << m_image_height << _NORMAL_CONSOLE_TEXT_ << std::endl;
#endif

    // Define initial bounding box - calculations only needed once
    float scaled_x = x * m_image_width;
    float scaled_y = y * m_image_height;
    float scaled_width = w * m_image_width;
    float scaled_height = h * m_image_height;
    // Simplify boolean check
    m_is_tracking_active_initial = (x > 0);

    // Clamp coordinates within frame boundaries using std::clamp for conciseness and safety
    // Assuming radius is the side length, so bbox_x + radius should be <= width
    scaled_x = std::clamp(scaled_x, 0.0f, static_cast<float>(m_image_width) - scaled_width);
    scaled_y = std::clamp(scaled_y, 0.0f, static_cast<float>(m_image_height) - scaled_height);

    std::cout << "scaled_x:" << scaled_x << " scaled_y:" << scaled_y << "scaled_h:" << scaled_height << "scaled_w:" << scaled_width << std::endl;

#ifdef DDEBUG
    std::cout << "scaled_x,scaled_y:" << scaled_x << ":" << scaled_y << _NORMAL_CONSOLE_TEXT_ << std::endl;
#endif

    // Initialize bounding boxes for the tracker
    bbox_2d = cv::Rect2d(scaled_x, scaled_y, scaled_width, scaled_height);
    bbox = cv::Rect(static_cast<int>(scaled_x), static_cast<int>(scaled_y), static_cast<int>(scaled_width), static_cast<int>(scaled_height)); // Cast to int for cv::Rect


    if (m_is_tracking_active_initial)
    {
        if (m_islegacy)
        {
            m_legacy_tracker->init(frame, bbox_2d);
        }
        else
        {
            m_tracker->init(frame, bbox);
        }
    }
    m_process = true; // Flag to control the tracking loop

    // Cache callback pointer to avoid redundant nullptr checks inside the loop if it's frequent
    // Note: This assumes m_callback_tracker doesn't change during the loop.
    auto *callback_tracker = m_callback_tracker;
    bool current_valid_track_status = false; // Initialize to false, assuming no track initially


    // --- Main Tracking and Streaming Loop ---
    while (m_process)
    {
        // Start measuring time for the current frame
        auto start_time = std::chrono::high_resolution_clock::now();

        // Capture a new frame
        video_capture >> frame;
        if (frame.empty())
        {
            // If frame is empty, it means the video stream has ended or there's an issue.
            // Consider breaking the loop instead of continuing indefinitely.
            std::cerr << _ERROR_CONSOLE_BOLD_TEXT_ << "WARNING:" << _INFO_CONSOLE_TEXT << " Captured an empty frame. Stopping tracking."
                      << _NORMAL_CONSOLE_TEXT_ << std::endl;
            m_process = false; // Stop the loop
            continue; // Skip the rest of the current iteration
        }

        // --- Tracking Logic ---
        if (m_is_tracking_active_initial) // Use the initial tracking state
        {
            bool new_valid_track;
            if (m_islegacy)
            {
                new_valid_track = m_legacy_tracker->update(frame, bbox_2d);
            }
            else
            {
                new_valid_track = m_tracker->update(frame, bbox);
            }

            if (new_valid_track != current_valid_track_status)
            {
                current_valid_track_status = new_valid_track; // Update internal state
                if (callback_tracker) // Use cached pointer
                    callback_tracker->onTrackStatusChanged(TrackingTarget_STATUS_TRACKING_DETECTED);
            }

            if (current_valid_track_status)
            {
                // Tracking success: Draw the tracked object and call callback
                if (m_islegacy)
                {
                    if (callback_tracker)
                        callback_tracker->onTrack(revScaleX(bbox_2d.x), revScaleY(bbox_2d.y),
                                                   revScaleX(bbox_2d.width), revScaleY(bbox_2d.height),
                                                   m_camera_orientation, m_camera_forward);
                    cv::rectangle(frame, bbox_2d, cv::Scalar(0, 255, 255), 2, 1);
#ifdef DDEBUG
                    std::cout << "Tracking_legacy at " << bbox_2d << std::endl;
#endif
                }
                else
                {
                    if (callback_tracker)
                        callback_tracker->onTrack(revScaleX(bbox.x), revScaleY(bbox.y),
                                                   revScaleX(bbox.width), revScaleY(bbox.height),
                                                   m_camera_orientation, m_camera_forward);
#ifdef DDEBUG
                    std::cout << "Tracking at " << bbox.x << " revX:" << revScaleX(bbox.x) << "  --    " << bbox.y << " revY:" << revScaleY(bbox.y) << " xxx "
                              << bbox.width << "  --    " << bbox.height << std::endl; // Directly print ints
#endif
                    cv::rectangle(frame, bbox, cv::Scalar(0, 255, 255), 2, 1);
                }
            }
            else
            {
                // Tracking failure detected. Only draw text if it's truly failed
                cv::putText(frame, "Tracking failure detected", cv::Point(100, 80), cv::FONT_HERSHEY_SIMPLEX, 0.75, cv::Scalar(0, 0, 255), 2);
            }
        }
        else 
        {
            // If not tracking, ensure status is consistently false and update only if it changes
            if (current_valid_track_status != false) // Only call callback if status changes
            {
                current_valid_track_status = false;
                if (callback_tracker)
                    callback_tracker->onTrackStatusChanged(TrackingTarget_STATUS_TRACKING_LOST);
            }
        }

        // --- Streaming to Virtual Video Device ---
        if (m_video_fd != -1)
        {
            cv::cvtColor(frame, yuv_frame, cv::COLOR_BGR2YUV_I420); // Or COLOR_BGR2YUYV if that's what m_yuv_frame_size implies

            // Check continuity once after conversion. Size check is critical.
            if (yuv_frame.isContinuous() && yuv_frame.total() * yuv_frame.elemSize() == m_yuv_frame_size)
            {
                ssize_t bytes_written = write(m_video_fd, yuv_frame.data, m_yuv_frame_size);
                if (bytes_written < 0)
                {
                    std::cout << "Error: Failed to write frame to " << m_output_video_path << ": " << strerror(errno) << std::endl;
                    if (errno == EAGAIN || errno == EWOULDBLOCK)
                    {
                        // This warning is fine to output, but don't stop the loop.
                        // Consider adding a counter for repeated warnings to avoid spamming console.
                        std::cerr << "Warning: Virtual device " << m_output_video_path << " buffer full? Try reading from it." << std::endl;
                        // No `continue` here, we still want to process the frame and display if needed.
                        // The next write might succeed if the reader catches up.
                    }
                    // For other severe errors, consider setting m_process = false;
                }
            }
            else
            {
                // This indicates a critical error in setup or understanding of m_yuv_frame_size/conversion.
                std::cerr << "Fatal Error: YUV frame is not continuous or has unexpected size ("
                          << yuv_frame.total() * yuv_frame.elemSize() << " vs " << m_yuv_frame_size
                          << "). Cannot write to V4L2 device. Stopping stream." << std::endl;
                // Consider setting m_process = false; or closing m_video_fd
            }
        }

        // --- Frame Rate Control Implementation ---
        auto end_time = std::chrono::high_resolution_clock::now();
        auto elapsed_time = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);

        if (elapsed_time < target_frame_time_ms)
        {
            auto time_to_sleep = target_frame_time_ms - elapsed_time;
            #ifdef DDEBUG
                        std::cout << "Elapsed: " << elapsed_time.count() << "ms, Sleeping for: " << time_to_sleep.count() << "ms" << std::endl;
            #endif
                        std::this_thread::sleep_for(time_to_sleep);
                    }
            #ifdef DDEBUG
                    else
                    {
                        std::cout << _INFO_CONSOLE_BOLD_TEXT << "Warning: Frame processing took " << _LOG_CONSOLE_BOLD_TEXT << elapsed_time.count() << "ms " << _INFO_CONSOLE_BOLD_TEXT << ", exceeding target "
                                << _LOG_CONSOLE_BOLD_TEXT << target_frame_time_ms.count() << "ms." << _INFO_CONSOLE_BOLD_TEXT << " Cannot maintain 30 FPS." << _NORMAL_CONSOLE_TEXT_ << std::endl;
                    }
            #endif

        } // End of while (m_process) loop

    std::cout << _LOG_CONSOLE_BOLD_TEXT << "tracking off" << _NORMAL_CONSOLE_TEXT_ << std::endl;
}