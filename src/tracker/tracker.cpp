#include <errno.h>
#include <chrono> // For high-resolution timing
#include <thread> // For std::this_thread::sleep_for
#include <algorithm> // For std::clamp
#include <sys/mman.h>  // For mmap, munmap, PROT_READ, PROT_WRITE, MAP_SHARED, MAP_FAILED
#include <opencv2/opencv.hpp>
#include <opencv2/tracking.hpp>
#include <opencv2/core/ocl.hpp>

#include "../de_common/helpers/colors.hpp"
#include "../de_common/helpers/helpers.hpp"
#include "video.hpp"
#include "tracker.hpp"

// Headers for V4L2
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/videodev2.h>
#include "../de_common/de_databus/messages.hpp"
using namespace de::tracker;





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

    std::cout << _SUCCESS_CONSOLE_BOLD_TEXT_<< "Successfully opened virtual video device: " << _LOG_CONSOLE_BOLD_TEXT << m_output_video_path << _NORMAL_CONSOLE_TEXT_ <<  std::endl;

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
        std::cerr << _ERROR_CONSOLE_BOLD_TEXT_ << "Failed to set video format on " << _INFO_CONSOLE_BOLD_TEXT << m_output_video_path << " " << strerror(errno) << _NORMAL_CONSOLE_TEXT_ << std::endl;
        close(m_video_fd);
        return false;
    }

    std::cout << _SUCCESS_CONSOLE_BOLD_TEXT_<< "Successfully set format for " << m_output_video_path << ":" << _INFO_CONSOLE_BOLD_TEXT << fmt.fmt.pix.width << _LOG_CONSOLE_BOLD_TEXT << "x" << _INFO_CONSOLE_BOLD_TEXT << fmt.fmt.pix.height << _LOG_CONSOLE_BOLD_TEXT << " pixformat YUV420" << _NORMAL_CONSOLE_TEXT_ <<  std::endl;


    // We are using the simple non-blocking write() path for streaming.
    // No need to request or mmap V4L2 buffers in this mode.

    // Compute YUV frame size for I420
    m_yuv_frame_size = m_image_width * m_image_height * 3 / 2;

    m_virtual_device_opened = true;
    return true;
}

bool CTracker::init(const enum ENUM_TRACKER_TYPE tracker_type, const std::string& video_path
    , const uint16_t camera_orientation, const bool camera_forward, const std::string& output_video_device
    , uint16_t frames_to_skip_between_messages, uint16_t frame_to_skip_between_track_process
    , int desired_input_width, int desired_input_height)
{

    // Enable OpenCV OpenCL acceleration if available for operations like cvtColor
    cv::ocl::setUseOpenCL(true);

    if (frames_to_skip_between_messages < frame_to_skip_between_track_process)
    {
        std::cout << _ERROR_CONSOLE_BOLD_TEXT_ << "FATAL ERROR:" << _INFO_CONSOLE_TEXT << " frames_to_skip_between_messages should be larger than frame_to_skip_between_track_process. " <<  _NORMAL_CONSOLE_TEXT_ << std::endl;
        return false;
    }

    m_frames_to_skip_between_messages = frames_to_skip_between_messages;
    m_frame_to_skip_between_track_process = frame_to_skip_between_track_process;

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

    video_capture.open(video_path.c_str(), cv::CAP_V4L2);
    if (!video_capture.isOpened())
    {
        // TODO: send error message to WebClient please.

        std::cout << _ERROR_CONSOLE_BOLD_TEXT_ << "FATAL ERROR:" << _BK_RED_WHITE_TEXT_ << " could not open camera at " << _ERROR_CONSOLE_TEXT_ << video_path << _NORMAL_CONSOLE_TEXT_ << std::endl;

        return false;
    }

    // Set camera properties once after opening.
    // Apply desired resolution if provided (>0), then FPS and FOURCC.
    if (desired_input_width > 0 && desired_input_height > 0)
    {
        video_capture.set(cv::CAP_PROP_FRAME_WIDTH, desired_input_width);
        video_capture.set(cv::CAP_PROP_FRAME_HEIGHT, desired_input_height);
    }
    video_capture.set(cv::CAP_PROP_FPS, 30);
    video_capture.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));
    
    m_image_width = static_cast<int>(video_capture.get(cv::CAP_PROP_FRAME_WIDTH));
    m_image_height = static_cast<int>(video_capture.get(cv::CAP_PROP_FRAME_HEIGHT));
    double actual_fps = video_capture.get(cv::CAP_PROP_FPS);
    
    if (m_image_width == 0 || m_image_height == 0) {
        std::cerr << _ERROR_CONSOLE_BOLD_TEXT_ << "Error: Failed to get camera resolution from " << video_path << _NORMAL_CONSOLE_TEXT_ << std::endl;
        video_capture.release();
        return false;
    }

    
    std::cout << _SUCCESS_CONSOLE_BOLD_TEXT_ << "-------------------------------------------" << _NORMAL_CONSOLE_TEXT_ << std::endl;

    std::cout << _SUCCESS_CONSOLE_BOLD_TEXT_ << "Video Capture:" << _LOG_CONSOLE_BOLD_TEXT << video_path << _NORMAL_CONSOLE_TEXT_ << std::endl;
   
    // Ensure FOURCC is set (some drivers require set after open and before reads)
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

    // Release input capture safely
    if (video_capture.isOpened())
    {
        video_capture.release();
    }

    // Close and cleanup virtual output device if opened
    if (m_virtual_device_opened || m_video_fd != -1 || m_buffers != nullptr)
    {
        destroyVirtualVideoDevice();
    }

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

        if (!video_capture.read(frame) || frame.empty())
        {
            std::cerr << _ERROR_CONSOLE_BOLD_TEXT_ << "WARNING:" << _INFO_CONSOLE_TEXT << " Captured an empty frame. Stopping tracking."
                      << _NORMAL_CONSOLE_TEXT_ << std::endl;
            if (m_callback_tracker) m_callback_tracker->onTrackStatusChanged(TrackingTarget_STATUS_TRACKING_STOPPED);

            std::this_thread::sleep_for(std::chrono::milliseconds(200));

            continue;
        }
        
        // --- Tracking Logic ---
        if (m_is_tracking_active_initial)// Use the initial tracking state
        {
            const bool should_skip_track_process = (frame_counter % m_frame_to_skip_between_track_process) != 0;

            // --- REFINED: Update tracking status only when processed ---
            if (!should_skip_track_process) {
                
                track_success = m_islegacy ? m_legacy_tracker->update(frame, bbox_2d) : m_tracker->update(frame, bbox);
                m_valid_track = track_success;
                
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
                if (m_islegacy) cv::rectangle(frame, bbox_2d, cv::Scalar(0, 0, 200), 2, 1);
                else cv::rectangle(frame, bbox, cv::Scalar(0, 0, 200), 2, 1);
                
                const bool should_skip_message = (frame_counter % m_frames_to_skip_between_messages) != 0;
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
                m_valid_track = false;
            }
             // Always draw crosshair if tracking was started
            cv::line(frame, cv::Point(center_x - cross_arm_length, center_y), cv::Point(center_x + cross_arm_length, center_y), cv::Scalar(0, 255, 0), 2);
            cv::line(frame, cv::Point(center_x, center_y - cross_arm_length), cv::Point(center_x, center_y + cross_arm_length), cv::Scalar(0, 255, 0), 2);
        }

                    

        // --- OPTIMIZATION: Use V4L2 buffer queuing instead of write() --- (NOT WORKING)
        // if (m_virtual_device_opened)
        // {
        //     cv::cvtColor(frame, yuv_frame, cv::COLOR_BGR2YUV_I420);
            
        //     struct v4l2_buffer buf = {};
        //     buf.type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
        //     buf.memory = V4L2_MEMORY_MMAP;

        //     // Dequeue a buffer (non-blocking)
        //     if (CVideo::xioctl(m_video_fd, VIDIOC_DQBUF, &buf) == 0)
        //     {
        //         // Copy frame data into the dequeued buffer
        //         memcpy(m_buffers[buf.index].start, yuv_frame.data, m_yuv_frame_size);
                
        //         // Queue the buffer back to the driver
        //         if (CVideo::xioctl(m_video_fd, VIDIOC_QBUF, &buf) < 0) {
        //             perror("VIDIOC_QBUF");
        //             m_process = false; // Stop on critical error
        //         }
        //     }
        //     else if (errno != EAGAIN) {
        //         perror("VIDIOC_DQBUF");
        //         m_process = false; // Stop on critical error
        //     }
        //     // If errno is EAGAIN, it means the output queue is full. We simply drop the frame,
        //     // which is the desired behavior to prevent the pipeline from blocking.
        // }

        if (m_virtual_device_opened)
        {
             cv::cvtColor(frame, yuv_frame, cv::COLOR_BGR2YUV_I420);
             // Check continuity once after conversion. Size check is critical.
            if (yuv_frame.isContinuous() && yuv_frame.total() * yuv_frame.elemSize() == m_yuv_frame_size)
            {
                ssize_t bytes_written = write(m_video_fd, yuv_frame.data, m_yuv_frame_size);
                if (bytes_written < 0)
                {
                    std::cout << "Error: Failed to write frame to " << m_output_video_path << ": " << strerror(errno) << " (errno: " << errno << ")" << std::endl;

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

        auto end_time = std::chrono::high_resolution_clock::now();
        auto elapsed_time = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
        
        if (elapsed_time < target_frame_time_ms)
        {
            auto sleep_for = (target_frame_time_ms - elapsed_time);
            std::this_thread::sleep_for(sleep_for);
            #ifdef DDEBUG
                        std::cout << "Elapsed: " << elapsed_time.count() << "ms, Sleeping for: " << sleep_for.count() << "ms" << std::endl;
            #endif
        }
        #ifdef DDEBUG
        else
        {
            std::cout << _INFO_CONSOLE_BOLD_TEXT << "Warning: Frame processing took " << _LOG_CONSOLE_BOLD_TEXT << elapsed_time.count() << "ms " << _INFO_CONSOLE_BOLD_TEXT << ", exceeding target "
                    << _LOG_CONSOLE_BOLD_TEXT << target_frame_time_ms.count() << "ms." << _INFO_CONSOLE_BOLD_TEXT << " Cannot maintain 30 FPS." << _NORMAL_CONSOLE_TEXT_ << std::endl;
        }
        #endif

        #ifdef DDEBUG
        if (frame_counter % 10 == 0)
        {
            std::cout << _INFO_CONSOLE_BOLD_TEXT << "Elapsed: " << _LOG_CONSOLE_BOLD_TEXT << elapsed_time.count() << "ms" << std::endl;
        }
        #endif
        
        ++frame_counter;
    }// End of while (m_process) loop

    std::cout << _LOG_CONSOLE_BOLD_TEXT << "tracking off" << _NORMAL_CONSOLE_TEXT_ << std::endl;
}