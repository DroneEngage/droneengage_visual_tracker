#include <opencv2/opencv.hpp>
#include <opencv2/tracking.hpp>
#include <opencv2/core/ocl.hpp>

#include "../helpers/colors.hpp"
#include "tracker.hpp"

// Headers for V4L2
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/videodev2.h>

using namespace de::tracker;

std::thread m_framesThread;

/**
 * @brief Queries the current resolution of a V4L2 video device.
 *
 * This function attempts to open the specified video device and use the
 * VIDIOC_G_FMT ioctl to retrieve its current width and height.
 *
 * @param video_device_path The path to the V4L2 video device (e.g., "/dev/video0").
 * @param width Reference to an unsigned int to store the retrieved width.
 * @param height Reference to an unsigned int to store the retrieved height.
 * @return true if the resolution was successfully retrieved, false otherwise.
 */
bool CTracker::getVideoResolution(const std::string& video_device_path, unsigned int& width, unsigned int& height)
{
    // Initialize output parameters
    width = 0;
    height = 0;

    // Check if the path looks like a V4L2 device
    if (video_device_path.rfind("/dev/video", 0) != 0) {
        std::cerr << "Error: " << video_device_path << " does not appear to be a V4L2 device path (does not start with /dev/video)." << std::endl;
        return false;
    }

    int fd = open(video_device_path.c_str(), O_RDONLY | O_NONBLOCK);
    if (fd < 0) {
        std::cerr << "Error: Failed to open V4L2 device " << video_device_path << ": " << strerror(errno) << std::endl;
        return false;
    }

    struct v4l2_format fmt = {0};
    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE; // We are querying a capture device

    if (ioctl(fd, VIDIOC_G_FMT, &fmt) == 0) {
        width = fmt.fmt.pix.width;
        height = fmt.fmt.pix.height;
        std::cout << "Successfully queried V4L2 device " << video_device_path
                  << ". Resolution: " << width << "x" << height << std::endl;
        close(fd);
        return true;
    } else {
        std::cerr << "Error: Failed to get format for V4L2 device " << video_device_path << ": " << strerror(errno) << std::endl;
        close(fd);
        return false;
    }
}


bool CTracker::initTargetVirtualVideoDevice(const std::string &target_video_device)
{
    m_virtual_device_opened = false;

    if (target_video_device != std::string(""))
    {
        m_target_video_path = target_video_device;
        m_target_video_active = true;
    }
    else
    {
        m_target_video_active = false;
        // not an error
        return true;
    }
    
    cv::Mat frame;
    video_capture_cap >> frame;
    cv::Mat yuv_frame;
    cv::cvtColor(frame, yuv_frame, cv::COLOR_BGR2YUV_I420);

    // Open the virtual video device
    m_video_fd = open(m_target_video_path.c_str(), O_WRONLY | O_NONBLOCK);
    if (m_video_fd < 0)
    {
        std::cout << "Error: Could not open virtual video device " << m_target_video_path << ": " << strerror(errno) << std::endl;
        return false;
    }

    std::cout << "Successfully opened virtual video device: " << m_target_video_path << std::endl;

    struct v4l2_format fmt = {0};
    fmt.type = V4L2_BUF_TYPE_VIDEO_OUTPUT; // We are outputting frames to this device
    fmt.fmt.pix.width = m_image_width;
    fmt.fmt.pix.height = m_image_height;
    fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_YUV420; // YUV420 planar (I420)
    fmt.fmt.pix.field = V4L2_FIELD_NONE;           // Progressive scan
    // For YUV420, sizeimage = width * height * 3 / 2
    // bytesperline for Y plane is width. For U and V planes, it's width / 2.
    // Some drivers might calculate this automatically if set to 0, or expect it.
    fmt.fmt.pix.bytesperline = m_image_width; // Stride of the Y plane
    fmt.fmt.pix.sizeimage = (m_image_width * m_image_height * 3) / 2;

    if (ioctl(m_video_fd, VIDIOC_S_FMT, &fmt) < 0)
    {
        fprintf(stderr, "Failed to set video format on %s: %s\n", m_target_video_path.c_str(), strerror(errno));
        close(m_video_fd);
        video_capture_cap.release(); // Release the input video capture
        return 1;
    }
    fprintf(stderr, "Successfully set format for %s: %dx%d, pixformat YUV420\n",
            m_target_video_path.c_str(), fmt.fmt.pix.width, fmt.fmt.pix.height);
    // Store the calculated frame size for later writing

    // Set the YUV frame size based on the expected resolution
    m_yuv_frame_size = m_image_width * m_image_height * 3 / 2; // YUV420 format

    m_virtual_device_opened = true;
    return true;
};

bool CTracker::init(const enum ENUM_TRACKER_TYPE tracker_type, const std::string &video_path, const std::string &target_video_device)
{

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

    if (getVideoResolution(m_video_path, detected_width, detected_height))
    {
        m_image_width = detected_width;
        m_image_height = detected_height;
    }

    video_capture_cap.open(video_path);
    if (!video_capture_cap.isOpened())
    {
        // TODO: send error message to WebClient please.

        std::cout << _ERROR_CONSOLE_BOLD_TEXT_ << "FATAL ERROR:" << _INFO_CONSOLE_TEXT << " could not open camera at " << _ERROR_CONSOLE_TEXT_ << video_path << _NORMAL_CONSOLE_TEXT_ << std::endl;

        return false;
    }
    else
    {
        std::cout << _SUCCESS_CONSOLE_BOLD_TEXT_ << "Video Capture:" << _INFO_CONSOLE_TEXT << video_path << _NORMAL_CONSOLE_TEXT_ << std::endl;
    }

    // Read the first frame to get its actual width and height
    
    video_capture_cap.set(cv::CAP_PROP_POS_FRAMES, 0); // Rewind to start of video for actual tracking

    video_capture_cap.set(
        cv::CAP_PROP_FRAME_WIDTH,
        m_image_width);

    video_capture_cap.set(
        cv::CAP_PROP_FRAME_HEIGHT,
        m_image_height);

    video_capture_cap.set(
        cv::CAP_PROP_FPS,
        m_image_fps);

    // --- V4L2 Output Device Initialization ---
    if (!initTargetVirtualVideoDevice(target_video_device))
    {
        video_capture_cap.release();
        return false;
    }

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
    if (m_callback_tracker != nullptr)
    {
        m_callback_tracker->onTrackStatusChanged(false);
    }

    if (m_framesThread.joinable())
        m_framesThread.join();
}

void CTracker::track(const float x, const float y, const float radius, const bool display)
{
    m_valid_track = false;

    std::cout << "X,y,r:" << std::to_string(x) << "," << std::to_string(y) << "," << std::to_string(radius) << _NORMAL_CONSOLE_TEXT_ << std::endl;

    if (m_video_path == std::string(""))
    {
        // TODO: send error message.
        return;
    }

    m_framesThread = std::thread([x, y, radius, display, this]()
                                 { this->track2(x, y, radius, display); });
}

void CTracker::track2(const float x, const float y, const float radius, const bool display)
{

    cv::Mat frame;

    if (!video_capture_cap.isOpened())
    {
        // TODO: send error message to WebClient please.

        std::cout << _ERROR_CONSOLE_BOLD_TEXT_ << "FATAL ERROR:" << _INFO_CONSOLE_TEXT << " could not open camera at " << _ERROR_CONSOLE_TEXT_ << m_video_path << _NORMAL_CONSOLE_TEXT_ << std::endl;

        return;
    }

    video_capture_cap >> frame;
    m_image_width = frame.cols;
    m_image_height = frame.rows;

#ifdef DDEBUG
    std::cout << _LOG_CONSOLE_BOLD_TEXT << "frame: " << _INFO_CONSOLE_TEXT << m_image_width << "x" << m_image_height << _NORMAL_CONSOLE_TEXT_ << std::endl;
#endif

    // Define initial bounding box
    float scaled_x = x * m_image_width;
    float scaled_y = y * m_image_height;

    if (scaled_x < 0)
    {
        scaled_x = 0;
    }
    else if (scaled_x + radius > m_image_width)
    {
        scaled_x = m_image_width - radius;
    }

    if (scaled_y < 0)
    {
        scaled_y = 0;
    }
    else if (scaled_y + radius > m_image_height)
    {
        scaled_y = m_image_height - radius;
    }

#ifdef DDEBUG
    std::cout << "scaled_x,scaled_y:" << std::to_string(scaled_x) << ":" << std::to_string(scaled_y) << std::endl;
#endif

    cv::Rect2d bbox_2d(scaled_x, scaled_y, radius, radius);
    cv::Rect bbox(scaled_x, scaled_y, radius, radius);

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
                if (m_callback_tracker != nullptr)
                    m_callback_tracker->onTrackStatusChanged(m_valid_track);
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
                if (m_callback_tracker != nullptr)
                    m_callback_tracker->onTrack(revScaleX(bbox_2d.x), revScaleY(bbox_2d.y), revScaleX(bbox_2d.width), revScaleY(bbox_2d.height));

                if (display || m_target_video_active)
                    cv::rectangle(frame, bbox_2d, cv::Scalar(0, 255, 255), 2, 1);
#ifdef DDEBUG
                std::cout << "Tracking at " << bbox_2d << std::endl;
#endif
            }
            else
            {
                if (m_callback_tracker != nullptr)
                    m_callback_tracker->onTrack(revScaleX(bbox.x), revScaleY(bbox.y), revScaleX(bbox.width), revScaleY(bbox.height));

#ifdef DDEBUG
                std::cout << "Tracking at " << std::to_string(bbox.x) << "  --    " << std::to_string(bbox.y) << " xxx "
                          << std::to_string(bbox.width) << "  --    " << std::to_string(bbox.height) << std::endl;
#endif
                if (display || m_target_video_active)
                    cv::rectangle(frame, bbox, cv::Scalar(0, 255, 255), 2, 1);
            }
        }
        else
        {
            if (m_callback_tracker != nullptr)
                m_callback_tracker->onTrackStatusChanged(false);

            // Tracking failure detected.
            if (display || m_target_video_active)
            {
                cv::putText(frame, "Tracking failure detected", cv::Point(100, 80), cv::FONT_HERSHEY_SIMPLEX, 0.75, cv::Scalar(0, 0, 255), 2);
            }
        }

        // --- Start of new code for streaming to virtual video device ---
        if (m_video_fd != -1)
        { // Check if virtual device is open
            cv::Mat yuv_frame;

            // Convert BGR frame (from OpenCV) to YUYV format (for V4L2 device)
            // Ensure the frame dimensions match what was set for the V4L2 device.
            // If they don't, you might need to resize 'frame' before conversion.
            // For simplicity, we assume consistency based on init() setup.
            cv::cvtColor(frame, yuv_frame, cv::COLOR_BGR2YUV_I420);

            

            // Check if the YUV frame is continuous and has the expected size
            if (yuv_frame.isContinuous()) // && yuv_frame.total() * yuv_frame.elemSize() == m_yuv_frame_size)
            {
                ssize_t bytes_written = write(m_video_fd, yuv_frame.data, m_yuv_frame_size);
                if (bytes_written < 0)
                {
                    std::cout << "Error: Failed to write frame to " << m_target_video_path << ": " << strerror(errno) << std::endl;
                    // EAGAIN or EWOULDBLOCK might occur if O_NONBLOCK is used and buffer is full.
                    if (errno == EAGAIN || errno == EWOULDBLOCK)
                    {
                        std::cout << "Warning: Virtual device " << m_target_video_path << " buffer full? Try reading from it." << std::endl;
                        // continue; // Optionally, skip frame and try next, or implement a delay
                    }
                    // For other errors, it's likely more serious, consider breaking the loop or attempting recovery.
                    // break; // Uncomment to stop on write error
                }
                else if (static_cast<size_t>(bytes_written) != m_yuv_frame_size)
                {
                    std::cout << "Warning: Incomplete frame write to " << m_target_video_path << " (wrote " << bytes_written << " of " << m_yuv_frame_size << " bytes)" << std::endl;
                }
            }
            else
            {
                std::cout << "Error: YUV frame is not continuous or has unexpected size. Cannot write to V4L2 device." << std::endl;
            }
        }
        // --- End of new code for streaming to virtual video device ---

        if (display)
        {
            // Display frame.
            cv::imshow("Tracking", frame);
            cv::waitKey(1);
        }
    }

    std::cout << _LOG_CONSOLE_BOLD_TEXT << "tracking off" << _NORMAL_CONSOLE_TEXT_ << std::endl;
}