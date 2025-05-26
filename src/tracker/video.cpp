#include <stdio.h>
#include <string.h>
#include <iostream>

// Headers for V4L2
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/videodev2.h>

#include "../helpers/colors.hpp"

#include "video.hpp"

using namespace de::tracker;

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
bool CVideo::getVideoResolution (const std::string& video_device_path, unsigned int& width, unsigned int& height)
{
    // Initialize output parameters
    width = 0;
    height = 0;

    // Check if the path looks like a V4L2 device
    if (video_device_path.rfind("/dev/video", 0) != 0) {
        std::cout << _ERROR_CONSOLE_BOLD_TEXT_ "Error: " << _ERROR_CONSOLE_BOLD_TEXT_ << video_device_path << _NORMAL_CONSOLE_TEXT_ << " does not appear to be a V4L2 device path (does not start with /dev/video)." << _NORMAL_CONSOLE_TEXT_ << std::endl;
        return false;
    }

    int fd = open(video_device_path.c_str(), O_RDONLY | O_NONBLOCK);
    if (fd < 0) {
        std::cout << _ERROR_CONSOLE_TEXT_ << "Error: Failed to open V4L2 device " << _ERROR_CONSOLE_BOLD_TEXT_ << video_device_path << _NORMAL_CONSOLE_TEXT_ <<  ": " << strerror(errno) << _NORMAL_CONSOLE_TEXT_ << std::endl;
        return false;
    }

    struct v4l2_format fmt = {0};
    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE; // We are querying a capture device

    if (ioctl(fd, VIDIOC_G_FMT, &fmt) == 0) {
        width = fmt.fmt.pix.width;
        height = fmt.fmt.pix.height;
        std::cout << _SUCCESS_CONSOLE_TEXT_ << "Queried V4L2 device " << _LOG_CONSOLE_BOLD_TEXT << video_device_path
                  << _INFO_CONSOLE_TEXT << " Resolution: " << _LOG_CONSOLE_BOLD_TEXT << width 
                  << _INFO_CONSOLE_TEXT << "x" << _LOG_CONSOLE_BOLD_TEXT << height << _NORMAL_CONSOLE_TEXT_ << std::endl;
        close(fd);
        return true;
    } else {
        std::cout << _ERROR_CONSOLE_TEXT_ << "Error: Failed to get format for V4L2 device " << video_device_path << ": " << strerror(errno) << _NORMAL_CONSOLE_TEXT_ << std::endl;
        close(fd);
        return false;
    }   
}