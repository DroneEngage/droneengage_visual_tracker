#include <stdio.h>
#include <string.h>
#include <iostream>
#include <fstream>       
#include <iostream>      // For input/output operations (e.g., std::cout, std::cerr)
#include <fstream>       // For file stream operations (e.g., std::ifstream)
#include <filesystem>    // For directory iteration and path manipulation (C++17+)
#include <algorithm>     // For std::remove_if (used for trimming whitespace)
#include <stdexcept>     // For std::runtime_error



#include "../helpers/colors.hpp"

#include "video.hpp"

using namespace de::tracker;

namespace fs = std::filesystem; // Use a shorter alias for std::filesystem



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

    if (CVideo::xioctl(fd, VIDIOC_G_FMT, &fmt) == 0) {
        width = fmt.fmt.pix.width;
        height = fmt.fmt.pix.height;
        std::cout << _SUCCESS_CONSOLE_TEXT_ << "Queried V4L2 device " << _LOG_CONSOLE_BOLD_TEXT << video_device_path
                  << _SUCCESS_CONSOLE_TEXT_ << " Resolution: " << _LOG_CONSOLE_BOLD_TEXT << width 
                  << _INFO_CONSOLE_TEXT << "x" << _LOG_CONSOLE_BOLD_TEXT << height << _NORMAL_CONSOLE_TEXT_ << std::endl;
        close(fd);
        return true;
    } else {
        std::cout << _ERROR_CONSOLE_TEXT_ << "Error: Failed to get format for V4L2 device " << video_device_path << ": " << strerror(errno) << _NORMAL_CONSOLE_TEXT_ << std::endl;
        close(fd);
        return false;
    }   
}



// Function to trim leading and trailing whitespace from a string
std::string CVideo::trim(const std::string& str) {
    size_t first = str.find_first_not_of(" \t\n\r\f\v");
    if (std::string::npos == first) {
        return str; // No non-whitespace characters
    }
    size_t last = str.find_last_not_of(" \t\n\r\f\v");
    return str.substr(first, (last - first + 1));
}

/**
 * @brief Finds the index of a video device given its name.
 *
 * This function iterates through /sys/devices/virtual/video4linux/ directories,
 * reads the 'name' or 'card' file within each, and compares it to the
 * provided target device name.
 *
 * @param targetDeviceName The name of the video device to search for.
 * @return The integer index of the device (e.g., 0 for /dev/video0),
 * or -1 if the device is not found.
 */
int CVideo::findVideoDeviceIndex(const std::string& targetDeviceName) {
    std::cout << _LOG_CONSOLE_BOLD_TEXT << ".... Searching for virtual camera: " << targetDeviceName  << _NORMAL_CONSOLE_TEXT_ << std::endl;

    // Define the base path where video4linux devices are listed in sysfs
    fs::path sysfsPath("/sys/devices/virtual/video4linux/");

    // Check if the directory exists and is accessible
    if (!fs::exists(sysfsPath) || !fs::is_directory(sysfsPath)) {
        std::cerr << _ERROR_CONSOLE_BOLD_TEXT_ << "Error: " << sysfsPath << " does not exist or is not a directory." << _NORMAL_CONSOLE_TEXT_ << std::endl;
        return -1;
    }

    try {
        // Iterate through all entries in the video4linux directory
        for (const auto& entry : fs::directory_iterator(sysfsPath)) {
            // Check if the current entry is a directory and its name starts with "video"
            if (entry.is_directory() && entry.path().filename().string().rfind("video", 0) == 0) {
                std::string currentCardLabel;
                fs::path deviceDir = entry.path(); // Get the path to the videoX directory

                // Try to read 'name' file first (more common on recent kernels for v4l2loopback)
                fs::path nameFilePath = deviceDir / "name";
                std::ifstream nameFile(nameFilePath);
                if (nameFile.is_open()) {
                    std::getline(nameFile, currentCardLabel);
                    nameFile.close();
                } else {
                    // Fallback to 'card' file for older kernels
                    fs::path cardFilePath = deviceDir / "card";
                    std::ifstream cardFile(cardFilePath);
                    if (cardFile.is_open()) {
                        std::getline(cardFile, currentCardLabel);
                        cardFile.close();
                    }
                }

                // Debugging: Uncomment the line below to see all device labels being checked
                // std::cout << "Checking device " << deviceDir.filename().string()
                //           << " with label: '" << currentCardLabel << "'" << std::endl;

                // Check if the current device's label matches our target, ignoring leading/trailing whitespace
                if (trim(currentCardLabel) == trim(targetDeviceName)) {
                    // Extract the device number from the directory name (e.g., "video0" -> "0")
                    std::string deviceNumberStr = deviceDir.filename().string().substr(5); // Remove "video" prefix
                    try {
                        int deviceNumber = std::stoi(deviceNumberStr); // Convert string to integer
                        std::cout << _SUCCESS_CONSOLE_BOLD_TEXT_ << ".... Found target device: /dev/video" << deviceNumber  << _NORMAL_CONSOLE_TEXT_ << std::endl;
                        return deviceNumber; // Found our target, return the index
                    } catch (const std::invalid_argument& e) {
                        std::cerr << _ERROR_CONSOLE_BOLD_TEXT_ << "Error: Could not convert '" << deviceNumberStr << "' to integer: " << e.what() << _NORMAL_CONSOLE_TEXT_ << std::endl;
                    } catch (const std::out_of_range& e) {
                        std::cerr << _ERROR_CONSOLE_BOLD_TEXT_ << "Error: Device number '" << deviceNumberStr << "' out of range: " << e.what() << _NORMAL_CONSOLE_TEXT_ << std::endl;
                    }
                }
            }
        }
    } catch (const fs::filesystem_error& e) {
        // Catch filesystem-related errors
        std::cerr << "Filesystem error: " << e.what() << std::endl;
        return -1;
    }

    std::cout << "Virtual camera '" << targetDeviceName << "' not found." << std::endl;
    return -1; // Device not found
}