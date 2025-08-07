
/**
 * 
 * This class listens to UDP packets sent from postprocessor object detection class in rpicamera (DE-Version)
 * This is related to my work in [https://github.com/raspberrypi/rpicam-apps/pull/851]
 * 
 * Date: 07 Aug 2025
 */
#include <iostream>      // For input/output operations (e.g., std::cout, std::cerr)
#include <vector>        // Not directly used but often useful for collections
#include <sstream>       // Not directly used but useful for string parsing/formatting
#include <cstring>       // For memset, used for memory operations
#include <stdexcept>     // For std::runtime_error

#include "../helpers/colors.hpp"

#include "udp_de_object_detection_tracking.hpp"

using namespace de::tracker;


// Private Constructor: Initializes member variables.
// The socket is not created here. It will be created in the `init` method.
CUDP_AI_Receiver::CUDP_AI_Receiver() {
    std::cout << "CUDP_AI_Receiver singleton constructed." << std::endl;
}

// Public Destructor: Ensures cleanup is performed if the object is ever destroyed,
// though this should not happen during normal operation for a singleton.
CUDP_AI_Receiver::~CUDP_AI_Receiver() {
    uninit(); // Call uninit() to ensure resources are properly cleaned up.
    std::cout << "CUDP_AI_Receiver singleton destroyed." << std::endl;
}

// The `init` method performs the one-time setup.
void CUDP_AI_Receiver::init(const int port, ONRECIEVE callback) {
    std::lock_guard<std::mutex> lock(m_mutex);

    if (sockfd_ != -1) {
        std::cerr << "CUDP_AI_Receiver is already initialized." << std::endl;
        return;
    }

    port_ = port;
    m_callback = callback;
    m_exit = false;

    // Create a UDP socket.
    if ((sockfd_ = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
        perror("Failed to create socket");
        throw std::runtime_error("Failed to create socket.");
    }
    
    // Set up server address structure.
    memset(&servaddr_, 0, sizeof(servaddr_));
    servaddr_.sin_family = AF_INET;
    servaddr_.sin_addr.s_addr = INADDR_ANY;
    servaddr_.sin_port = htons(port_);

    // Bind the socket to the specified IP address and port.
    if (bind(sockfd_, (const struct sockaddr *)&servaddr_, sizeof(servaddr_)) < 0) {
        perror("Failed to bind socket");
        close(sockfd_);
        sockfd_ = -1;
        throw std::runtime_error("Failed to bind socket.");
    }

    std::cout << "Listening for UDP detection packets on port " << port_ << "..." << std::endl;

    // Start the receive thread.
    m_receiveThread = std::thread(&CUDP_AI_Receiver::run, this);

    std::cout << _LOG_CONSOLE_BOLD_TEXT << "Start UDP AI Receiver on port: " << _INFO_CONSOLE_BOLD_TEXT << port << _NORMAL_CONSOLE_TEXT_ << std::endl;
    
}

// The `uninit` method stops the thread and cleans up resources.
void CUDP_AI_Receiver::uninit() {
    std::lock_guard<std::mutex> lock(m_mutex);

    // If the socket is not open, the receiver is not initialized.
    if (sockfd_ == -1) {
        return;
    }

    std::cout << "Stopping CUDP_AI_Receiver..." << std::endl;

    m_exit = true; // Signal the thread to exit.

    // Close the socket. This will cause `recvfrom` to return with an error,
    // which will break the loop in the `run` method.
    close(sockfd_);
    
    // Wait for the thread to finish.
    if (m_receiveThread.joinable()) {
        m_receiveThread.join();
    }

    // Reset member variables to their initial state.
    sockfd_ = -1;
    port_ = -1;
    m_callback = nullptr;

    std::cout << "CUDP_AI_Receiver stopped and resources released." << std::endl;
}

// The main receiving loop, which runs in a separate thread.
void CUDP_AI_Receiver::run() {
    char buffer[MAX_BUFFER_SIZE];
    ParsedDetection detection;
    socklen_t len;

    while (!m_exit) {
        len = sizeof(clientaddr_);
        
        // Receive data from the socket.
        ssize_t n = recvfrom(sockfd_, (char*)buffer, MAX_BUFFER_SIZE, 0,
                             (struct sockaddr *)&clientaddr_, &len);

        // Check if `m_exit` has been set, which happens when `uninit` is called.
        // If the socket is closed, `recvfrom` will return an error, and we should check
        // the exit flag to distinguish between a graceful shutdown and an actual error.
        if (m_exit) {
            break; 
        }

        if (n < 0) {
            perror("recvfrom failed");
            continue; // Continue the loop to check `m_exit` again.
        }

        // If a valid callback is set and parsing is successful, invoke the callback.
        if (m_callback && parseDetection(buffer, n, detection)) {
            m_callback(detection);
        }
    }
}

// Helper method to parse the raw byte buffer into a ParsedDetection struct.
// @param buffer A pointer to the raw byte buffer containing the received data.
// @param size The number of bytes in the buffer.
// @param detection_data A reference to a ParsedDetection struct to populate.
// @return true if parsing was successful, false otherwise (e.g., malformed packet).
bool CUDP_AI_Receiver::parseDetection(const char* buffer, size_t size, ParsedDetection& detection_data) {
    // Define a start delimiter used to validate the beginning of a valid packet.
    const uint32_t START_DELIMITER = 0xDDCCBBAA;
    // Define the minimum expected size of a valid packet, excluding the name string length.
    const size_t MIN_PACKET_SIZE = sizeof(START_DELIMITER) + (sizeof(double) * 4) + sizeof(uint8_t) + sizeof(float);

    if (size < MIN_PACKET_SIZE) {
        std::cerr << "Packet too small to be a valid detection." << std::endl;
        return false;
    }

    if (*(uint32_t*)buffer != START_DELIMITER) {
        std::cerr << "Invalid packet: delimiter not found." << std::endl;
        return false;
    }

    size_t offset = sizeof(START_DELIMITER);

    // Read x, y, width, and height.
    detection_data.x = *(double*)(buffer + offset);
    offset += sizeof(double);
    detection_data.y = *(double*)(buffer + offset);
    offset += sizeof(double);
    detection_data.width = *(double*)(buffer + offset);
    offset += sizeof(double);
    detection_data.height = *(double*)(buffer + offset);
    offset += sizeof(double);

    // Read the length of the object's name.
    uint8_t name_length = *(uint8_t*)(buffer + offset);
    offset += sizeof(uint8_t);

    if (offset + name_length + sizeof(float) > size) {
        std::cerr << "Invalid packet: name length exceeds buffer size." << std::endl;
        return false;
    }

    // Extract the name string.
    detection_data.name.assign(buffer + offset, name_length);
    offset += name_length;

    detection_data.category = *(uint32_t*)(buffer + offset);
    offset += sizeof(uint32_t);

    // Read the confidence score.
    detection_data.confidence = *(float*)(buffer + offset);

    return true;
}