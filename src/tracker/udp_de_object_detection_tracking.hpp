#ifndef UDP_DE_OBJECT_DETECTIONTRACKING_H
#define UDP_DE_OBJECT_DETECTIONTRACKING_H

// Required for networking functionalities
#include <sys/socket.h>  
#include <netinet/in.h>  
#include <unistd.h>      
#include <arpa/inet.h>   


#include <string>    
#include <atomic>    
#include <thread>    
#include <mutex>     
#include <condition_variable> 
#include <functional> 


// --- Data Structures ---

// A struct to hold the parsed detection data received over UDP.
// This structure defines the format of the object detection information.
struct ParsedDetection {
    double x, y;          // Top-left corner coordinates of the detected object's bounding box
    double width, height; // Dimensions of the detected object's bounding box
    std::string name;     // The name or label of the detected object (e.g., "person", "car")
    uint32_t category;    // object class-id
    float confidence;     // The confidence score of the detection (0.0 to 1.0)
};

// Use std::function to allow any callable object (including lambdas).
using ONRECIEVE = std::function<void(ParsedDetection)>;

namespace de
{
namespace tracker
{
// The CUDP_AI_Receiver class handles the establishment of a UDP socket,
// receiving incoming data, and parsing it into the ParsedDetection format.
class CUDP_AI_Receiver {
    public:
        // Public method to get the singleton instance.
        static CUDP_AI_Receiver& getInstance()
        {
            static CUDP_AI_Receiver instance;
            return instance;
        }

        // Delete copy constructor and assignment operator to enforce singleton pattern.
        CUDP_AI_Receiver(CUDP_AI_Receiver const&)         = delete;
        void operator=(CUDP_AI_Receiver const&)          = delete;

    
    public:
        // Public destructor.
        ~CUDP_AI_Receiver ();

        // Initializes the UDP receiver, sets the port, and starts the receive thread.
        void init(const int port, ONRECIEVE callback);
        
        // Stops the receive thread and cleans up resources.
        void uninit();

    private:
        // Private constructor for the singleton pattern.
        CUDP_AI_Receiver();

        // The main receiving loop runs in a separate thread.
        void run();
        
        // Helper method to parse the raw byte buffer into a ParsedDetection struct.
        bool parseDetection(const char* buffer, size_t size, ParsedDetection& detection_data);

    
    private:
        int sockfd_ = -1;                 // Socket file descriptor
        int port_ = -1;                   // UDP port number
        struct sockaddr_in servaddr_;     // Server address structure
        struct sockaddr_in clientaddr_;   // Client address structure (for `recvfrom` to store sender's info)
        static const int MAX_BUFFER_SIZE = 1024; // Maximum size for the UDP receive buffer

        std::atomic_bool m_exit {false}; // Atomic flag to safely signal the thread to exit.
        ONRECIEVE m_callback = nullptr;  // Callback function pointer.
        std::thread m_receiveThread;     // The thread that will run the `run()` method.
        std::mutex m_mutex;              // Mutex for thread-safe operations.
};

} // namespace tracker
} // namespace de

#endif // UDP_DE_OBJECT_DETECTIONTRACKING_H