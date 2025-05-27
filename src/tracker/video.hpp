#ifndef VIDEO_H
#define VIDEO_H

namespace de
{
namespace tracker
{

class CVideo
{

    public:
    
        static int xioctl(int fh, unsigned long request, void *arg);
        static bool getVideoResolution(const std::string& video_device_path, unsigned int& width, unsigned int& height);

};

}
}

#endif
