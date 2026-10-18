// Standalone v4l2loopback producer for de_tracker bench tests.
//
// Opens the output side, sets MJPG format, STREAMON (so exclusive_caps nodes
// advertise CAPTURE), then writes a MJPEG frame every `interval_ms` — a bright
// box slowly oscillating around the seed rect used by the tests. SIGSTOP the
// process to simulate an rpicam/ffmpeg producer stall (consumer reads block);
// SIGCONT resumes it.
//
// Build: g++ -std=gnu++17 mjpeg_feeder.cpp \
//          $(pkg-config --cflags --libs opencv4) -o mjpeg_feeder
// Run:   ./mjpeg_feeder /dev/video50 [interval_ms]

#include <chrono>
#include <cmath>
#include <cstring>
#include <fcntl.h>
#include <iostream>
#include <linux/videodev2.h>
#include <opencv2/opencv.hpp>
#include <sys/ioctl.h>
#include <thread>
#include <unistd.h>
#include <vector>

int main(int argc, char **argv) {
  const char *dev = (argc > 1) ? argv[1] : "/dev/video50";
  const int interval_ms = (argc > 2) ? atoi(argv[2]) : 50;
  const int W = 640, H = 480;

  int fd = open(dev, O_RDWR);
  if (fd < 0) {
    std::cerr << "[feed] open " << dev << " failed: " << strerror(errno)
              << std::endl;
    return 1;
  }
  struct v4l2_format fmt {};
  fmt.type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
  fmt.fmt.pix.width = W;
  fmt.fmt.pix.height = H;
  fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_MJPEG;
  fmt.fmt.pix.field = V4L2_FIELD_NONE;
  fmt.fmt.pix.sizeimage = W * H * 3;
  if (ioctl(fd, VIDIOC_S_FMT, &fmt) < 0)
    std::cerr << "[feed] S_FMT failed: " << strerror(errno) << std::endl;
  enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
  if (ioctl(fd, VIDIOC_STREAMON, &type) < 0)
    std::cerr << "[feed] STREAMON failed: " << strerror(errno) << std::endl;
  std::cout << "[feed] producing MJPG " << W << "x" << H << " on " << dev
            << " every " << interval_ms << "ms" << std::endl;

  int frame_no = 0;
  for (;;) {
    cv::Mat img(H, W, CV_8UC3, cv::Scalar(30, 30, 30));
    int bx = 280 + static_cast<int>(20.0 * std::sin(frame_no / 10.0));
    cv::rectangle(img, cv::Rect(bx, 200, 120, 100), cv::Scalar(0, 200, 255),
                  cv::FILLED);
    std::vector<uchar> jpg;
    cv::imencode(".jpg", img, jpg);
    if (write(fd, jpg.data(), jpg.size()) < 0)
      std::cerr << "[feed] write failed: " << strerror(errno) << std::endl;
    ++frame_no;
    std::this_thread::sleep_for(std::chrono::milliseconds(interval_ms));
  }
  return 0;
}
