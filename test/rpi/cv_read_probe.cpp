// Probe how cv::VideoCapture::read() behaves on a v4l2loopback device whose
// producer is stalled: with an empty queue each read blocks in select() for
// ~10 s and returns an empty frame. Used while investigating CA-0041.
//
// Build: g++ -std=gnu++17 cv_read_probe.cpp \
//          $(pkg-config --cflags --libs opencv4) -o cv_read_probe
// Run:   ./cv_read_probe /dev/video50   (with a stalled producer attached)
#include <opencv2/opencv.hpp>
#include <chrono>
#include <cstdio>
int main(int argc, char **argv) {
  const char *dev = argc > 1 ? argv[1] : "/dev/video50";
  cv::VideoCapture cap;
  cap.open(dev, cv::CAP_V4L2);
  printf("isOpened=%d\n", (int)cap.isOpened());
  cap.set(cv::CAP_PROP_FRAME_WIDTH, 640);
  cap.set(cv::CAP_PROP_FRAME_HEIGHT, 480);
  cap.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));
  printf("w=%.0f h=%.0f\n", cap.get(cv::CAP_PROP_FRAME_WIDTH),
         cap.get(cv::CAP_PROP_FRAME_HEIGHT));
  cv::Mat f;
  for (int i = 0; i < 5; i++) {
    auto t0 = std::chrono::steady_clock::now();
    bool ok = cap.read(f);
    auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                  std::chrono::steady_clock::now() - t0)
                  .count();
    printf("read %d: ok=%d empty=%d took=%lldms\n", i, (int)ok, (int)f.empty(),
           (long long)ms);
    fflush(stdout);
  }
  return 0;
}
