// CA-0041 reproducer: reseed while the capture thread is in its pre-start
// window (blocked on the first-frame read of a stalled v4l2loopback producer).
//
// Old code: second trackRect move-assigns over the joinable m_framesThread
//           -> std::terminate -> SIGABRT.
// New code: joins the superseded thread first; process survives, and the new
//           session resumes tracking once frames show up again.
//
// v4l2loopback with exclusive_caps=1 only advertises CAPTURE to new openers
// once a producer holds the output side AND has issued VIDIOC_STREAMON - no
// frames need to be written. So the feeder opens + S_FMT + STREAMON and then
// stalls: the consumer's first read blocks in select() (~10 s OpenCV
// timeout), which is exactly the CA-0017 pre-start window.
//
// Build (on the Pi, from this directory):
//   g++ -std=gnu++17 test_reseed_stall.cpp \
//       ../../src/tracker/tracker.cpp ../../src/tracker/video.cpp \
//       $(pkg-config --cflags --libs opencv4) -o test_reseed_stall -pthread
// Run:
//   modprobe v4l2loopback video_nr=50 card_label="TE-TEST" exclusive_caps=1
//   ./test_reseed_stall /dev/video50

#include <atomic>
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

#include "../../src/tracker/tracker.hpp"

using namespace de::tracker;

class CTestCallback : public CCallBack_Tracker {
public:
  std::atomic<int> track_calls{0};
  std::atomic<int> last_status{-1};

  void onTrack(const float &, const float &, const float &, const float &,
               const bool) override {
    ++track_calls;
  }
  void onTrackStatusChanged(const int &status) override {
    last_status = status;
    std::cout << "[cb] onTrackStatusChanged=" << status << std::endl;
  }
  void onCameraOverlayRect(const float &, const float &, const float &,
                           const float &) override {}
  void onCameraOverlayCrosshair(const float &, const float &,
                                const float &) override {}
  void onCameraOverlayText(const float &, const float &, const float &,
                           const std::string &) override {}
  void onCameraOverlayRemoveRect() override {}
  void onCameraOverlayRemoveText() override {}
  void onCameraOverlayRemove() override {}
};

class CFeeder {
public:
  CFeeder(const char *dev, int w, int h) : m_dev(dev), m_w(w), m_h(h) {}

  // Producer thread: open output side + S_FMT + STREAMON (flips the node to
  // CAPTURE for consumers) then write MJPEG frames only while m_running.
  void run() {
    int fd = open(m_dev, O_RDWR);
    if (fd < 0) {
      std::cerr << "[feed] open " << m_dev << " failed: " << strerror(errno)
                << std::endl;
      return;
    }
    struct v4l2_format fmt {};
    fmt.type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
    fmt.fmt.pix.width = m_w;
    fmt.fmt.pix.height = m_h;
    fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_MJPEG;
    fmt.fmt.pix.field = V4L2_FIELD_NONE;
    fmt.fmt.pix.sizeimage = m_w * m_h * 3;
    if (ioctl(fd, VIDIOC_S_FMT, &fmt) < 0)
      std::cerr << "[feed] S_FMT failed: " << strerror(errno) << std::endl;
    enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
    if (ioctl(fd, VIDIOC_STREAMON, &type) < 0)
      std::cerr << "[feed] STREAMON failed: " << strerror(errno) << std::endl;
    m_opened = true;

    int frame_no = 0;
    while (!m_done) {
      if (m_running) {
        cv::Mat img(m_h, m_w, CV_8UC3, cv::Scalar(30, 30, 30));
        // slow oscillation around the seed rect so the pixel tracker can hold
        int bx = 280 + static_cast<int>(20.0 * std::sin(frame_no / 10.0));
        cv::rectangle(img, cv::Rect(bx, 200, 120, 100),
                      cv::Scalar(0, 200, 255), cv::FILLED);
        std::vector<uchar> jpg;
        cv::imencode(".jpg", img, jpg);
        if (write(fd, jpg.data(), jpg.size()) < 0)
          std::cerr << "[feed] write failed: " << strerror(errno) << std::endl;
        ++frame_no;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    close(fd);
  }

  std::atomic<bool> m_opened{false};
  std::atomic<bool> m_running{false};
  std::atomic<bool> m_done{false};

private:
  const char *m_dev;
  int m_w, m_h;
};

int main(int argc, char **argv) {
  const char *dev = (argc > 1) ? argv[1] : "/dev/video50";
  const int W = 640, H = 480;

  // Producer attached (STREAMON) but stalled: empty queue -> consumer's
  // first read blocks, giving us the pre-start window.
  CFeeder feeder(dev, W, H);
  std::thread feed_thread(&CFeeder::run, &feeder);
  while (!feeder.m_opened && !feeder.m_done)
    std::this_thread::sleep_for(std::chrono::milliseconds(20));

  CTestCallback cb;
  CTracker tracker(&cb);
  if (!tracker.init(TRACKER_KCF, dev, "", 15, 5, W, H, false, false)) {
    std::cerr << "init failed" << std::endl;
    feeder.m_done = true;
    feed_thread.join();
    return 2;
  }
  std::cout << "[test] init ok" << std::endl;

  // --- session A: spawned while the producer is stalled --------------------
  // track2Rect blocks in `video_capture >> frame` -> the pre-start window.
  tracker.trackRect(0.4f, 0.4f, 0.2f, 0.2f);
  std::this_thread::sleep_for(std::chrono::seconds(3));

  // --- reseed inside the window --------------------------------------------
  // Producer resumes after ~2 s so session A's blocked read returns and the
  // join in the second trackRect can complete.
  std::thread resume([&] {
    std::this_thread::sleep_for(std::chrono::seconds(2));
    feeder.m_running = true;
  });

  std::cout << "[test] reseed while session A is blocked on first read..."
            << std::endl;
  auto t0 = std::chrono::steady_clock::now();
  tracker.trackRect(0.42f, 0.40f, 0.22f, 0.24f); // OLD code aborts here
  auto waited_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                       std::chrono::steady_clock::now() - t0)
                       .count();
  resume.join();
  std::cout << "[test] second trackRect returned after " << waited_ms
            << " ms, no abort" << std::endl;

  // --- producer stall while session B runs ----------------------------------
  std::this_thread::sleep_for(std::chrono::seconds(3));
  int tracks_before_stall = cb.track_calls.load();
  std::cout << "[test] onTrack calls before stall: " << tracks_before_stall
            << std::endl;

  feeder.m_running = false; // producer stall: frames stop, device stays open
  std::cout << "[test] producer stalled 4 s..." << std::endl;
  std::this_thread::sleep_for(std::chrono::seconds(4));
  int tracks_during_stall = cb.track_calls.load();

  feeder.m_running = true; // frames return
  std::this_thread::sleep_for(std::chrono::seconds(4));
  int tracks_after_resume = cb.track_calls.load();
  std::cout << "[test] onTrack calls after resume: " << tracks_after_resume
            << " (during stall: " << tracks_during_stall << ")" << std::endl;

  tracker.stop();
  feeder.m_done = true;
  feed_thread.join();
  tracker.uninit();

  bool resumed = tracks_after_resume > tracks_during_stall;
  std::cout << "[test] RESULT: abort-free reseed "
            << (waited_ms >= 1500 ? "(join waited for stalled session) " : "")
            << "| tracking resumed: " << (resumed ? "yes" : "no") << std::endl;
  return resumed ? 0 : 3;
}
