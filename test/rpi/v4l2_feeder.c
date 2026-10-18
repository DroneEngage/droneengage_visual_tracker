// Minimal v4l2loopback producer for bench tests: open the output side, set
// MJPG format, STREAMON, write `nframes` frames, then hold the device open
// (stalled producer) until the hold time expires. Usage:
//   v4l2_feeder <dev> <nframes> <hold_seconds>
// Build: gcc v4l2_feeder.c -o v4l2_feeder
#include <errno.h>
#include <fcntl.h>
#include <linux/videodev2.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <unistd.h>

int main(int argc, char **argv) {
  const char *dev = argc > 1 ? argv[1] : "/dev/video50";
  int nframes = argc > 2 ? atoi(argv[2]) : 3;
  int hold = argc > 3 ? atoi(argv[3]) : 60;

  int fd = open(dev, O_RDWR);
  if (fd < 0) {
    perror("open");
    return 1;
  }
  struct v4l2_format fmt = {0};
  fmt.type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
  fmt.fmt.pix.width = 640;
  fmt.fmt.pix.height = 480;
  fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_MJPEG;
  fmt.fmt.pix.sizeimage = 640 * 480 * 3;
  if (ioctl(fd, VIDIOC_S_FMT, &fmt) < 0) {
    perror("s_fmt");
    return 2;
  }
  enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
  if (ioctl(fd, VIDIOC_STREAMON, &type) < 0)
    perror("streamon");
  char *buf = malloc(fmt.fmt.pix.sizeimage);
  memset(buf, 0x55, fmt.fmt.pix.sizeimage);
  for (int i = 0; i < nframes; ++i)
    if (write(fd, buf, fmt.fmt.pix.sizeimage) < 0)
      perror("write");
  printf("producer wrote %d frame(s); holding open %ds\n", nframes, hold);
  fflush(stdout);
  sleep(hold);
  close(fd);
  return 0;
}
