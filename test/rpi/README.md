# test/rpi — hardware-assisted tracker tests

Tests here need a Raspberry Pi (or any Linux host) with `v4l2loopback` —
unlike `test/` they talk to a real V4L2 device. Not part of `make test`.

## test_reseed_stall — CA-0041 reproducer

Seeds `CTracker::trackRect()` while the capture thread is blocked on its
first-frame read (the pre-start window that SIGABRTed de_tracker in
CA-0017), then stalls/resumes the producer mid-session.

```bash
sudo modprobe v4l2loopback video_nr=50 card_label="TE-TEST" exclusive_caps=1
g++ -std=gnu++17 -I../../src/tracker test_reseed_stall.cpp \
    ../../src/tracker/tracker.cpp ../../src/tracker/video.cpp \
    $(pkg-config --cflags --libs opencv4) -o test_reseed_stall -pthread
./test_reseed_stall /dev/video50
```

Pass: the second `trackRect` joins the superseded (still-blocked) thread
instead of aborting, and `onTrack` callbacks resume after the producer stall.
Old code aborts with `terminate called without an active exception`.

v4l2loopback note: with `exclusive_caps=1` the node only advertises CAPTURE
once a producer has opened the output side and issued `VIDIOC_STREAMON`
(no frames needed). `v4l2_feeder.c` is a minimal producer for probing;
`cv_read_probe.cpp` measures `VideoCapture::read()` blocking behaviour.
