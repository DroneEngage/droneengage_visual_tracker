#!/bin/bash

ffmpeg -f lavfi -i testsrc=size=1280x720:rate=30        -pix_fmt yuv420p        -s 1280x720        -f v4l2 /dev/video3