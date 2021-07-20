#!/bin/sh

# i2cset -y 1 0x70 0x00 0x01
# gpio -g mode 17 out
# gpio -g mode 4  out
# gpio -g write 17 0 #set the gpio17 low
# gpio -g write 4 0 #set the gpio4   low

python3 imageProcess.py

#feh --scale-down --auto-zoom

feh photos/filtered.jpg

