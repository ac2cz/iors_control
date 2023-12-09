#!/bin/bash

# This assumes we are using the default soundcard
# The audio file is stereo, even though only one channel is needed
echo STARTING SSTV
aplay -c2 -Dhw:0,0 -r48000 $1

