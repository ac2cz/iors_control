#!/bin/bash

# TODO - need to pass in the soundcard name
aplay -c2 -Dhw:2,0 -r48000 $1

