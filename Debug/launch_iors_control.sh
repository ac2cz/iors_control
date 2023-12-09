#!/bin/bash

HOME_DIR=/home/pi
BIN_DIR=/home/pi/bin/iors_control
USB_DIR=/mnt/usb-disk/ariss
CONFIG_FILE=iors_control.config
CONFIG=$HOME_DIR/$CONFIG
BIN=iors_control
COMMAND=$HOME_DIR/bin/$BIN

echo Launching IORS CONTROL

# If the config exists on the USB drive then use it
if [ -f "$USB_DIR/$CONFIG_FILE" ]; then
    CONFIG=$USB_DIR/$CONFIG_FILE
fi

# If the program exists on the USB drive then run it from there
if [ -f "$USB_DIR/bin/$BIN" ]; then
    COMMAND=$USB_DIR/bin/$BIN
fi

echo Running: $COMMAND -c $CONFIG

$COMMAND -c $CONFIG > /tmp/iors_control.log

