#!/bin/bash

INSTALL_DIR=/mnt/usb-disk/ariss
SRC_DIR=.

echo Installing the iors control program

if [ ! -d "$INSTALL_DIR" ]; then
    echo "Mising $INSTALL_DIR"
    exit 5 # missing file system
fi

if [ ! -d "$INSTALL_DIR/bin" ]; then
    mkdir $INSTALL_DIR/bin
fi
if [ ! -d "$INSTALL_DIR/sstv_q" ]; then
    mkdir $INSTALL_DIR/sstv_q
fi
if [ ! -d "$INSTALL_DIR/pacsat_dir" ]; then
    mkdir $INSTALL_DIR/pacsat_dir
fi
cp $SRC_DIR/iors_control.config $INSTALL_DIR

