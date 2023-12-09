#!/bin/bash

if [ "$#" -ne 1 ]; then
    echo "Usage: $0 <device name>"
    echo "  On the pi the first USB device is likely /dev/sda"
    echo "  BUT DANGER! /dev/sda is the primary hard disk on Ubuntu!"
    exit 1
fi
MOUNT_POINT=/mnt/usb-disk
echo "Rebuilding USB Drive $1 with partition ${1}1"

#Unmount the disk if mounted.  May be several partitions
umount ${1}1 || echo "${1}1 partition was not mounted"
umount ${1}2 || echo "${1}2 partition was not mounted"
umount ${1}3 || echo "${1}3 partition was not mounted"

echo 'type=83' | sudo sfdisk $1 || exit 1

mkfs.ext4 -F ${1}1 || exit 1

if [ ! -d $MOUNT_POINT ]; then
    echo Making mount point
    mkdir $MOUNT_POINT || exit 1
fi

# Not re-mount - this has to be in fstab already
mount -a || exit 1
systemctl daemon-reload

mkdir $MOUNT_POINT/ariss
chown pi $MOUNT_POINT/ariss
