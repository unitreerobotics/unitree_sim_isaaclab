#!/bin/bash
# =============================================================================
# Script to setup UVC camera access for non-root users and allow
# passwordless modprobe reload of the UVC driver.
#
# This will:
# 1. Add udev rules so users in the 'video' group can access USB cameras.
# 2. Add the current user to the 'video' group.
# 3. Grant passwordless sudo permission to reload the uvcvideo kernel module.
# 4. Reload the UVC driver immediately.

# Author: https://github.com/silencht
# Company: Unitree Robotics
# Copyright © 2016–2025 YuShu Technology Co., Ltd. All Rights Reserved.
# =============================================================================

set -e

# Step 1: udev rules for USB video devices
echo 'SUBSYSTEM=="usb", ENV{DEVTYPE}=="usb_device", GROUP="video", MODE="0664"' | sudo tee /etc/udev/rules.d/10-libuvc.rules > /dev/null

# Apply udev rules immediately
sudo udevadm trigger

# Step 2: add current user to the 'video' group
sudo usermod -a -G video $USER
echo "User $USER added to 'video' group. Please logout and login again to apply."

# Step 3: grant passwordless sudo for modprobe
MODPROBE_PATH=$(which modprobe)
echo "ALL ALL=(ALL) NOPASSWD: $MODPROBE_PATH -r uvcvideo, $MODPROBE_PATH uvcvideo debug=*" | sudo tee /etc/sudoers.d/uvc_modprobe > /dev/null
sudo chmod 0440 /etc/sudoers.d/uvc_modprobe

# Step 4: reload UVC driver
sudo $MODPROBE_PATH -r uvcvideo
sudo $MODPROBE_PATH uvcvideo debug=0

echo "UVC setup completed successfully."