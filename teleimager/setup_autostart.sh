#!/bin/bash

# ==============================================================
# Teleimager Auto-Start Setup Script
# --------------------------------------------------------------
# This script sets up a systemd service to automatically start
# the Teleimager image server on system boot.
#
# Features:
#   - Automatically detects your Conda installation path.
#   - Prompts for or detects the Conda environment to use.
#   - Checks for the 'teleimager-server' command availability.
#   - Optionally enables RealSense camera support (--rs).
#   - Creates a persistent systemd service at:
#         /etc/systemd/system/teleimager.service
#   - Enables, starts, and verifies the service.
#
# Requirements:
#   - Must be run with a user that has sudo privileges.
#   - The selected Conda environment must contain 'teleimager-server'.
#   - The script will prompt for your sudo password once.
#
# Usage:
#   bash setup_autostart.sh
#
# After setup, manage the service with:
#   sudo systemctl status teleimager.service     # Check status
#   sudo journalctl -u teleimager.service -f     # View logs
#   sudo systemctl restart teleimager.service    # Restart service
#   sudo systemctl disable teleimager.service    # Disable auto-start
#
# Author: https://github.com/silencht
# Company: Unitree Robotics
# Copyright © 2016–2025 YuShu Technology Co., Ltd. All Rights Reserved.
# ==============================================================

set -e
echo "=== Setting up Teleimager auto-start service ==="

# Step 0: Detect script directory
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
echo "Script directory detected: $SCRIPT_DIR"

# Ensure PYTHONPATH target exists
if [ ! -d "$SCRIPT_DIR/src" ]; then
    echo "Warning: $SCRIPT_DIR/src not found. Creating it..."
    mkdir -p "$SCRIPT_DIR/src"
fi

# Step 1: Detect Conda installation (robust way)
if command -v conda >/dev/null 2>&1; then
    CONDA_BIN_PATH="$(which conda)"
    # Example: /home/changhe/miniforge3/envs/tv/bin/conda
    if [[ "$CONDA_BIN_PATH" == *"/envs/"* ]]; then
        # remove everything from /envs/... onward, leaving the base prefix
        CONDA_PATH="${CONDA_BIN_PATH%/envs/*}"
    else
        # fallback: go up two directories from conda binary
        CONDA_PATH="$(dirname "$(dirname "$CONDA_BIN_PATH")")"
    fi
    echo "Detected Conda base path: $CONDA_PATH"
else
    read -p "Conda not detected automatically. Please enter the full Conda base path: " CONDA_PATH
    if [ ! -d "$CONDA_PATH" ]; then
        echo "Error: Conda path does not exist: $CONDA_PATH"
        exit 1
    fi
fi

# Step 2: Determine Conda environment
CURRENT_ENV="${CONDA_DEFAULT_ENV:-}"
if [ -n "$CURRENT_ENV" ]; then
    echo "Current Conda environment detected: $CURRENT_ENV"
    read -p "Use this environment for Teleimager? (y for yes, n/anything else for no): " USE_CURRENT
    if [[ "$USE_CURRENT" =~ ^([yY])$ ]]; then
        CONDA_ENV="$CURRENT_ENV"
    fi
fi

# If CONDA_ENV not set, ask user
if [ -z "$CONDA_ENV" ]; then
    read -p "Enter the Conda environment name for Teleimager (e.g., tv): " CONDA_ENV
fi

# Verify environment exists
if ! "$CONDA_PATH/bin/conda" env list | grep -qE "^[[:space:]]*$CONDA_ENV[[:space:]]+"; then
    echo "Error: Conda environment '$CONDA_ENV' not found."
    exit 1
fi
echo "Using Conda environment: $CONDA_ENV"

# Step 3: Test teleimager-server availability
echo "Checking teleimager-server command in environment '$CONDA_ENV'..."
source "$CONDA_PATH/etc/profile.d/conda.sh"
conda activate "$CONDA_ENV"

if ! command -v teleimager-server >/dev/null 2>&1; then
    echo "Error: teleimager-server command not found in '$CONDA_ENV'."
    exit 1
fi
echo "teleimager-server command found successfully."

# Step 4: Ask if RealSense cameras are used
read -p "Are you using RealSense cameras? (y for yes, n/anything else for no): " USE_RS_INPUT
if [[ "$USE_RS_INPUT" =~ ^([yY])$ ]]; then
    USE_RS="--rs"
else
    USE_RS=""
fi
echo "RealSense option: $USE_RS"

# Step 5: Create systemd service file
SERVICE_NAME="teleimager.service"
SERVICE_FILE="/etc/systemd/system/$SERVICE_NAME"
CMD_WITH_ARGS="teleimager-server $USE_RS"

echo "Creating systemd service file at $SERVICE_FILE..."
sudo tee "$SERVICE_FILE" > /dev/null << EOL
[Unit]
Description=Teleimager Image Server
After=network.target

[Service]
Type=simple
User=root
WorkingDirectory=$SCRIPT_DIR
CPUAffinity=0 1 2
ExecStart=/bin/bash -lc "source $CONDA_PATH/etc/profile.d/conda.sh && conda activate $CONDA_ENV && $CMD_WITH_ARGS"
Restart=always
RestartSec=5
Environment="PATH=$CONDA_PATH/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin"
Environment="PYTHONPATH=$SCRIPT_DIR/src"
Environment="XR_TELEOP_CERT=/home/unitree/.config/xr_teleoperate/cert.pem"
Environment="XR_TELEOP_KEY=/home/unitree/.config/xr_teleoperate/key.pem"
StandardOutput=journal+console
StandardError=journal+console

[Install]
WantedBy=multi-user.target
EOL

# Step 6: Enable service
echo "Reloading systemd daemon..."
sudo systemctl daemon-reload

echo "Enabling $SERVICE_NAME to start on boot..."
sudo systemctl enable "$SERVICE_NAME"

echo "Starting $SERVICE_NAME..."
sudo systemctl restart "$SERVICE_NAME"

echo "Checking service status..."
sleep 2
sudo systemctl status "$SERVICE_NAME" --no-pager

echo ""
echo "=== Teleimager auto-start service setup completed successfully! ==="
echo "You can manage the service using:"
echo "  sudo systemctl status $SERVICE_NAME      # Check status"
echo "  sudo journalctl -u $SERVICE_NAME -f      # View logs in real-time"
echo "  sudo systemctl restart $SERVICE_NAME     # Restart service"
echo "  sudo systemctl stop $SERVICE_NAME        # Stop service"
echo "  sudo systemctl disable $SERVICE_NAME     # Disable auto-start on boot"
echo "================================================================"
