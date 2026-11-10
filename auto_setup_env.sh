#!/bin/bash
# Automatic environment setup for unitree_sim_isaaclab
#   Isaac Sim 6.2.0 + Isaac Lab 3.0.0 (Python 3.12, PyTorch 2.11 / CUDA 13.0)
#
# Usage:   bash auto_setup_env.sh <env_name> [cuda_tag]
# Example: bash auto_setup_env.sh unitree_sim_env            # default cuda_tag: cu130
#
# Requirements: Ubuntu 22.04 or newer (GLIBC >= 2.35), NVIDIA driver >= 580.65.06, conda.
# IsaacLab, cyclonedds and unitree_sdk2_python are cloned next to this repository.
set -e

# ---- versions -------------------------------------------------------------
PYTHON_VER="3.12"
TORCH_PKG="torch==2.11.0 torchvision==0.26.0"
DEFAULT_CUDA="cu130"
ISAAC_SIM_PKG="isaacsim[all,extscache]==6.2.0.0"
ISAAC_LAB_TAG="v3.0.0"
CYCLONEDDS_BRANCH="releases/0.10.x"

# 0. Directory safety check
if [ ! -f "requirements.txt" ] || [ ! -f "sim_main.py" ]; then
    echo "Error: Please place and run this script from the root of the unitree_sim_isaaclab project!"
    exit 1
fi
# teleimager (image server) is a git submodule (branch sim)
git submodule update --init --recursive teleimager
if [ ! -f "teleimager/pyproject.toml" ]; then
    echo "Error: the teleimager submodule is missing. Run: git submodule update --init teleimager"
    exit 1
fi

# 1. Check arguments
if [ "$#" -lt 1 ]; then
    echo "Usage: bash auto_setup_env.sh <env_name> [cuda_tag (default: $DEFAULT_CUDA)]"
    echo "Example: bash auto_setup_env.sh unitree_sim_env"
    echo "Installs Isaac Sim 6.2.0 + Isaac Lab 3.0.0 (Python $PYTHON_VER)."
    exit 1
fi
ENV_NAME=$1
CUDA_VER=${2:-$DEFAULT_CUDA}

# 2. System checks (warnings only)
GLIBC_VER=$(ldd --version 2>/dev/null | head -n1 | grep -oE '[0-9]+\.[0-9]+$' || true)
if [ -n "$GLIBC_VER" ] && [ "$(printf '%s\n' "2.35" "$GLIBC_VER" | sort -V | head -n1)" != "2.35" ]; then
    echo "Warning: GLIBC $GLIBC_VER found, Isaac Sim 6.2 needs GLIBC >= 2.35 (Ubuntu 22.04 or newer)."
fi
if command -v nvidia-smi >/dev/null 2>&1; then
    DRIVER_VER=$(nvidia-smi --query-gpu=driver_version --format=csv,noheader | head -n1)
    if [ "$(printf '%s\n' "580.65.06" "$DRIVER_VER" | sort -V | head -n1)" != "580.65.06" ]; then
        echo "Warning: NVIDIA driver $DRIVER_VER found, Isaac Sim 6.2 needs >= 580.65.06."
    fi
else
    echo "Warning: nvidia-smi not found - is the NVIDIA driver installed?"
fi
if ! command -v conda >/dev/null 2>&1; then
    echo "Error: conda not found. Please install Miniconda / Anaconda first."
    exit 1
fi

sudo apt-get update && sudo apt-get install -y cmake build-essential openssl git git-lfs unzip libturbojpeg

UNITREE_DIR=$(pwd)

# ==========================================
# PHASE 1: CLONE REPOSITORIES
# ==========================================
echo "=================================================="
echo "=== Phase 1: Cloning required repositories ==="
echo "=================================================="
cd ..

if [ ! -d "IsaacLab" ]; then
    echo "Cloning Isaac Lab ($ISAAC_LAB_TAG)..."
    git clone https://github.com/isaac-sim/IsaacLab.git
fi
(cd IsaacLab && git fetch --tags && git checkout "$ISAAC_LAB_TAG")

if [ ! -d "cyclonedds" ]; then
    echo "Cloning CycloneDDS..."
    git clone https://github.com/eclipse-cyclonedds/cyclonedds -b "$CYCLONEDDS_BRANCH"
fi

if [ ! -d "unitree_sdk2_python" ]; then
    echo "Cloning unitree_sdk2_python..."
    git clone https://github.com/unitreerobotics/unitree_sdk2_python
fi

cd "$UNITREE_DIR"

echo "**************************************************"
echo "Generating certificate files for the image server (WebRTC / https)..."
echo "**************************************************"
# teleimager 2.x reads ~/.config/teleimager/{cert,key}.pem (it falls back to ~/.config/xr_teleoperate/)
CERT_DIR=~/.config/teleimager
mkdir -p "$CERT_DIR"
if [ -f "$CERT_DIR/cert.pem" ] && [ -f "$CERT_DIR/key.pem" ]; then
    echo "Certificates already exist in $CERT_DIR, skipping."
elif [ -f ~/.config/xr_teleoperate/cert.pem ] && [ -f ~/.config/xr_teleoperate/key.pem ]; then
    echo "Copying the existing certificates from ~/.config/xr_teleoperate/ to $CERT_DIR."
    cp ~/.config/xr_teleoperate/cert.pem ~/.config/xr_teleoperate/key.pem "$CERT_DIR/"
else
    TMP_CERT=$(mktemp -d)
    openssl req -x509 -nodes -days 3650 -newkey rsa:2048 -subj "/CN=unitree-sim" \
        -keyout "$TMP_CERT/key.pem" -out "$TMP_CERT/cert.pem"
    cp "$TMP_CERT/key.pem" "$TMP_CERT/cert.pem" "$CERT_DIR/"
    rm -rf "$TMP_CERT"
fi
chmod 600 "$CERT_DIR/key.pem"

echo "**************************************************"
echo "Downloading scene / robot assets..."
echo "**************************************************"
chmod +x fetch_assets.sh
bash fetch_assets.sh

# ==========================================
# PHASE 2: BUILD C++ DEPENDENCIES
# ==========================================
echo "=================================================="
echo "=== Phase 2: Building CycloneDDS ==="
echo "=================================================="
cd ../cyclonedds
if [ ! -d "install" ]; then
    mkdir -p build install
    cd build
    cmake .. -DCMAKE_INSTALL_PREFIX=../install
    cmake --build . --target install
    cd ..
fi
export CYCLONEDDS_HOME="$(pwd)/install"
cd "$UNITREE_DIR"

# ==========================================
# PHASE 3: CONDA ENVIRONMENT & PACKAGES
# ==========================================
echo "=================================================="
echo "=== Phase 3: Conda environment and packages ==="
echo "=================================================="
CONDA_BASE=$(conda info --base)
source "$CONDA_BASE/etc/profile.d/conda.sh"

if conda env list | awk '{print $1}' | grep -qx "$ENV_NAME"; then
    echo "Conda environment '$ENV_NAME' already exists, reusing it."
else
    echo "Creating conda environment: $ENV_NAME (Python $PYTHON_VER)..."
    conda create -y -n "$ENV_NAME" python="$PYTHON_VER"
fi
conda activate "$ENV_NAME"
# recent libstdc++ (Isaac Sim needs GLIBCXX_3.4.30). Only conda package of the environment, installed before
# any pip package: conda packages with Python dependencies (e.g. pinocchio -> numpy / scipy) would mix conda
# and pip versions of the same modules. pinocchio comes from pip (package "pin", a dependency of Isaac Lab).
conda install -y -c conda-forge libstdcxx-ng
pip install --upgrade pip

echo "Installing PyTorch ($TORCH_PKG, $CUDA_VER)..."
pip install $TORCH_PKG --index-url "https://download.pytorch.org/whl/$CUDA_VER"

echo "Installing Isaac Sim 6.2.0..."
pip install "$ISAAC_SIM_PKG" --extra-index-url https://pypi.nvidia.com
export OMNI_KIT_ACCEPT_EULA=YES

echo "**************************************************"
echo "Installing Isaac Lab $ISAAC_LAB_TAG..."
echo "**************************************************"
cd ../IsaacLab
./isaaclab.sh --install

echo "**************************************************"
echo "Installing unitree_sdk2_python..."
echo "**************************************************"
cd ../unitree_sdk2_python
pip install -e .

echo "**************************************************"
echo "Installing remaining requirements and teleimager..."
echo "**************************************************"
cd "$UNITREE_DIR"
pip install -r requirements.txt
# image server; its simulator camera config (teleimager_server_isaacsim.yaml) is bundled in the package
pip install -e "./teleimager[server]"
# Ubuntu 22.04 ships libjpeg-turbo 2.x; PyTurboJPEG 2.x needs libjpeg-turbo 3
if ! python -c "from turbojpeg import TurboJPEG; TurboJPEG()" >/dev/null 2>&1; then
    pip install "PyTurboJPEG<2"
fi

echo "**************************************************"
echo "Checking the environment..."
echo "**************************************************"
python - <<'PYEOF'
import numpy, scipy.sparse, pinocchio, torch
assert int(numpy.__version__.split(".")[0]) >= 2, f"numpy {numpy.__version__} found, Isaac Lab 3.0 needs numpy >= 2"
print(f"numpy {numpy.__version__}, scipy {scipy.__version__}, pinocchio {pinocchio.__version__}, torch {torch.__version__}")
PYEOF

# accept the Omniverse EULA automatically in this environment
conda env config vars set OMNI_KIT_ACCEPT_EULA=YES >/dev/null

echo "=================================================="
echo "Environment $ENV_NAME is ready (Isaac Sim 6.2.0 / Isaac Lab 3.0.0)."
echo "To begin using it, run: conda activate $ENV_NAME"
echo "Quick test:"
echo "  python sim_main.py --device cpu --task Isaac-PickPlace-Cylinder-G129-Dex1-Joint --enable_dex1_dds --robot_type g129"
echo "=================================================="
