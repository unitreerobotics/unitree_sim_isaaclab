#!/bin/bash
set -e 

# 0. Directory safety check
if [ ! -d "teleimager" ] || [ ! -f "requirements.txt" ]; then
    echo "Error: Please place and run this script from the root of the unitree_sim_isaaclab project!"
    exit 1
fi

# 1. Check Arguments
if [ "$#" -lt 2 ]; then
    echo "Usage: bash setup_env.sh <4.5|5.0|5.1> <env_name> [cuda_version (e.g., cu121, cu126)]"
    echo "Example: bash setup_env.sh 5.1 my_unitree_env cu126"
    exit 1
fi

# cmake/build-essential/openssl/git-lfs/unzip installed manually beforehand (no passwordless sudo in this environment)

ISAAC_VERSION=$1
ENV_NAME=$2
CUDA_VER=$3

# 2. Assign Version-Specific Variables
if [ "$ISAAC_VERSION" == "4.5" ]; then
    PYTHON_VER="3.10"
    TORCH_PKG="torch==2.5.1 torchvision==0.20.1"
    DEFAULT_CUDA="cu121"
    ISAAC_SIM_PKG="isaacsim[all,extscache]==4.5.0"
    # ISAAC_LAB_COMMIT="91ad4944f2b7fad29d52c04a5264a082bcaad71d"
elif [ "$ISAAC_VERSION" == "5.0" ]; then
    PYTHON_VER="3.11"
    TORCH_PKG="torch==2.7.0 torchvision==0.22.0 torchaudio==2.7.0"
    DEFAULT_CUDA="cu126"
    ISAAC_SIM_PKG="isaacsim[all,extscache]==5.0.0"
    # ISAAC_LAB_COMMIT="v2.2.0"
elif [ "$ISAAC_VERSION" == "5.1" ]; then
    PYTHON_VER="3.11"
    TORCH_PKG="torch==2.7.0 torchvision==0.22.0 torchaudio==2.7.0"
    DEFAULT_CUDA="cu126"
    ISAAC_SIM_PKG="isaacsim[all,extscache]==5.1.0"
    # ISAAC_LAB_COMMIT="80094be3245aa5c8376a7464d29cb4412ea518f5"
else
    echo "Error: Unsupported version. Please use 4.5, 5.0, or 5.1."
    exit 1
fi

if [ -z "$CUDA_VER" ]; then
    CUDA_VER=$DEFAULT_CUDA
fi
PYTORCH_CMD="pip install $TORCH_PKG --index-url https://download.pytorch.org/whl/$CUDA_VER"

UNITREE_DIR=$(pwd)


# ==========================================
# PHASE 1: PRE-CLONE ALL REPOSITORIES
# ==========================================
echo "=================================================="
echo "=== Phase 1: Cloning all required repositories ==="
echo "=================================================="


cd ..

if [ ! -d "IsaacLab" ]; then
    echo "Cloning Isaac Lab..."
    git clone https://github.com/isaac-sim/IsaacLab.git
fi

if [ ! -d "cyclonedds" ]; then
    echo "Cloning CycloneDDS..."
    git clone https://github.com/eclipse-cyclonedds/cyclonedds -b releases/0.10.x
fi

if [ ! -d "unitree_sdk2_python" ]; then
    echo "Cloning unitree_sdk2_python..."
    git clone https://github.com/unitreerobotics/unitree_sdk2_python
fi


cd "$UNITREE_DIR"
echo "**************************************************"
echo "Initializing submodules for unitree_sim_isaaclab..."
echo "**************************************************"
git submodule update --init --depth 1
# git submodule update --remote --merge


echo "**************************************************"
echo "Generate certificate files..."
echo "**************************************************"
mkdir -p ~/.config/xr_teleoperate/
if [ ! -f ~/.config/xr_teleoperate/cert.pem ]; then
    openssl req -x509 -nodes -days 365 -newkey rsa:2048 -keyout key.pem -out cert.pem -subj "/CN=xr-teleoperate"
    cp key.pem cert.pem ~/.config/xr_teleoperate/
    rm key.pem cert.pem
else
    echo "Existing cert found at ~/.config/xr_teleoperate/, keeping it."
fi


echo "**************************************************"
echo "Downloading Isaac Lab assets..."
echo "**************************************************"   
chmod +x fetch_assets.sh
bash fetch_assets.sh

# ==========================================
# PHASE 2: BUILD C++ DEPENDENCIES
# ==========================================
echo "=================================================="
echo "=== Phase 2: Pre-compiling CycloneDDS ==="
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
# PHASE 3: CONDA ENVIRONMENT & INSTALLATION
# ==========================================
echo "=================================================="
echo "=== Phase 3: Setting up Conda and installing packages ==="
echo "=================================================="
echo "**************************************************"
echo "Initializing Conda environment..."
echo "**************************************************"
CONDA_BASE=$(conda info --base)
source "$CONDA_BASE/etc/profile.d/conda.sh"

echo "Creating Conda environment: $ENV_NAME (Python $PYTHON_VER)..."
conda create -y -n "$ENV_NAME" python="$PYTHON_VER"
conda activate "$ENV_NAME"

echo "Installing PyTorch (CUDA: $CUDA_VER)..."
eval "$PYTORCH_CMD"

echo "Installing Isaac Sim $ISAAC_VERSION..."
pip install --upgrade pip
pip install "$ISAAC_SIM_PKG" --extra-index-url https://pypi.nvidia.com

echo "**************************************************"
echo "Modifying teleimager configurations..."
echo "**************************************************"
sed -i 's/type:.*/type: isaacsim/' teleimager/cam_config_server.yaml
sed -i 's/image_shape:.*/image_shape: [480, 640]/' teleimager/cam_config_server.yaml

echo "**************************************************"
echo "Installing Isaac Lab..."
echo "**************************************************"
cd ../IsaacLab
if [ "$ISAAC_VERSION" == "5.0" ]; then
    git checkout v2.2.0
fi
# git checkout "$ISAAC_LAB_COMMIT"
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
cd teleimager
pip install -e .
cd ..

if [ "$ISAAC_VERSION" == "5.0" ] || [ "$ISAAC_VERSION" == "5.1" ]; then
    echo "Applying libstdc++ patch..."
    conda install -y -c conda-forge libstdcxx-ng
fi


echo "=================================================="
echo "Environment $ENV_NAME setup complete for Isaac Sim $ISAAC_VERSION!"
echo "To begin using it, run: conda activate $ENV_NAME"
echo "=================================================="