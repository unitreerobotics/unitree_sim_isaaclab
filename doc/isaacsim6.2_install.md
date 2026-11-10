## Isaac Sim 6.2.0 / Isaac Lab 3.0.0 Environment Installation

Requirements: Ubuntu 22.04 or newer (GLIBC ≥ 2.35), NVIDIA driver ≥ 580.65.06, conda.
`auto_setup_env.sh` performs all of the steps below automatically. The commands assume that `IsaacLab`, `cyclonedds` and `unitree_sdk2_python` are cloned next to `unitree_sim_isaaclab`.

### 1. Clone the repository

```bash
git clone --recurse-submodules https://github.com/unitreerobotics/unitree_sim_isaaclab.git
cd unitree_sim_isaaclab
sudo apt install cmake build-essential openssl git-lfs unzip libturbojpeg
```

### 2. Create the conda environment

```bash
conda create -n unitree_sim_env python=3.12
conda activate unitree_sim_env
pip install --upgrade pip
```

### 3. Install PyTorch

```bash
pip install torch==2.11.0 torchvision==0.26.0 --index-url https://download.pytorch.org/whl/cu130
```

### 4. Install Isaac Sim 6.2.0

```bash
pip install "isaacsim[all,extscache]==6.2.0.0" --extra-index-url https://pypi.nvidia.com
```

Accept the EULA once (or set `export OMNI_KIT_ACCEPT_EULA=YES`):

```bash
isaacsim     # "Do you accept the EULA? (Yes/No): Yes"
```

### 5. Install Isaac Lab 3.0.0

```bash
cd ..
git clone https://github.com/isaac-sim/IsaacLab.git
cd IsaacLab
git checkout v3.0.0
./isaaclab.sh --install
```

Check the installation:

```bash
./isaaclab.sh -p scripts/tutorials/00_sim/create_empty.py
```

### 6. conda-forge packages

```bash
conda install -c conda-forge libstdcxx-ng
```

`libstdcxx-ng` provides a recent `libstdc++.so.6` (see FAQ 1). Do **not** install pinocchio with conda: it comes from pip (package `pin`, installed with Isaac Lab), and the conda package pulls conda numpy / scipy into the pip environment.

### 7. Build CycloneDDS and install unitree_sdk2_python

```bash
cd ..
git clone https://github.com/eclipse-cyclonedds/cyclonedds -b releases/0.10.x
cd cyclonedds && mkdir build install && cd build
cmake .. -DCMAKE_INSTALL_PREFIX=../install
cmake --build . --target install
cd ..
export CYCLONEDDS_HOME="$(pwd)/install"

cd ..
git clone https://github.com/unitreerobotics/unitree_sdk2_python
cd unitree_sdk2_python
pip install -e .
```

### 8. Project requirements and the image server

```bash
cd ../unitree_sim_isaaclab
pip install -r requirements.txt
git submodule update --init teleimager   # image server (submodule, branch sim)
pip install -e "./teleimager[server]"
```

The simulator camera config of the image server is bundled with the teleimager package (`teleimager/src/teleimager/teleimager_server_isaacsim.yaml`: `type: isaacsim`, stereo head camera `image_shape: [480, 1280]`). To use your own, point the environment variable `TELEIMAGER_CONFIG` to it.

Ubuntu 22.04 ships libjpeg-turbo 2.x; if `python -c "from turbojpeg import TurboJPEG; TurboJPEG()"` fails, run `pip install "PyTurboJPEG<2"` (`auto_setup_env.sh` does this automatically).

Certificates for the WebRTC (https) image pages:

```bash
mkdir -p ~/.config/teleimager/
openssl req -x509 -nodes -days 3650 -newkey rsa:2048 -subj "/CN=unitree-sim" \
    -keyout ~/.config/teleimager/key.pem -out ~/.config/teleimager/cert.pem
```

### 9. Assets

```bash
bash fetch_assets.sh
```

### 10. Run

```bash
python sim_main.py --device cpu --task Isaac-PickPlace-Cylinder-G129-Dex1-Joint --enable_dex1_dds --robot_type g129
```

## FAQ

**1. `GLIBCXX_3.4.30' not found`**

```
OSError: .../envs/unitree_sim_env/bin/../lib/libstdc++.so.6: version `GLIBCXX_3.4.30' not found (required by .../site-packages/omni/libcarb.so)
```

Solution: `conda install -c conda-forge libstdcxx-ng`

**2. `Could not locate cyclonedds` when installing unitree_sdk2_python**

Build CycloneDDS as in step 7 and set `CYCLONEDDS_HOME` before `pip install -e .`. See the [unitree_sdk2_python FAQ](https://github.com/unitreerobotics/unitree_sdk2_python?tab=readme-ov-file#faq).

**3. The image server reports `Address already in use`**

Another program holds one of the ports 55555–55557 / 60001–60003 (ephemeral ports can be taken by any client connection). Reserve them:

```bash
sudo sysctl -w net.ipv4.ip_local_reserved_ports=55555-55557,60001-60003
```

**4. Isaac Sim 5.1 or older**

Isaac Sim 4.5 / 5.0 / 5.1: Please switch to the IsaacSim5.1.0 branch.

**5. `No module named 'teleimager'` / the `teleimager/` folder is empty**

The image server is a git submodule. Fetch and install it:

```bash
git submodule update --init --recursive
pip install -e "./teleimager[server]"
```

**6. `Failed to initialize TurboJPEG` / `libjpeg-turbo 3.0+ is required`**

Ubuntu 22.04 ships libjpeg-turbo 2.x. Install the system library and the matching binding:

```bash
sudo apt install libturbojpeg
pip install "PyTurboJPEG<2"
```
