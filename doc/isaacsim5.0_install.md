## Isaac Sim 5.0.0 Environment Installation
### 2.1 Installation on Ubuntu 22.04 and Later(pip install)

- **Create Virtual Environment**

```
conda create -n unitree_sim_env python=3.11
conda activate unitree_sim_env
```
- **Install Pytorch**

This needs to be installed according to your CUDA version. Please refer to the [official PyTorch installation guide](https://pytorch.org/get-started/locally/). The following example uses CUDA 12:

```
pip install torch==2.7.0 torchvision==0.22.0 torchaudio==2.7.0 --index-url https://download.pytorch.org/whl/cu126
```
- **Install Isaac Sim 5.0.0**

```
pip install --upgrade pip

pip install "isaacsim[all,extscache]==5.0.0" --extra-index-url https://pypi.nvidia.com
```
Verify successful installation:
```
isaacsim
```
First execution will show: Do you accept the EULA? (Yes/No):  Yes

-  **Install Isaac Lab**

```
git clone git@github.com:isaac-sim/IsaacLab.git

sudo apt install cmake build-essential

cd IsaacLab

git checkout v2.2.0

./isaaclab.sh --install 

```

Verify successful installation:
```
python scripts/tutorials/00_sim/create_empty.py
or
./isaaclab.sh -p scripts/tutorials/00_sim/create_empty.py
```

- **Install unitree_sdk2_python**

```
git clone https://github.com/unitreerobotics/unitree_sdk2_python

cd unitree_sdk2_python

pip3 install -e .
```

- **Install other dependencies**
```
pip install -r requirements.txt
```

### 2.2 Installation on Ubuntu 20.04(binary install)

- **Download Isaac Sim Binary**

Download the [Isaac Sim 5.0.0 binary](https://docs.isaacsim.omniverse.nvidia.com/latest/installation/download.html#download-isaac-sim-shortl) and extract it.

Assume the path to Isaac Sim is ``/home/unitree/tools/isaac-sim``. Follow the steps below:

- **Set environment variables**

Please replace with your own path

```
export ISAACSIM_PATH="${HOME}/tools/isaac-sim"            
export ISAACSIM_PYTHON_EXE="${ISAACSIM_PATH}/python.sh"  
```
Verify the setup:
```
${ISAACSIM_PATH}/isaac-sim.sh
# or
${ISAACSIM_PYTHON_EXE} -c "print('Isaac Sim configuration is now complete.')"

Note: All conda environments (including base) must be deactivated before running this.
```
**Note:** You can add the above commands to your ~/.bashrc file for convenience.

- **Install Isaac Lab**

```
git clone git@github.com:isaac-sim/IsaacLab.git

sudo apt install cmake build-essential

cd IsaacLab

ln -s ${HOME}/tools/isaac-sim/ _isaac_sim     (Please replace with your own path)

./isaaclab.sh --conda unitree_sim_env

conda activate unitree_sim_env

./isaaclab.sh --install

```

- **Install unitree_sdk2_python**

```
git clone https://github.com/unitreerobotics/unitree_sdk2_python

cd unitree_sdk2_python

pip3 install -e .

```

- **Install other dependencies**

```
pip install -r requirements.txt
```

### Problems

- `libstdc++.so.6` version is too low
  
  **Error:**
  ```
  OSError: /home/unitree/tools/anaconda3/envs/env_isaaclab_tem/bin/../lib/libstdc++.so.6: version GLIBCXX_3.4.30' not found (required by /home/unitree/tools/anaconda3/envs/env_isaaclab_tem/lib/python3.11/site-packages/omni/libcarb.so)
  ```
  
  **Solution:**
  ```bash
  conda install -c conda-forge libstdcxx-ng
  ```

- Error when installing unitree_sdk2_python
  
  **Error:**
  ```
  Could not locate cyclonedds. Try to set CYCLONEDDS_HOME or CMAKE_PREFIX_PATH
  ```
  
  **Solution:**
  First, compile and install cyclonedds, then set `CYCLONEDDS_HOME` to the path where cyclonedds was compiled, and finally install unitree_sdk2_python. For more detailed instructions, please refer to the [unitree_sdk2_python](https://github.com/unitreerobotics/unitree_sdk2_python?tab=readme-ov-file#faq) official documentation.
