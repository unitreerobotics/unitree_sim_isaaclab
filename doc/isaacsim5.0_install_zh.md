## Isaac Sim 5.0.0环境安装
### 2.1 Ubuntu 22.04 以及以上的安装(pip install)

-  创建虚拟环境

```
conda create -n unitree_sim_env python=3.11
conda activate unitree_sim_env
```
- 安装Pytorch

这个需要根据自己的CUDA版本进行安装，具体参考[Pytorch官方教程](https://pytorch.org/get-started/locally/),下面以CUDA 12为例进行安装

```
pip install torch==2.7.0 torchvision==0.22.0 torchaudio==2.7.0 --index-url https://download.pytorch.org/whl/cu126
```
-  安装 Isaac Sim 5.0.0

```
pip install --upgrade pip

pip install "isaacsim[all,extscache]==5.0.0" --extra-index-url https://pypi.nvidia.com

```
验证是否安装成功
```
isaacsim
```
第一次执行会有:Do you accept the EULA? (Yes/No):  Yes



- 安装Isaac Lab

```
git clone git@github.com:isaac-sim/IsaacLab.git

sudo apt install cmake build-essential

cd IsaacLab

git checkout v2.2.0

./isaaclab.sh --install 

```

验证安装是否成功
```
python scripts/tutorials/00_sim/create_empty.py
or
./isaaclab.sh -p scripts/tutorials/00_sim/create_empty.py
```

- 安装unitree_sdk2_python

```
git clone https://github.com/unitreerobotics/unitree_sdk2_python

cd unitree_sdk2_python

pip3 install -e .
```
- 安装其他依赖
```
pip install -r requirements.txt
```

### 2.2 Ubuntu 20.4安装(二进制安装)

- 下载二进制的Isaaac Sim

下载对应版本的
[二进制Isaac Sim 5.0.0](https://docs.isaacsim.omniverse.nvidia.com/latest/installation/download.html#download-isaac-sim-short)并解压；

假设isaac sim放在`/home/unitree/tools/isaac-sim`,请按照下面的步骤进行安装；

- 设在环境变量

```
export ISAACSIM_PATH="${HOME}/tools/isaac-sim"

export ISAACSIM_PYTHON_EXE="${ISAACSIM_PATH}/python.sh"

```
测试设置是否成功

```
${ISAACSIM_PATH}/isaac-sim.sh

或

${ISAACSIM_PYTHON_EXE} -c "print('Isaac Sim configuration is now complete.')" 

需要退出包括base在内的所有的conda环境

```

**注意：** 可以把上面命令写到bashrc文件中

- 安装 Isaac Lab

```
git clone git@github.com:isaac-sim/IsaacLab.git

sudo apt install cmake build-essential

cd IsaacLab

ln -s ${HOME}/tools/isaac-sim/ _isaac_sim     (请根据自己路径填写)

./isaaclab.sh --conda unitree_sim_env

conda activate  unitree_sim_env

./isaaclab.sh --install

```

- 安装 unitree_sdk2_python

```
git clone https://github.com/unitreerobotics/unitree_sdk2_python

cd unitree_sdk2_python

pip3 install -e .

```

- 安装其他的依赖

```
pip install -r requirements.txt

```

### 问题
- `libstdc++.so.6` 版本低
  
  **错误信息:**
  ```
  OSError: /home/unitree/tools/anaconda3/envs/env_isaaclab_tem/bin/../lib/libstdc++.so.6: version GLIBCXX_3.4.30' not found (required by /home/unitree/tools/anaconda3/envs/env_isaaclab_tem/lib/python3.11/site-packages/omni/libcarb.so)
  ```
  
  **解决方案:**
  ```bash
  conda install -c conda-forge libstdcxx-ng
  ```

- 安装 unitree_sdk2_python 时出错
  
  **错误信息:**
  ```
  Could not locate cyclonedds. Try to set CYCLONEDDS_HOME or CMAKE_PREFIX_PATH
  ```
  
  **解决方案:**
  首先编译并安装 cyclonedds，然后将 `CYCLONEDDS_HOME` 设置为编译的 cyclonedds 路径，最后安装 unitree_sdk2_python。更详细的说明请参考 [unitree_sdk2_python](https://github.com/unitreerobotics/unitree_sdk2_python?tab=readme-ov-file#faq) 官方文档。
