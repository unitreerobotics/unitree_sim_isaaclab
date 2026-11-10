## Isaac Sim 6.2.0 / Isaac Lab 3.0.0 环境安装

环境要求：Ubuntu 22.04 及以上（GLIBC ≥ 2.35）、NVIDIA 驱动 ≥ 580.65.06、conda。
`auto_setup_env.sh` 会自动完成下面所有步骤。以下命令默认 `IsaacLab`、`cyclonedds`、`unitree_sdk2_python` 与 `unitree_sim_isaaclab` 放在同一级目录。

### 1. 克隆仓库

```bash
git clone --recurse-submodules https://github.com/unitreerobotics/unitree_sim_isaaclab.git
cd unitree_sim_isaaclab
sudo apt install cmake build-essential openssl git-lfs unzip libturbojpeg
```

### 2. 创建 conda 环境

```bash
conda create -n unitree_sim_env python=3.12
conda activate unitree_sim_env
pip install --upgrade pip
```

### 3. 安装 PyTorch

```bash
pip install torch==2.11.0 torchvision==0.26.0 --index-url https://download.pytorch.org/whl/cu130
```

### 4. 安装 Isaac Sim 6.2.0

```bash
pip install "isaacsim[all,extscache]==6.2.0.0" --extra-index-url https://pypi.nvidia.com
```

首次运行需要接受 EULA（或设置 `export OMNI_KIT_ACCEPT_EULA=YES`）：

```bash
isaacsim     # "Do you accept the EULA? (Yes/No): Yes"
```

### 5. 安装 Isaac Lab 3.0.0

```bash
cd ..
git clone https://github.com/isaac-sim/IsaacLab.git
cd IsaacLab
git checkout v3.0.0
./isaaclab.sh --install
```

验证安装：

```bash
./isaaclab.sh -p scripts/tutorials/00_sim/create_empty.py
```

### 6. conda-forge 依赖

```bash
conda install -c conda-forge libstdcxx-ng
```

`libstdcxx-ng` 提供较新的 `libstdc++.so.6`（见常见问题 1）。**不要**用 conda 安装 pinocchio：它由 pip 安装（`pin` 包，随 Isaac Lab 一起装好），conda 版会把 conda 的 numpy / scipy 混进 pip 环境。

### 7. 编译 CycloneDDS 并安装 unitree_sdk2_python

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

### 8. 项目依赖与图像服务

```bash
cd ../unitree_sim_isaaclab
pip install -r requirements.txt
git submodule update --init teleimager   # 图像服务（子模块，sim 分支）
pip install -e "./teleimager[server]"
```

图像服务的仿真相机配置随 teleimager 包提供（`teleimager/src/teleimager/teleimager_server_isaacsim.yaml`：`type: isaacsim`，头部双目 `image_shape: [480, 1280]`），无需修改；如需自定义可设置环境变量 `TELEIMAGER_CONFIG` 指向自己的配置文件。

Ubuntu 22.04 自带的 libjpeg-turbo 是 2.x，若 `python -c "from turbojpeg import TurboJPEG; TurboJPEG()"` 报错，请执行 `pip install "PyTurboJPEG<2"`（`auto_setup_env.sh` 会自动处理）。

WebRTC（https）图像页面使用的证书：

```bash
mkdir -p ~/.config/teleimager/
openssl req -x509 -nodes -days 3650 -newkey rsa:2048 -subj "/CN=unitree-sim" \
    -keyout ~/.config/teleimager/key.pem -out ~/.config/teleimager/cert.pem
```

### 9. 资源下载

```bash
bash fetch_assets.sh
```

### 10. 运行

```bash
python sim_main.py --device cpu --task Isaac-PickPlace-Cylinder-G129-Dex1-Joint --enable_dex1_dds --robot_type g129
```

## 常见问题

**1. `GLIBCXX_3.4.30' not found`**

```
OSError: .../envs/unitree_sim_env/bin/../lib/libstdc++.so.6: version `GLIBCXX_3.4.30' not found (required by .../site-packages/omni/libcarb.so)
```

解决：`conda install -c conda-forge libstdcxx-ng`

**2. 安装 unitree_sdk2_python 时提示 `Could not locate cyclonedds`**

按第 7 步编译 CycloneDDS，并在 `pip install -e .` 之前设置 `CYCLONEDDS_HOME`。参考 [unitree_sdk2_python FAQ](https://github.com/unitreerobotics/unitree_sdk2_python?tab=readme-ov-file#faq)。

**3. 图像服务启动报错 `Address already in use`**

55555–55557 / 60001–60003 中有端口被其他程序占用（这些端口位于临时端口范围，任何客户端连接都可能占用）。可以将其预留：

```bash
sudo sysctl -w net.ipv4.ip_local_reserved_ports=55555-55557,60001-60003
```

**4. Isaac Sim 5.1 及更早版本**

Isaac Sim 4.5 / 5.0 / 5.1请切换到IsaacSim5.1.0分支。

**5. `No module named 'teleimager'` / `teleimager/` 目录为空**

图像服务是 git 子模块，需要拉取并安装：

```bash
git submodule update --init --recursive
pip install -e "./teleimager[server]"
```

**6. `Failed to initialize TurboJPEG` / `libjpeg-turbo 3.0+ is required`**

Ubuntu 22.04 自带的 libjpeg-turbo 是 2.x，请安装系统库和对应版本的绑定：

```bash
sudo apt install libturbojpeg
pip install "PyTurboJPEG<2"
```
