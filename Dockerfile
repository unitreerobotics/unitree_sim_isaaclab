# unitree_sim_isaaclab: Isaac Sim 6.2.0 + Isaac Lab 3.0.0 (Python 3.12, PyTorch 2.11 / CUDA 13.0)
#
# 构建（在仓库根目录，使用当前目录的源码）:
#   sudo docker build -t unitree-sim:latest -f Dockerfile .
#   # 需要代理时: --build-arg http_proxy=http://127.0.0.1:7890 --build-arg https_proxy=http://127.0.0.1:7890
#
# 资源（assets/，约 2 GB）不打包进镜像，运行时挂载到 /home/code/unitree_sim_isaaclab/assets（见 README）。
# 宿主机要求：NVIDIA 驱动 >= 580.65.06，已安装 NVIDIA Container Toolkit。

# ==============================
# Stage 1: Builder
# ==============================
FROM nvidia/cuda:13.0.0-runtime-ubuntu22.04 AS builder

ENV DEBIAN_FRONTEND=noninteractive
ENV TZ=Asia/Shanghai
ENV CONDA_DIR=/opt/conda
ENV PATH=$CONDA_DIR/bin:$PATH
ENV OMNI_KIT_ACCEPT_EULA=YES
ENV OMNI_KIT_ALLOW_ROOT=1

# 设置 HTTP/HTTPS 代理（可选）
ARG http_proxy
ARG https_proxy
ENV http_proxy=${http_proxy}
ENV https_proxy=${https_proxy}

# 使用阿里云源
RUN sed -i 's|http://archive.ubuntu.com/ubuntu/|http://mirrors.aliyun.com/ubuntu/|g' /etc/apt/sources.list && \
    sed -i 's|http://security.ubuntu.com/ubuntu/|http://mirrors.aliyun.com/ubuntu/|g' /etc/apt/sources.list

# 构建依赖（GCC 12 + GLU + Vulkan）
RUN apt-get update && apt-get install -y --no-install-recommends \
    gcc-12 g++-12 cmake build-essential git git-lfs unzip wget ca-certificates openssl \
    libglu1-mesa-dev libvulkan1 vulkan-tools \
    && update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-12 100 \
    && update-alternatives --install /usr/bin/g++ g++ /usr/bin/g++-12 100 \
    && rm -rf /var/lib/apt/lists/*

# 安装 Miniconda
RUN wget https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-x86_64.sh -O miniconda.sh && \
    bash miniconda.sh -b -p $CONDA_DIR && \
    rm miniconda.sh && \
    $CONDA_DIR/bin/conda clean -afy

# 接受 Conda TOS + 创建 Python 3.12 环境
RUN conda tos accept --override-channels --channel https://repo.anaconda.com/pkgs/main && \
    conda tos accept --override-channels --channel https://repo.anaconda.com/pkgs/r && \
    conda create -n unitree_sim_env python=3.12 -y && \
    conda clean -afy

# 后续命令在 Conda 环境中执行
SHELL ["conda", "run", "-n", "unitree_sim_env", "/bin/bash", "-c"]

# conda-forge 依赖：新版 libstdc++（Isaac Sim 需要 GLIBCXX_3.4.30）。pinocchio 由 pip 安装（Isaac Lab 依赖的 pin 包），
# 不要用 conda 安装（会把 conda 的 numpy / scipy 混进 pip 环境）
RUN conda install -y -c conda-forge "libgcc-ng>=12" "libstdcxx-ng>=12" && \
    conda clean -afy

# PyTorch 2.11（CUDA 13.0）
RUN pip install --upgrade pip && \
    pip install torch==2.11.0 torchvision==0.26.0 --index-url https://download.pytorch.org/whl/cu130

# Isaac Sim 6.2.0
RUN pip install "isaacsim[all,extscache]==6.2.0.0" --extra-index-url https://pypi.nvidia.com

RUN mkdir -p /home/code
WORKDIR /home/code

# Isaac Lab 3.0.0
RUN git clone https://github.com/isaac-sim/IsaacLab.git && \
    cd IsaacLab && git checkout v3.0.0 && \
    ./isaaclab.sh --install

# CycloneDDS
RUN git clone https://github.com/eclipse-cyclonedds/cyclonedds -b releases/0.10.x /cyclonedds && \
    cd /cyclonedds && mkdir build install && cd build && \
    cmake .. -DCMAKE_INSTALL_PREFIX=../install && \
    cmake --build . --target install
ENV CYCLONEDDS_HOME=/cyclonedds/install

# unitree_sdk2_python
RUN git clone https://github.com/unitreerobotics/unitree_sdk2_python && \
    cd unitree_sdk2_python && pip install -e .

# unitree_sim_isaaclab（当前目录的源码，teleimager 为源码目录；assets 见 .dockerignore）
COPY . /home/code/unitree_sim_isaaclab
RUN cd /home/code/unitree_sim_isaaclab && \
    pip install -r requirements.txt && \
    cd teleimager && pip install -e . && \
    pip cache purge

# 图像服务（WebRTC / https）使用的自签名证书
RUN mkdir -p /root/.config/xr_teleoperate && \
    openssl req -x509 -nodes -days 3650 -newkey rsa:2048 -subj "/CN=unitree-sim" \
        -keyout /root/.config/xr_teleoperate/key.pem -out /root/.config/xr_teleoperate/cert.pem


# ==============================
# Stage 2: Runtime
# ==============================
FROM nvidia/cuda:13.0.0-runtime-ubuntu22.04 AS runtime

ENV DEBIAN_FRONTEND=noninteractive
ENV TZ=Asia/Shanghai
ENV CONDA_DIR=/opt/conda
ENV PATH=$CONDA_DIR/bin:$PATH
ENV OMNI_KIT_ACCEPT_EULA=YES
ENV OMNI_KIT_ALLOW_ROOT=1
ENV OMNI_KIT_DISABLE_STARTUP=1
ENV CYCLONEDDS_HOME=/cyclonedds/install

# 运行时依赖（Isaac Sim 渲染 / 窗口所需的 X11、GL、Vulkan 库）
RUN apt-get update && apt-get install -y --no-install-recommends \
    libglu1-mesa libgl1 libegl1 libvulkan1 libxt6 libxrandr2 libxi6 libxkbcommon0 libsm6 libice6 \
    git git-lfs zenity unzip \
    && rm -rf /var/lib/apt/lists/*

COPY --from=builder /opt/conda /opt/conda
COPY --from=builder /cyclonedds /cyclonedds
COPY --from=builder /home/code /home/code
COPY --from=builder /root/.config/xr_teleoperate /root/.config/xr_teleoperate

RUN echo 'source /opt/conda/etc/profile.d/conda.sh' >> ~/.bashrc && \
    echo 'conda activate unitree_sim_env' >> ~/.bashrc

WORKDIR /home/code/unitree_sim_isaaclab

# 默认进入 Conda 环境 bash
CMD ["conda", "run", "--no-capture-output", "-n", "unitree_sim_env", "/bin/bash"]
