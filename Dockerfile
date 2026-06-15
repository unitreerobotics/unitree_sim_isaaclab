# ==============================
# Stage 1: Builder
# ==============================
FROM nvidia/cuda:12.2.0-runtime-ubuntu22.04 AS builder

ENV DEBIAN_FRONTEND=noninteractive
ENV TZ=Asia/Shanghai
ENV CONDA_DIR=/opt/conda
ENV PATH=$CONDA_DIR/bin:$PATH

# 设置 HTTP/HTTPS 代理（可选）
ARG http_proxy
ARG https_proxy
ENV http_proxy=${http_proxy}
ENV https_proxy=${https_proxy}

# 安装构建依赖（GCC 12 + GLU + Vulkan）
RUN apt-get update && apt-get install -y --no-install-recommends \
    gcc-12 g++-12 cmake build-essential unzip git-lfs \
    libglu1-mesa-dev vulkan-tools  wget\
    && update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-12 100 \
    && update-alternatives --install /usr/bin/g++ g++ /usr/bin/g++-12 100 \
    && rm -rf /var/lib/apt/lists/*
# 安装 Miniconda
RUN wget https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-x86_64.sh -O miniconda.sh && \
    bash miniconda.sh -b -p $CONDA_DIR && \
    rm miniconda.sh && \
    $CONDA_DIR/bin/conda clean -afy
# 接受 Conda TOS + 创建环境
RUN conda tos accept --override-channels --channel https://repo.anaconda.com/pkgs/main && \
    conda tos accept --override-channels --channel https://repo.anaconda.com/pkgs/r && \
    conda create -n unitree_sim_env python=3.11 -y && \
    conda clean -afy

# 切换到 Conda 环境
SHELL ["conda", "run", "-n", "unitree_sim_env", "/bin/bash", "-c"]  



RUN conda install -y -c conda-forge "libgcc-ng>=12" "libstdcxx-ng>=12" && \
    apt-get update && apt-get install -y libvulkan1 vulkan-tools && rm -rf /var/lib/apt/lists/*


# 安装 PyTorch（CUDA 12.6 对应）
RUN pip install --upgrade pip && \
    pip install torch==2.7.0 torchvision==0.22.0 torchaudio==2.7.0 --index-url https://download.pytorch.org/whl/cu126


# 安装 Isaac Sim
RUN pip install "isaacsim[all,extscache]==5.1.0" --extra-index-url https://pypi.nvidia.com

# 创建工作目录
RUN mkdir -p /home/code
WORKDIR /home/code

# 克隆并安装 IsaacLab
RUN git clone https://github.com/isaac-sim/IsaacLab.git && \
    cd IsaacLab && \
    echo "Yes" | ./isaaclab.sh --install
    
# 构建 CycloneDDS
RUN git clone https://github.com/eclipse-cyclonedds/cyclonedds -b releases/0.10.x /cyclonedds && \
    cd /cyclonedds && mkdir build install && cd build && \
    cmake .. -DCMAKE_INSTALL_PREFIX=../install && \
    cmake --build . --target install

# 设置 CycloneDDS 环境变量
ENV CYCLONEDDS_HOME=/cyclonedds/install

# 安装 unitree_sdk2_python
RUN git clone https://github.com/unitreerobotics/unitree_sdk2_python && \
    cd unitree_sdk2_python && pip install -e .

# 克隆 unitree_sim_isaaclab
RUN git clone https://github.com/unitreerobotics/unitree_sim_isaaclab.git /home/code/unitree_sim_isaaclab && \
    cd /home/code/unitree_sim_isaaclab && git submodule update --init --depth 1 && \
    cd teleimager && pip install -e . && \
    cd ../ && pip install -r requirements.txt


# ==============================
# Stage 2: Runtime
# ==============================
FROM nvidia/cuda:12.2.0-runtime-ubuntu22.04 AS runtime

ENV DEBIAN_FRONTEND=noninteractive
ENV TZ=Asia/Shanghai
ENV CONDA_DIR=/opt/conda
ENV PATH=$CONDA_DIR/bin:$PATH

# 禁止 Isaac Sim 自动启动
ENV OMNI_KIT_ALLOW_ROOT=1
ENV OMNI_KIT_DISABLE_STARTUP=1

# 安装运行时依赖
RUN apt-get update && apt-get install -y --no-install-recommends \
    libglu1-mesa git-lfs zenity unzip openssl \
    libxt6 libxrandr2 libxcursor1 libxi6 libxinerama1 libxxf86vm1 \
    && rm -rf /var/lib/apt/lists/*

# 复制 Conda 环境和代码
COPY --from=builder /home/code/IsaacLab /home/code/IsaacLab
COPY --from=builder /home/code/unitree_sdk2_python /home/code/unitree_sdk2_python

COPY --from=builder /cyclonedds /cyclonedds
COPY --from=builder /opt/conda /opt/conda
COPY --from=builder /home/code/unitree_sim_isaaclab /home/code/unitree_sim_isaaclab

# 生成 WebRTC 所需的自签名 TLS 证书
RUN openssl req -x509 -newkey rsa:2048 \
    -keyout /home/code/unitree_sim_isaaclab/teleimager/key.pem \
    -out /home/code/unitree_sim_isaaclab/teleimager/cert.pem \
    -days 3650 -nodes -subj "/CN=localhost" \
    -addext "subjectAltName=IP:127.0.0.1"


ENV CYCLONEDDS_HOME=/cyclonedds/install

# 写入 bashrc 初始化
RUN echo 'source /opt/conda/etc/profile.d/conda.sh' >> ~/.bashrc && \
    echo 'conda activate unitree_sim_env' >> ~/.bashrc && \
    echo 'export OMNI_KIT_ALLOW_ROOT=1' >> ~/.bashrc && \
    echo 'export OMNI_KIT_DISABLE_STARTUP=1' >> ~/.bashrc

WORKDIR /home/code

# 默认进入 Conda 环境 bash
CMD ["conda", "run", "-n", "unitree_sim_env", "/bin/bash"]