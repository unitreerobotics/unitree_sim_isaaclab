<div align="center">
  <h1 align="center">Tele Imager</h1>
  <a href="https://www.unitree.com/" target="_blank">
    <img src="https://www.unitree.com/images/0079f8938336436e955ea3a98c4e1e59.svg" alt="Unitree LOGO" width="15%">
  </a>
  <p align="center">
    <a href="README.md"> English</a> | <a>中文</a>
  </p>
  <p align="center">
  <p align="center">
    <a href="https://github.com/unitreerobotics/xr_teleoperate/wiki" target="_blank"> <img src="https://img.shields.io/badge/GitHub-Wiki-181717?logo=github" alt="Unitree LOGO"></a> <a href="https://discord.gg/ZwcVwxv5rq" target="_blank"><img src="https://img.shields.io/badge/-Discord-5865F2?style=flat&logo=Discord&logoColor=white" alt="Unitree LOGO"> <a href="https://deepwiki.com/unitreerobotics/teleimager"><img src="https://deepwiki.com/badge.svg" alt="Ask DeepWiki"></a> </a>
  </p>
</div>

## 1. 图像服务器（Image Server）

该仓库提供一个图像服务器，用于从多路摄像头（UVC、OpenCV 和 RealSense）采集视频流，并使用 ZeroMQ 或 WebRTC 方式进行网络发布。

目前 Tele Imager 用于 [xr_teleoperate](https://github.com/unitreerobotics/xr_teleoperate) 项目中提供遥操作视频流。

> 所有可供用户调用的 API 都在代码中的 `# public api` 注释下面。



### 1.0 ✨ 特性

- 📸 支持多路 UVC、OpenCV 和 Intel RealSense 摄像头
- 📢 使用 **ZeroMQ PUB-SUB** 方式发布视频帧
- 📢 使用 **WebRTC** 方式发布视频帧
- 🚧（TODO）本地共享内存模式，用于极低延迟访问帧数据
- 💬 通过 **ZeroMQ REQ-REP** 方式响应图像配置指令
- 🆔 多种摄像头识别方式：物理路径、序列号、video 设备路径
- ⚙️ 可配置分辨率和帧率
- 🚀 使用三重环形缓冲区实现高效帧处理



### 1.1 📥 环境配置

1. 安装 miniconda3

```
# for jetson orin nx (ARM architecture)
unitree@ubuntu:~$ mkdir -p ~/miniconda3
unitree@ubuntu:~$ wget https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-aarch64.sh -O ~/miniconda3/miniconda.sh
unitree@ubuntu:~$ bash ~/miniconda3/miniconda.sh -b -u -p ~/miniconda3
unitree@ubuntu:~$ rm ~/miniconda3/miniconda.sh
unitree@ubuntu:~$ source ~/miniconda3/bin/activate
(base) unitree@ubuntu:~$ conda init --all
```

2. 创建并激活 conda 环境：

```
(base) unitree@ubuntu:~$ conda create -n teleimager python=3.10 -y
(base) unitree@ubuntu:~$ conda activate teleimager
```

3. 安装项目与依赖：

```
(teleimager) unitree@ubuntu:~$ sudo apt install -y libusb-1.0-0-dev libturbojpeg-dev
(teleimager) unitree@ubuntu:~$ git clone https://github.com/unitreerobotics/teleimager.git
(teleimager) unitree@ubuntu:~$ cd teleimager
# 假如您只使用客户端
(teleimager) unitree@ubuntu:~/teleimager$ pip install -e .
# 假如您还使用服务端
(teleimager) unitree@ubuntu:~/teleimager$ pip install -e ".[server]"
```

4. 添加 video 权限（非 root 用户运行）：

```
bash setup_uvc.sh
```

5. 配置证书路径（WebRTC 模式需要）
    证书通常由 [televuer](https://github.com/unitreerobotics/televuer) 仓库生成。

   你可以通过 **用户配置目录** 或 **环境变量** 两种方式指定证书路径。

   方法 1：用户配置目录（推荐）

   ```bash
   mkdir -p ~/.config/xr_teleoperate/
   cp cert.pem key.pem ~/.config/xr_teleoperate/
   ```

   方法 2：环境变量方式

   ```bash
   echo 'export XR_TELEOP_CERT="your_file_path/cert.pem"' >> ~/.bashrc
   echo 'export XR_TELEOP_KEY="your_file_path/key.pem"' >> ~/.bashrc
   source ~/.bashrc
   ```

   方法 3：默认行为
    若不配置，Tele Imager 会从默认模块路径查找证书。



### 1.2 🔍 查找已连接的摄像头

运行以下命令可以自动发现已连接摄像头：

```bash
python -m teleimager.image_server --cf
# 或
teleimager-server --cf
```

你将看到类似下面的输出：
 ```bash
 (teleimager) unitree@ubuntu:~$ python -m teleimager.image_server --cf
 10:24:35:849900 INFO     ======================= Camera Discovery Start ================================== image_server.py:216
 10:24:35:851008 INFO     Found video devices: ['/dev/video0', '/dev/video1', '/dev/video2', '/dev/video3', image_server.py:217
                          '/dev/video4', '/dev/video5']                                                                        
 10:24:35:852089 INFO     Found RGB video devices: ['/dev/video0', '/dev/video2', '/dev/video4']            image_server.py:218
 10:24:35:852280 INFO     ------------------------- UVC Camera 1 ------------------------------------       image_server.py:227
 10:24:35:852575 INFO     video_path    : /dev/video0                                                       image_server.py:228
 10:24:35:852759 INFO     video_id      : 0                                                                 image_server.py:229
 10:24:35:852844 INFO     serial_number : 200901010002                                                      image_server.py:230
 10:24:35:852919 INFO     physical_path : /sys/devices/pci0000:00/0000:00:14.0/usb1/1-5/1-5:1.0             image_server.py:231
 10:24:35:852989 INFO     extra_info:                                                                       image_server.py:232
 10:24:35:853062 INFO         name: USB HDR Camera                                                          image_server.py:239
 10:24:35:853133 INFO         manufacturer: Generic                                                         image_server.py:239
 10:24:35:853198 INFO         serialNumber: 200901010002                                                    image_server.py:239
 10:24:35:853261 INFO         idProduct: 8272                                                               image_server.py:239
 10:24:35:853336 INFO         idVendor: 7749                                                                image_server.py:239
 10:24:35:853399 INFO         device_address: 4                                                             image_server.py:239
 10:24:35:853735 INFO         bus_number: 1                                                                 image_server.py:239
 10:24:35:853829 INFO         uid: 1:4                                                                      image_server.py:239
 ...
 10:24:36:033234 INFO         format: 480x640@30 MJPG                                                       image_server.py:243
 10:24:36:033249 INFO         format: 480x640@60 MJPG                                                       image_server.py:243
 ...
 10:24:36:034519 INFO     ------------------------- UVC Camera 2 ------------------------------------       image_server.py:227
 10:24:36:034551 INFO     video_path    : /dev/video2                                                       image_server.py:228
 10:24:36:034567 INFO     video_id      : 2                                                                 image_server.py:229
 10:24:36:034582 INFO     serial_number : 01.00.00                                                          image_server.py:230
 10:24:36:034595 INFO     physical_path : /sys/devices/pci0000:00/0000:00:14.0/usb1/1-11/1-11.1/1-11.1:1.0  image_server.py:231
 10:24:36:034608 INFO     extra_info:                                                                       image_server.py:232
 10:24:36:034622 INFO         name: Cherry Dual Camera                                                      image_server.py:239
 10:24:36:034635 INFO         manufacturer: DECXIN                                                          image_server.py:239
 10:24:36:034647 INFO         serialNumber: 01.00.00                                                        image_server.py:239
 10:24:36:034658 INFO         idProduct: 11599                                                              image_server.py:239
 10:24:36:034670 INFO         idVendor: 7119                                                                image_server.py:239
 10:24:36:034683 INFO         device_address: 9                                                             image_server.py:239
 10:24:36:034695 INFO         bus_number: 1                                                                 image_server.py:239
 10:24:36:034710 INFO         uid: 1:9                                                                      image_server.py:239
 ...
 10:24:36:435928 INFO         format: 480x1280@10 MJPG                                                      image_server.py:243
 10:24:36:435988 INFO         format: 480x1280@15 MJPG                                                      image_server.py:243
 10:24:36:436047 INFO         format: 480x1280@20 MJPG                                                      image_server.py:243
 10:24:36:436108 INFO         format: 480x1280@25 MJPG                                                      image_server.py:243
 10:24:36:436168 INFO         format: 480x1280@30 MJPG                                                      image_server.py:243
 10:24:36:436227 INFO         format: 480x1280@60 MJPG                                                      image_server.py:243
 10:24:36:436286 INFO         format: 480x1280@120 MJPG                                                     image_server.py:243
 ...
 10:24:36:524038 INFO     ------------------------- UVC Camera 3 ------------------------------------       image_server.py:227
 10:24:36:524203 INFO     video_path    : /dev/video4                                                       image_server.py:228
 10:24:36:524282 INFO     video_id      : 4                                                                 image_server.py:229
 10:24:36:524345 INFO     serial_number : 200901010001                                                      image_server.py:230
 10:24:36:524398 INFO     physical_path : /sys/devices/pci0000:00/0000:00:14.0/usb1/1-11/1-11.2/1-11.2:1.0  image_server.py:231
 10:24:36:524449 INFO     extra_info:                                                                       image_server.py:232
 10:24:36:524531 INFO         name: USB HDR Camera                                                          image_server.py:239
 10:24:36:524672 INFO         manufacturer: Generic                                                         image_server.py:239
 10:24:36:524734 INFO         serialNumber: 200901010001                                                    image_server.py:239
 10:24:36:524789 INFO         idProduct: 8272                                                               image_server.py:239
 10:24:36:524843 INFO         idVendor: 7749                                                                image_server.py:239
 10:24:36:524893 INFO         device_address: 10                                                            image_server.py:239
 10:24:36:524942 INFO         bus_number: 1                                                                 image_server.py:239
 10:24:36:524989 INFO         uid: 1:10                                                                     image_server.py:239
 10:24:36:688311 INFO         format: 240x320@30 MJPG                                                       image_server.py:243
 ...
 10:24:36:689031 INFO         format: 480x640@30 MJPG                                                       image_server.py:243
 10:24:36:689089 INFO         format: 480x640@60 MJPG                                                       image_server.py:243
 ...
 10:24:36:714374 INFO     =========================== Camera Discovery End ================================
 ```

如果存在 RealSense 设备并加上 `--rs` 参数，也会看到 RealSense 摄像头的搜索结果。

------

### 1.3 📡 启动图像服务器

根据摄像头搜索结果配置 `cam_config_server.yaml`。
 （示例配置见原文，此处不重复）

启动服务器：

```
python -m teleimager.image_server
python -m teleimager.image_server --rs   # 若使用 RealSense

# 或
teleimager-server
teleimager-server --rs
```



## 2. 图像客户端（Image Client）

该模块提供图像客户端，用于连接图像服务器并接收显示多路视频流。
 专为远程操作场景设计，与图像服务器配合使用。

所有可调用 API 都在 `# public api` 注释下。

------

### 2.1 🌀  ZMQ 使用方式

服务器运行后，在另一个终端启动客户端：

```
python -m teleimager.image_client
# 或
teleimager-client --host 127.0.0.1
```

若服务器运行在例如 `192.168.123.164` 的 G1 Jetson 上，则：

```
teleimager-client --host 192.168.123.164
```

然后你将看到各路 ZMQ 摄像头的视频窗口。

> 需要确保环境中安装了 opencv-python

### 2.2  🌀 WebRTC 使用方式

若使用 WebRTC，可通过浏览器访问：

```
https://<host_ip>:<webrtc_port>
# 例如
https://192.168.123.164:60001
```

点击左上角`start` 按钮



## 3. 🚀🚀🚀 自动启动服务

完成上述配置并测试成功后，可以通过以下脚本配置系统自动启动：

```
bash setup_autostart.sh
```

根据提示完成配置即可。



## 4. 🧠 设计原理



### 4.1 为什么需要多种摄像头识别方式？

在 Linux 系统中，一个摄像头设备可能对应多个不同的识别方式，例如：

- **物理路径 (physical_path)**
- **序列号 (serial_number)**
- **video 设备路径 (video_id → /dev/videoX)**

不同方式各有优缺点，因此 Tele Imager 同时支持三种方式，为用户提供最稳定、最灵活的摄像头识别能力。

------

#### 1. 物理路径（physical_path）

🎯 优点

- 不会随重启、插拔顺序变化
- 不依赖厂商是否提供唯一序列号
- 特别适合 **低成本摄像头序列号重复** 的场景
- 非常适合 **多摄像头固定部署**（比如机器人头部 + 双腕摄像头）

⚠️ 缺点

- 不灵活：换 USB 口就必须修改配置

------

#### 2. 序列号（serial_number）

🎯 优点

- 即使换 USB 口也不会变
- 识别准确性高
- 配置最简单
- RealSense 原生推荐使用 serial

⚠️ 缺点

- 许多廉价摄像头会“复用同一个序列号”
- 部分摄像头序列号格式奇怪或读取不稳定

------

#### 3. video 设备路径（video_id: /dev/videoX）

🎯 优点

- 使用最直接：看到 `/dev/video2` 就填 `video_id: 2`
- 适用于只有单摄像头或临时测试时使用

⚠️ 缺点（非常重要）

- 插拔顺序一变，videoX 就会改变
- 重启后也可能变
- Linux 内核枚举不同摄像头时顺序不固定
- 在多摄像头场景下极易错乱

------

#### 4. 三种方式定位

| 方式         | 稳定性 | 灵活度 | 推荐场景                           |
| ------------ | ------ | ------ | ---------------------------------- |
| **物理路径** | ⭐⭐⭐⭐⭐  | ⭐      | 机器人部署、多摄像头、低成本摄像头 |
| **序列号**   | ⭐⭐⭐⭐   | ⭐⭐⭐    | RealSense、拥有唯一序列号的摄像头  |
| **video_id** | ⭐⭐     | ⭐⭐⭐⭐⭐  | 临时调试、单摄像头使用             |



### 4.2 为什么需要三种图像传输方式？

本图像服务有两大用途：

1. **录制高质量数据 → 用于模型训练**
2. **实时可视化（XR / UI） → 用于调试、状态监控、远程操作界面**

不同传输场景（本地 / 局域网 / 远程网络）对延迟和带宽的要求不同，因此系统提供三种图像传输方式。

------

#### 1. ZeroMQ PUB–SUB

**适用于：服务器与客户端**不在同一台机器**，需要通过局域网传输时使用**。ZeroMQ 模式主要用于 **跨机器** 的图像传输，例如图像服务器在 A 电脑，数据记录程序在 B 电脑。

🎯 优点

- 在局域网（LAN）内传输高质量图像
- 尽可能减少包开销，提高吞吐
- 不牺牲图像质量的前提下保持低延迟

------

#### 2. WebRTC

**适用于：实时监控预览、VR 遥操作、UI 调试**。WebRTC 不是为训练数据设计的，而是为 **实时可视化流** 设计的：

🎯 优点

- 自动码率控制的前提下低延迟
- H.264(默认) / VP8
- 适用于浏览器、VR 设备等

------

#### 3. 共享内存

**适用于：服务器与客户端在同一台机器上运行时，获得最高性能**。如果图像服务器和图像查看器都运行在**同一台机器**，则 ZeroMQ 或 WebRTC 都会有不必要的开销：消息序列化（ZeroMQ）、视频编码/解码（WebRTC）和内核网络栈（socket buffer / scheduler）等。共享内存方式绕过这些开销。

🎯 优点

- 最高带宽（受限于内存带宽）
- μs 级延迟
- 可实现零拷贝（Zero-copy）或单拷贝
- CPU 占用低



### 4.3 Triple Ring Buffer 有什么好处？

- **非阻塞读写 (Non-blocking):**

  **写入者（Writer）** 不需要等待读取者读完。只要有空闲缓冲区，它就一直写。即便读取者卡住了，写入者也可以跳过被占用的槽位，继续在另外两个槽位间轮转。

  **读取者（Reader）** 不需要等待写入者写完。它总是能立即拿到最近一次**完整**写入的一帧数据。

- **消除“画面撕裂” (No Tearing):**

  由于读取和写入永远不会发生在同一个索引（`write` 函数中有专门的 `if write_index == read_index` 避让逻辑），读取者永远不会读到一张“写了一半”的图片。

- **始终最新 (Always Fresh):**

  与标准的队列（Queue）不同，队列是先进先出（FIFO），如果处理慢，队列会积压，导致读取者看到的画面有延迟。

  三重缓冲允许**丢帧**。如果写入太快，旧的帧会被覆盖，读取者永远拿到的是 `latest_index` 指向的那一帧。这对实时性至关重要。


## 5. 🧐 FAQ

1. 为什么 teleimager-server --cf 输出的信息中序列号等内容为 unknown？

    您可以尝试添加 `sudo` 权限运行该命令，某些摄像头需要更高权限才能读取完整信息。
    例如：

    ```bash
    sudo $(which teleimager-server) --cf
    ```

## 6. 🙏 Acknowledgement



部分代码参考了 https://github.com/ARCLab-MIT/beavr-bot