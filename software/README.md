# 激光振镜投影仪 PC 端控制软件

基于 YOLO 目标检测 + 坐标标定 + WebSocket 通信的激光振镜投影仪 PC 端控制软件。

## 功能特性

- **坐标标定**：自动投射标定点，通过摄像头捕捉计算透视变换矩阵
- **目标跟踪**：使用 YOLOv8 实时检测目标，通过激光跟踪
- **交互按钮**：投射虚拟激光按钮，支持手指检测和点击交互
- **WebSocket 通信**：与 ESP32-S3 振镜控制器实时通信
- **安全保护**：内置安全模式，目标丢失自动关闭激光

## 环境要求

- Python 3.8+
- 摄像头（USB 摄像头或笔记本内置摄像头）
- ESP32-S3 振镜控制器（已烧录固件并运行 WebSocket 服务）
- 同一局域网（或 ESP32-S3 的 AP 热点）

## 安装

### 1. 克隆项目

```bash
cd GalvoEye-S3/software
```

### 2. 创建虚拟环境（推荐）

```bash
python -m venv venv

# Linux / macOS
source venv/bin/activate

# Windows
venv\Scripts\activate
```

### 3. 安装依赖

```bash
pip install -r requirements.txt
```

### 4. 下载 YOLO 模型

首次运行时，YOLOv8n 模型会自动下载。如需使用自定义模型，将 `.pt` 文件放入 `models/` 目录。

```bash
mkdir -p models
# 如果需要手动下载预训练模型
wget https://github.com/ultralytics/assets/releases/download/v0.0.0/yolov8n.pt -P models/
```

## 使用方法

### 标定模式

首次使用前必须进行坐标标定，建立摄像头像素坐标与振镜坐标之间的映射关系。

```bash
python main.py --mode calibrate
```

可选参数：

```bash
# 指定 ESP32 IP 地址
python main.py --mode calibrate --host 192.168.4.1

# 使用 4x4 标定网格（16个点）
python main.py --mode calibrate --grid-size 4

# 指定摄像头
python main.py --mode calibrate --camera 1
```

**标定流程：**

1. 程序连接 ESP32-S3 WebSocket 服务
2. 摄像头打开并投射标定点
3. 逐个投射 9 个（3x3）或 16 个（4x4）标定点
4. 摄像头检测激光亮点位置
5. 计算透视变换矩阵
6. 评估标定精度并保存数据

标定数据保存在 `calibration_data/calibration_data.json`。

### 目标跟踪模式

使用 YOLO 检测目标并通过激光跟踪。

```bash
python main.py --mode track
```

可选参数：

```bash
# 指定自定义模型
python main.py --mode track --model models/custom_model.pt

# 调整检测置信度
python main.py --mode track --confidence 0.7

# 禁用目标跟踪（仅检测）
python main.py --mode track --no-tracking

# 禁用安全模式
python main.py --mode track --no-safety
```

**运行时快捷键：**

| 按键 | 功能 |
|------|------|
| `q` / `ESC` | 退出程序 |
| `s` | 切换安全模式 |
| `l` | 开关激光 |
| `h` | 振镜回到原点 |

### 交互式按钮模式

投射虚拟激光按钮，通过手指检测实现交互。

```bash
python main.py --mode interactive
```

**运行时快捷键：**

| 按键 | 功能 |
|------|------|
| `q` / `ESC` | 退出程序 |
| `r` | 重置按钮布局 |
| `+` / `=` | 增大按钮 |
| `-` | 缩小按钮 |

## 配置说明

所有配置参数集中在 `config.py` 中。可以通过命令行参数覆盖，也可以创建 JSON 配置文件。

### 创建自定义配置文件

创建 `my_config.json`：

```json
{
  "esp32_ip": "192.168.1.100",
  "websocket_port": 81,
  "camera_index": 0,
  "camera_width": 1280,
  "camera_height": 720,
  "yolo_confidence_threshold": 0.6,
  "yolo_enable_tracking": true,
  "safety_mode_enabled": true,
  "safety_distance_threshold": 50,
  "calibration_grid_size": 3,
  "log_level": "INFO"
}
```

使用自定义配置：

```bash
python main.py --mode track --config-file my_config.json
```

### 主要配置项

| 配置项 | 默认值 | 说明 |
|--------|--------|------|
| `ESP32_IP` | `192.168.4.1` | ESP32-S3 的 IP 地址 |
| `WEBSOCKET_PORT` | `81` | WebSocket 端口 |
| `CAMERA_INDEX` | `0` | 摄像头设备索引 |
| `CAMERA_WIDTH` | `1280` | 摄像头宽度 |
| `CAMERA_HEIGHT` | `720` | 摄像头高度 |
| `YOLO_CONFIDENCE_THRESHOLD` | `0.5` | 检测置信度阈值 |
| `YOLO_ENABLE_TRACKING` | `True` | 启用 ByteTrack 跟踪 |
| `CALIBRATION_GRID_SIZE` | `3` | 标定网格大小 |
| `SAFETY_MODE_ENABLED` | `True` | 安全模式开关 |
| `SAFETY_DISTANCE_THRESHOLD` | `50` | 安全距离阈值（像素） |
| `TARGET_LOST_TIMEOUT` | `2.0` | 目标丢失超时（秒） |

## 项目结构

```
software/
  main.py          # 主程序入口
  config.py        # 配置文件
  communicator.py  # WebSocket 通信模块
  calibrator.py    # 坐标标定模块
  detector.py      # YOLO 目标检测模块
  interactive.py   # 交互式激光按钮模块
  requirements.txt # Python 依赖
  README.md        # 使用说明
  models/          # YOLO 模型目录
  calibration_data/ # 标定数据目录
```

## 常见问题

### 1. 无法连接 ESP32-S3

- 确认 ESP32-S3 已开机并运行 WebSocket 服务
- 确认 PC 已连接到同一 WiFi 网络（或 ESP32 的 AP）
- 尝试 ping ESP32 IP 地址：`ping 192.168.4.1`
- 检查防火墙设置

### 2. 摄像头无法打开

- 确认摄像头已连接
- 尝试更换摄像头索引：`--camera 1` 或 `--camera 2`
- Linux 用户可能需要添加权限：`sudo usermod -a -G video $USER`
- 检查是否有其他程序占用摄像头

### 3. 标定精度差

- 确保摄像头和振镜投影面固定不动
- 调整环境光线，避免强光干扰
- 尝试增加标定点数量：`--grid-size 4`
- 检查激光亮度是否足够被摄像头捕捉
- 确保标定过程中没有振动

### 4. YOLO 模型加载失败

- 确认模型文件存在于 `models/` 目录
- 首次运行需要联网下载预训练模型
- 检查磁盘空间是否充足

### 5. 检测不到目标

- 降低置信度阈值：`--confidence 0.3`
- 确保目标在摄像头视野内
- 调整环境光线
- 考虑使用自定义训练模型提高特定目标检测率

### 6. 激光跟踪不准确

- 重新进行坐标标定
- 检查标定误差是否过大（>10 像素）
- 确保摄像头和投影面相对位置未改变
- 降低目标移动速度

## 开发调试

启用调试日志：

```bash
python main.py --mode track --debug
```

日志文件默认保存在 `laser_control.log`。
