# -*- coding: utf-8 -*-
"""
config.py - 激光振镜投影仪 PC 端控制软件配置文件
==============================================
包含所有可配置参数的集中管理。
"""

import os
import json

# ============================================================
# 路径配置
# ============================================================
# 软件根目录（自动检测）
BASE_DIR = os.path.dirname(os.path.abspath(__file__))

# 标定数据保存目录
CALIBRATION_DIR = os.path.join(BASE_DIR, "calibration_data")

# YOLO 模型目录
MODEL_DIR = os.path.join(BASE_DIR, "models")

# ============================================================
# 摄像头配置
# ============================================================
# 摄像头设备索引（0 = 默认摄像头）
CAMERA_INDEX = 0

# 摄像头分辨率
CAMERA_WIDTH = 1280
CAMERA_HEIGHT = 720

# 摄像头帧率
CAMERA_FPS = 30

# 摄像头是否使用 MJPEG 格式（某些摄像头需要）
CAMERA_USE_MJPEG = False

# ============================================================
# ESP32-S3 WebSocket 通信配置
# ============================================================
# ESP32-S3 设备的 IP 地址
ESP32_IP = "192.168.4.1"

# WebSocket 服务器端口
WEBSOCKET_PORT = 81

# WebSocket 连接超时时间（秒）
WEBSOCKET_TIMEOUT = 10

# WebSocket 自动重连间隔（秒）
WEBSOCKET_RECONNECT_INTERVAL = 3

# WebSocket 最大重连次数（0 = 无限重连）
WEBSOCKET_MAX_RETRIES = 0

# ============================================================
# 标定配置
# ============================================================
# 标定点网格大小（3x3 = 9个点，4x4 = 16个点）
CALIBRATION_GRID_SIZE = 3

# 标定点之间的间距比例（相对于振镜坐标范围）
CALIBRATION_POINT_SPACING = 0.25

# 标定点投射持续时间（秒）
CALIBRATION_POINT_DURATION = 2.0

# 标定点检测时的二值化阈值
CALIBRATION_BINARY_THRESHOLD = 200

# 标定点检测区域大小（像素）
CALIBRATION_DETECT_RADIUS = 50

# 标定数据文件名
CALIBRATION_FILE = "calibration_data.json"

# ============================================================
# 安全参数
# ============================================================
# 激光安全距离阈值（像素），低于此距离自动关闭激光
SAFETY_DISTANCE_THRESHOLD = 50

# 激光最大投射速度（振镜坐标单位/帧）
LASER_MAX_SPEED = 50.0

# 激光安全模式（True = 默认开启安全限制）
SAFETY_MODE_ENABLED = True

# 目标丢失超时时间（秒），超时后自动关闭激光
TARGET_LOST_TIMEOUT = 2.0

# ============================================================
# YOLO 目标检测配置
# ============================================================
# YOLO 模型路径（相对于 MODEL_DIR）
YOLO_MODEL_PATH = "yolov8n.pt"

# 是否使用自定义训练模型
YOLO_USE_CUSTOM_MODEL = False

# 自定义模型路径（相对于 MODEL_DIR）
YOLO_CUSTOM_MODEL_PATH = "custom_model.pt"

# 检测置信度阈值
YOLO_CONFIDENCE_THRESHOLD = 0.5

# IoU 阈值（用于 NMS 非极大值抑制）
YOLO_IOU_THRESHOLD = 0.45

# 检测图像尺寸
YOLO_IMG_SIZE = 640

# 是否启用 ByteTrack 目标跟踪
YOLO_ENABLE_TRACKING = True

# 跟踪器类型（bytetrack / botsort）
YOLO_TRACKER_TYPE = "bytetrack"

# 跟踪置信度阈值
YOLO_TRACK_CONF = 0.3

# 跟踪 IoU 阈值
YOLO_TRACK_IOU = 0.5

# ============================================================
# 交互式激光按钮配置
# ============================================================
# 按钮形状（circle / rectangle）
BUTTON_SHAPE = "circle"

# 按钮大小（像素）
BUTTON_SIZE = 80

# 按钮数量
BUTTON_COUNT = 3

# 按钮之间的间距（像素）
BUTTON_SPACING = 150

# 肤色检测 HSV 范围（下界）
SKIN_HSV_LOWER = [0, 30, 60]

# 肤色检测 HSV 范围（上界）
SKIN_HSV_UPPER = [20, 150, 255]

# 手指检测最小轮廓面积
FINGER_MIN_CONTOUR_AREA = 500

# 运动检测帧差阈值
MOTION_THRESHOLD = 25

# 运动检测最小变化像素数
MOTION_MIN_PIXELS = 100

# 点击判定时间窗口（秒）
CLICK_TIME_WINDOW = 0.5

# ============================================================
# 显示配置
# ============================================================
# 显示窗口名称
WINDOW_NAME_MAIN = "激光振镜投影仪 - 主画面"
WINDOW_NAME_CALIBRATION = "激光振镜投影仪 - 标定"
WINDOW_NAME_INTERACTIVE = "激光振镜投影仪 - 交互"

# 显示缩放比例（1.0 = 原始大小）
DISPLAY_SCALE = 1.0

# 是否显示检测框
DISPLAY_SHOW_BOXES = True

# 是否显示跟踪 ID
DISPLAY_SHOW_TRACK_IDS = True

# 是否显示 FPS
DISPLAY_SHOW_FPS = True

# ============================================================
# 日志配置
# ============================================================
# 日志级别（DEBUG / INFO / WARNING / ERROR / CRITICAL）
LOG_LEVEL = "INFO"

# 日志格式
LOG_FORMAT = "%(asctime)s [%(levelname)s] %(name)s: %(message)s"

# 日志文件路径（None = 不保存到文件）
LOG_FILE = os.path.join(BASE_DIR, "laser_control.log")


def get_calibration_file_path():
    """获取标定数据文件的完整路径。"""
    os.makedirs(CALIBRATION_DIR, exist_ok=True)
    return os.path.join(CALIBRATION_DIR, CALIBRATION_FILE)


def get_model_path():
    """获取 YOLO 模型的完整路径。"""
    if YOLO_USE_CUSTOM_MODEL:
        return os.path.join(MODEL_DIR, YOLO_CUSTOM_MODEL_PATH)
    return os.path.join(MODEL_DIR, YOLO_MODEL_PATH)


def load_config_from_file(filepath):
    """
    从 JSON 文件加载配置，覆盖默认值。

    Args:
        filepath: 配置文件路径

    Returns:
        dict: 配置字典
    """
    if not os.path.exists(filepath):
        return {}

    with open(filepath, "r", encoding="utf-8") as f:
        user_config = json.load(f)

    return user_config


def apply_config_overrides(config_dict):
    """
    应用用户配置覆盖默认值。

    Args:
        config_dict: 用户配置字典
    """
    global ESP32_IP, WEBSOCKET_PORT, CAMERA_INDEX
    global CAMERA_WIDTH, CAMERA_HEIGHT, CAMERA_FPS
    global YOLO_CONFIDENCE_THRESHOLD, YOLO_IOU_THRESHOLD
    global YOLO_ENABLE_TRACKING, SAFETY_MODE_ENABLED
    global SAFETY_DISTANCE_THRESHOLD, TARGET_LOST_TIMEOUT
    global CALIBRATION_GRID_SIZE, LOG_LEVEL

    if "esp32_ip" in config_dict:
        ESP32_IP = config_dict["esp32_ip"]
    if "websocket_port" in config_dict:
        WEBSOCKET_PORT = config_dict["websocket_port"]
    if "camera_index" in config_dict:
        CAMERA_INDEX = config_dict["camera_index"]
    if "camera_width" in config_dict:
        CAMERA_WIDTH = config_dict["camera_width"]
    if "camera_height" in config_dict:
        CAMERA_HEIGHT = config_dict["camera_height"]
    if "camera_fps" in config_dict:
        CAMERA_FPS = config_dict["camera_fps"]
    if "yolo_confidence_threshold" in config_dict:
        YOLO_CONFIDENCE_THRESHOLD = config_dict["yolo_confidence_threshold"]
    if "yolo_iou_threshold" in config_dict:
        YOLO_IOU_THRESHOLD = config_dict["yolo_iou_threshold"]
    if "yolo_enable_tracking" in config_dict:
        YOLO_ENABLE_TRACKING = config_dict["yolo_enable_tracking"]
    if "safety_mode_enabled" in config_dict:
        SAFETY_MODE_ENABLED = config_dict["safety_mode_enabled"]
    if "safety_distance_threshold" in config_dict:
        SAFETY_DISTANCE_THRESHOLD = config_dict["safety_distance_threshold"]
    if "target_lost_timeout" in config_dict:
        TARGET_LOST_TIMEOUT = config_dict["target_lost_timeout"]
    if "calibration_grid_size" in config_dict:
        CALIBRATION_GRID_SIZE = config_dict["calibration_grid_size"]
    if "log_level" in config_dict:
        LOG_LEVEL = config_dict["log_level"]
