# -*- coding: utf-8 -*-
"""
main.py - 激光振镜投影仪 PC 端控制软件主程序
===============================================
主程序入口，支持三种运行模式：
  - calibrate:  坐标标定模式
  - track:      目标跟踪模式（YOLO 检测 + 激光跟踪）
  - interactive: 交互式激光按钮模式

使用方法：
  python main.py --mode calibrate
  python main.py --mode track
  python main.py --mode interactive
"""

import argparse
import asyncio
import logging
import signal
import sys
from typing import Optional

import cv2
import numpy as np

import config
from communicator import LaserCommunicator
from calibrator import Calibrator
from detector import ObjectDetector
from interactive import InteractiveManager

logger = logging.getLogger(__name__)


def setup_logging():
    """配置日志系统。"""
    log_level = getattr(logging, config.LOG_LEVEL.upper(), logging.INFO)

    # 创建日志格式
    formatter = logging.Formatter(config.LOG_FORMAT)

    # 控制台日志
    console_handler = logging.StreamHandler(sys.stdout)
    console_handler.setLevel(log_level)
    console_handler.setFormatter(formatter)

    # 根日志器
    root_logger = logging.getLogger()
    root_logger.setLevel(log_level)
    root_logger.addHandler(console_handler)

    # 文件日志（如果配置了）
    if config.LOG_FILE:
        try:
            file_handler = logging.FileHandler(
                config.LOG_FILE, encoding="utf-8",
            )
            file_handler.setLevel(log_level)
            file_handler.setFormatter(formatter)
            root_logger.addHandler(file_handler)
            logger.info("日志文件: %s", config.LOG_FILE)
        except Exception as e:
            logger.warning("无法创建日志文件: %s", e)


def parse_args():
    """
    解析命令行参数。

    Returns:
        解析后的参数对象
    """
    parser = argparse.ArgumentParser(
        description="激光振镜投影仪 PC 端控制软件",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
使用示例:
  # 标定模式
  python main.py --mode calibrate

  # 目标跟踪模式
  python main.py --mode track

  # 交互式按钮模式
  python main.py --mode interactive

  # 指定 ESP32 IP
  python main.py --mode track --host 192.168.1.100

  # 指定摄像头
  python main.py --mode calibrate --camera 1

  # 使用自定义 YOLO 模型
  python main.py --mode track --model custom_model.pt
        """,
    )

    parser.add_argument(
        "--mode", "-m",
        type=str,
        choices=["calibrate", "track", "interactive"],
        default="track",
        help="运行模式: calibrate(标定), track(跟踪), interactive(交互)",
    )

    parser.add_argument(
        "--host",
        type=str,
        default=None,
        help=f"ESP32-S3 IP 地址 (默认: {config.ESP32_IP})",
    )

    parser.add_argument(
        "--port", "-p",
        type=int,
        default=None,
        help=f"WebSocket 端口 (默认: {config.WEBSOCKET_PORT})",
    )

    parser.add_argument(
        "--camera", "-c",
        type=int,
        default=None,
        help=f"摄像头索引 (默认: {config.CAMERA_INDEX})",
    )

    parser.add_argument(
        "--model",
        type=str,
        default=None,
        help="YOLO 模型文件路径",
    )

    parser.add_argument(
        "--confidence",
        type=float,
        default=None,
        help=f"检测置信度阈值 (默认: {config.YOLO_CONFIDENCE_THRESHOLD})",
    )

    parser.add_argument(
        "--no-tracking",
        action="store_true",
        help="禁用目标跟踪（仅检测）",
    )

    parser.add_argument(
        "--grid-size",
        type=int,
        default=None,
        help=f"标定网格大小 (默认: {config.CALIBRATION_GRID_SIZE})",
    )

    parser.add_argument(
        "--config-file",
        type=str,
        default=None,
        help="自定义配置文件路径 (JSON)",
    )

    parser.add_argument(
        "--no-safety",
        action="store_true",
        help="禁用安全模式",
    )

    parser.add_argument(
        "--debug",
        action="store_true",
        help="启用调试日志",
    )

    return parser.parse_args()


def apply_command_line_args(args):
    """
    应用命令行参数覆盖配置。

    Args:
        args: 命令行参数对象
    """
    if args.host:
        config.ESP32_IP = args.host
    if args.port:
        config.WEBSOCKET_PORT = args.port
    if args.camera is not None:
        config.CAMERA_INDEX = args.camera
    if args.confidence is not None:
        config.YOLO_CONFIDENCE_THRESHOLD = args.confidence
    if args.no_tracking:
        config.YOLO_ENABLE_TRACKING = False
    if args.grid_size:
        config.CALIBRATION_GRID_SIZE = args.grid_size
    if args.no_safety:
        config.SAFETY_MODE_ENABLED = False
    if args.debug:
        config.LOG_LEVEL = "DEBUG"

    # 如果指定了自定义模型
    if args.model:
        config.YOLO_USE_CUSTOM_MODEL = True
        config.YOLO_CUSTOM_MODEL_PATH = args.model


def open_camera(camera_index: int) -> Optional[cv2.VideoCapture]:
    """
    打开摄像头。

    Args:
        camera_index: 摄像头设备索引

    Returns:
        VideoCapture 对象，失败返回 None
    """
    cap = cv2.VideoCapture(camera_index)

    if not cap.isOpened():
        logger.error("无法打开摄像头 (索引: %d)", camera_index)
        logger.error("请检查摄像头是否已连接")
        return None

    # 设置分辨率
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, config.CAMERA_WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, config.CAMERA_HEIGHT)
    cap.set(cv2.CAP_PROP_FPS, config.CAMERA_FPS)

    if config.CAMERA_USE_MJPEG:
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))

    actual_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    actual_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    actual_fps = cap.get(cv2.CAP_PROP_FPS)

    logger.info("摄像头已打开:")
    logger.info("  分辨率: %dx%d", actual_w, actual_h)
    logger.info("  帧率: %.1f", actual_fps)

    return cap


# ============================================================
# 标定模式
# ============================================================

async def run_calibrate_mode(args):
    """
    运行标定模式。

    执行坐标标定流程：
    1. 连接 ESP32-S3
    2. 打开摄像头
    3. 投射标定点并检测
    4. 计算变换矩阵
    5. 保存标定数据

    Args:
        args: 命令行参数
    """
    logger.info("=" * 60)
    logger.info("  模式: 坐标标定")
    logger.info("=" * 60)

    # 创建通信器
    comm = LaserCommunicator(
        host=config.ESP32_IP,
        port=config.WEBSOCKET_PORT,
    )

    # 创建标定器
    calibrator = Calibrator(
        communicator=comm,
        camera_index=config.CAMERA_INDEX,
        grid_size=config.CALIBRATION_GRID_SIZE,
    )

    # 尝试连接
    logger.info("正在连接 ESP32-S3...")
    connected = await comm.connect()

    if not connected:
        logger.error("无法连接到 ESP32-S3 (%s:%d)", config.ESP32_IP, config.WEBSOCKET_PORT)
        logger.error("请检查:")
        logger.error("  1. ESP32-S3 是否已开机")
        logger.error("  2. WiFi 是否已连接")
        logger.error("  3. IP 地址是否正确: %s", config.ESP32_IP)
        await comm.disconnect()
        return

    try:
        # 执行标定
        success = await calibrator.run_calibration()

        if success:
            logger.info("标定成功完成!")
            logger.info("标定数据已保存到: %s", config.get_calibration_file_path())
        else:
            logger.error("标定失败，请检查日志排查问题")

    except KeyboardInterrupt:
        logger.info("用户中断标定")
    except Exception as e:
        logger.error("标定过程异常: %s", e)
    finally:
        await comm.disconnect()


# ============================================================
# 目标跟踪模式
# ============================================================

async def run_track_mode(args):
    """
    运行目标跟踪模式。

    使用 YOLO 检测目标并通过激光跟踪：
    1. 连接 ESP32-S3
    2. 加载标定数据
    3. 加载 YOLO 模型
    4. 打开摄像头
    5. 实时检测 -> 坐标映射 -> 发送振镜坐标

    Args:
        args: 命令行参数
    """
    logger.info("=" * 60)
    logger.info("  模式: 目标跟踪")
    logger.info("=" * 60)

    # 创建通信器
    comm = LaserCommunicator(
        host=config.ESP32_IP,
        port=config.WEBSOCKET_PORT,
    )

    # 创建标定器（用于坐标映射）
    calibrator = Calibrator(communicator=comm)

    # 创建检测器
    model_path = args.model if args.model else config.get_model_path()
    detector = ObjectDetector(
        model_path=model_path,
        calibrator=calibrator,
        enable_tracking=config.YOLO_ENABLE_TRACKING,
    )

    # 尝试连接 ESP32-S3
    logger.info("正在连接 ESP32-S3...")
    connected = await comm.connect()

    if not connected:
        logger.warning("无法连接到 ESP32-S3，将以离线模式运行（仅检测）")
        logger.warning("激光跟踪功能不可用")

    # 加载标定数据
    logger.info("正在加载标定数据...")
    calib_loaded = calibrator.load_calibration()
    if not calib_loaded:
        logger.warning("未找到标定数据，坐标映射功能不可用")
        logger.warning("请先运行标定模式: python main.py --mode calibrate")
    else:
        logger.info("标定数据加载成功")

    # 加载 YOLO 模型
    logger.info("正在加载 YOLO 模型...")
    model_loaded = detector.load_model()
    if not model_loaded:
        logger.error("YOLO 模型加载失败")
        await comm.disconnect()
        return

    # 打开摄像头
    logger.info("正在打开摄像头...")
    cap = open_camera(config.CAMERA_INDEX)
    if cap is None:
        await comm.disconnect()
        return

    # 安全模式设置
    if connected and config.SAFETY_MODE_ENABLED:
        await comm.set_safety_mode("normal")
        logger.info("安全模式已开启")

    logger.info("开始目标跟踪...")
    logger.info("按 'q' 退出, 's' 切换安全模式, 'l' 开关激光")

    # 跟踪状态
    laser_on = False
    safety_mode = config.SAFETY_MODE_ENABLED
    prev_target: Optional[tuple] = None

    try:
        while True:
            # 读取帧
            ret, frame = cap.read()
            if not ret:
                logger.warning("读取帧失败")
                await asyncio.sleep(0.01)
                continue

            # 执行目标检测
            if config.YOLO_ENABLE_TRACKING:
                detections = detector.detect_with_tracking(frame)
            else:
                detections = detector.detect(frame)

            # 获取主要目标
            target = detector.get_primary_target(detections)

            # 处理目标跟踪
            if target and target.galvo_center:
                gx, gy = target.galvo_center

                # 安全检查：目标移动速度限制
                if prev_target and config.SAFETY_MODE_ENABLED:
                    dx = abs(gx - prev_target[0])
                    dy = abs(gy - prev_target[1])
                    speed = np.sqrt(dx * dx + dy * dy)

                    if speed > config.LASER_MAX_SPEED:
                        logger.warning(
                            "目标移动速度过快 (%.1f)，跳过此帧",
                            speed,
                        )
                        prev_target = (gx, gy)
                        continue

                # 发送目标坐标到振镜
                if connected and laser_on:
                    await comm.send_target(gx, gy)

                prev_target = (gx, gy)
            else:
                # 目标丢失
                if detector.is_target_lost():
                    if connected and laser_on:
                        await comm.laser_off()
                        logger.debug("目标丢失，激光已关闭")
                    prev_target = None

            # 可视化
            vis = detector.visualize(frame, detections)

            # 显示安全模式状态
            status_color = (0, 255, 0) if safety_mode else (0, 0, 255)
            status_text = f"Safety: {'ON' if safety_mode else 'OFF'}"
            cv2.putText(
                vis, status_text, (10, 90),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                status_color, 2,
            )

            laser_color = (0, 255, 0) if laser_on else (0, 0, 255)
            laser_text = f"Laser: {'ON' if laser_on else 'OFF'}"
            cv2.putText(
                vis, laser_text, (10, 115),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                laser_color, 2,
            )

            # 显示连接状态
            conn_color = (0, 255, 0) if connected else (0, 0, 255)
            conn_text = f"WS: {'Connected' if connected else 'Disconnected'}"
            cv2.putText(
                vis, conn_text, (10, 140),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                conn_color, 2,
            )

            # 显示窗口
            window_name = config.WINDOW_NAME_MAIN
            cv2.imshow(window_name, vis)

            # 键盘控制
            key = cv2.waitKey(1) & 0xFF
            if key == ord("q") or key == 27:  # q 或 ESC
                logger.info("用户退出")
                break
            elif key == ord("s"):
                # 切换安全模式
                safety_mode = not safety_mode
                if connected:
                    await comm.set_safety_mode("normal" if safety_mode else "off")
                logger.info("安全模式: %s", "开启" if safety_mode else "关闭")
            elif key == ord("l"):
                # 切换激光
                laser_on = not laser_on
                if connected:
                    if laser_on:
                        await comm.laser_on()
                    else:
                        await comm.laser_off()
                logger.info("激光: %s", "开启" if laser_on else "关闭")
            elif key == ord("h"):
                # 回到原点
                if connected:
                    await comm.go_home()
                logger.info("振镜回到原点")

    except KeyboardInterrupt:
        logger.info("用户中断")
    except Exception as e:
        logger.error("跟踪过程异常: %s", e)
    finally:
        # 清理
        logger.info("正在清理...")
        if connected:
            await comm.laser_off()
            await comm.clear_display()
            await comm.disconnect()
        cap.release()
        cv2.destroyAllWindows()
        logger.info("目标跟踪模式已退出")


# ============================================================
# 交互式按钮模式
# ============================================================

async def run_interactive_mode(args):
    """
    运行交互式激光按钮模式。

    投射虚拟按钮，通过手指检测实现交互：
    1. 连接 ESP32-S3
    2. 加载标定数据
    3. 打开摄像头
    4. 创建按钮布局
    5. 实时检测手指交互

    Args:
        args: 命令行参数
    """
    logger.info("=" * 60)
    logger.info("  模式: 交互式激光按钮")
    logger.info("=" * 60)

    # 创建通信器
    comm = LaserCommunicator(
        host=config.ESP32_IP,
        port=config.WEBSOCKET_PORT,
    )

    # 创建标定器
    calibrator = Calibrator(communicator=comm)

    # 创建交互管理器
    manager = InteractiveManager(
        communicator=comm,
        calibrator=calibrator,
    )

    # 定义按钮回调函数
    def on_button_click(button_id: str, state: str):
        """按钮点击回调。"""
        logger.info(">>> 按钮事件: id=%s, state=%s", button_id, state)

    # 尝试连接
    logger.info("正在连接 ESP32-S3...")
    connected = await comm.connect()

    if not connected:
        logger.warning("无法连接到 ESP32-S3，将以离线模式运行")

    # 加载标定数据
    logger.info("正在加载标定数据...")
    calib_loaded = calibrator.load_calibration()
    if not calib_loaded:
        logger.warning("未找到标定数据，按钮坐标映射不可用")
        logger.warning("请先运行标定模式: python main.py --mode calibrate")

    # 打开摄像头
    logger.info("正在打开摄像头...")
    cap = open_camera(config.CAMERA_INDEX)
    if cap is None:
        await comm.disconnect()
        return

    # 创建默认按钮布局
    buttons = manager.create_default_buttons()
    for btn in buttons:
        btn.callback = on_button_click

    # 开启激光
    if connected:
        await comm.laser_on()
        logger.info("激光已开启")

    logger.info("开始交互式按钮模式...")
    logger.info("按 'q' 退出, 'r' 重置按钮, '+'/'-' 调整按钮大小")

    # 按钮投射任务
    draw_task = None
    if connected:
        draw_task = asyncio.create_task(_continuous_draw_buttons(manager))

    try:
        while True:
            # 读取帧
            ret, frame = cap.read()
            if not ret:
                logger.warning("读取帧失败")
                await asyncio.sleep(0.01)
                continue

            # 处理帧（检测手指交互）
            vis = manager.process_frame(frame)

            # 显示连接状态
            conn_color = (0, 255, 0) if connected else (0, 0, 255)
            conn_text = f"WS: {'Connected' if connected else 'Disconnected'}"
            cv2.putText(
                vis, conn_text, (10, vis.shape[0] - 20),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                conn_color, 2,
            )

            # 显示窗口
            window_name = config.WINDOW_NAME_INTERACTIVE
            cv2.imshow(window_name, vis)

            # 键盘控制
            key = cv2.waitKey(1) & 0xFF
            if key == ord("q") or key == 27:
                logger.info("用户退出")
                break
            elif key == ord("r"):
                # 重置按钮
                manager.reset()
                manager.create_default_buttons()
                for btn in manager.buttons:
                    btn.callback = on_button_click
                logger.info("按钮已重置")
            elif key == ord("+") or key == ord("="):
                # 增大按钮
                for btn in manager.buttons:
                    btn.size = min(btn.size + 10, 200)
                    if calibrator.inverse_matrix is not None:
                        btn.update_galvo_coords(calibrator)
                logger.info("按钮大小已增大")
            elif key == ord("-"):
                # 缩小按钮
                for btn in manager.buttons:
                    btn.size = max(btn.size - 10, 30)
                    if calibrator.inverse_matrix is not None:
                        btn.update_galvo_coords(calibrator)
                logger.info("按钮大小已缩小")

    except KeyboardInterrupt:
        logger.info("用户中断")
    except Exception as e:
        logger.error("交互模式异常: %s", e)
    finally:
        # 清理
        logger.info("正在清理...")
        if draw_task:
            draw_task.cancel()
            try:
                await draw_task
            except asyncio.CancelledError:
                pass

        if connected:
            await comm.laser_off()
            await comm.clear_display()
            await comm.disconnect()

        cap.release()
        cv2.destroyAllWindows()
        logger.info("交互式按钮模式已退出")


async def _continuous_draw_buttons(manager: InteractiveManager):
    """
    持续投射按钮的后台任务。

    Args:
        manager: 交互管理器
    """
    try:
        while True:
            await manager.draw_buttons()
            await asyncio.sleep(0.15)
    except asyncio.CancelledError:
        pass
    except Exception as e:
        logger.error("按钮投射异常: %s", e)


# ============================================================
# 主函数
# ============================================================

async def main():
    """主函数入口。"""
    # 解析命令行参数
    args = parse_args()

    # 加载自定义配置文件
    if args.config_file:
        user_config = config.load_config_from_file(args.config_file)
        config.apply_config_overrides(user_config)
        logger.info("已加载自定义配置: %s", args.config_file)

    # 应用命令行参数
    apply_command_line_args(args)

    # 配置日志
    setup_logging()

    # 打印启动信息
    logger.info("=" * 60)
    logger.info("  激光振镜投影仪 PC 端控制软件")
    logger.info("  Laser Galvo Projector PC Controller")
    logger.info("=" * 60)
    logger.info("运行模式: %s", args.mode)
    logger.info("ESP32 地址: %s:%d", config.ESP32_IP, config.WEBSOCKET_PORT)
    logger.info("摄像头索引: %d", config.CAMERA_INDEX)
    logger.info("安全模式: %s", "开启" if config.SAFETY_MODE_ENABLED else "关闭")

    # 根据模式运行
    if args.mode == "calibrate":
        await run_calibrate_mode(args)
    elif args.mode == "track":
        await run_track_mode(args)
    elif args.mode == "interactive":
        await run_interactive_mode(args)

    logger.info("程序结束")


def entry_point():
    """程序入口点（同步包装器）。"""
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\n程序被用户中断")
    except Exception as e:
        logging.error("程序异常退出: %s", e)
        sys.exit(1)


if __name__ == "__main__":
    entry_point()
