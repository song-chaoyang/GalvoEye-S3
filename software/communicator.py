# -*- coding: utf-8 -*-
"""
communicator.py - WebSocket 通信模块
======================================
负责 PC 端与 ESP32-S3 振镜控制器之间的 WebSocket 通信。
支持发送目标坐标、绘图指令、安全模式切换，以及接收设备状态。
内置自动重连机制。
"""

import asyncio
import json
import logging
import time
from typing import Optional, Callable, Any

import websockets
from websockets.exceptions import (
    ConnectionClosedError,
    ConnectionClosedOK,
    WebSocketException,
)

import config

logger = logging.getLogger(__name__)


class LaserCommunicator:
    """
    激光振镜 WebSocket 通信器。

    通过 WebSocket 协议与 ESP32-S3 振镜控制器通信，
    发送坐标和绘图指令，接收设备状态反馈。
    """

    # 指令类型常量
    CMD_TARGET = "target"          # 发送目标坐标
    CMD_DRAW_POINT = "draw_point"  # 绘制点
    CMD_DRAW_LINE = "draw_line"    # 绘制线
    CMD_DRAW_CIRCLE = "draw_circle"  # 绘制圆
    CMD_DRAW_RECT = "draw_rect"    # 绘制矩形
    CMD_CLEAR = "clear"            # 清除画面
    CMD_SAFETY = "safety"          # 安全模式切换
    CMD_HOME = "home"              # 回到原点
    CMD_LASER_ON = "laser_on"      # 开启激光
    CMD_LASER_OFF = "laser_off"    # 关闭激光
    CMD_CALIBRATE = "calibrate"    # 标定点投射
    CMD_STATUS = "status"          # 请求状态

    def __init__(
        self,
        host: str = None,
        port: int = None,
        reconnect_interval: float = None,
        max_retries: int = None,
    ):
        """
        初始化通信器。

        Args:
            host: ESP32-S3 的 IP 地址
            port: WebSocket 端口
            reconnect_interval: 重连间隔（秒）
            max_retries: 最大重连次数（0 = 无限）
        """
        self.host = host or config.ESP32_IP
        self.port = port or config.WEBSOCKET_PORT
        self.reconnect_interval = reconnect_interval or config.WEBSOCKET_RECONNECT_INTERVAL
        self.max_retries = max_retries if max_retries is not None else config.WEBSOCKET_MAX_RETRIES

        # WebSocket 连接对象
        self._ws: Optional[websockets.WebSocketClientProtocol] = None
        # 连接状态标志
        self._connected = False
        # 重连计数器
        self._retry_count = 0
        # 是否正在运行
        self._running = False
        # 接收消息回调
        self._message_callback: Optional[Callable[[dict], None]] = None
        # 状态变化回调
        self._status_callback: Optional[Callable[[bool], None]] = None
        # 后台接收任务
        self._receive_task: Optional[asyncio.Task] = None
        # 发送队列锁
        self._send_lock = asyncio.Lock()
        # 重连任务（防止重复创建）
        self._reconnect_task: Optional[asyncio.Task] = None

        # WebSocket URI
        self._uri = f"ws://{self.host}:{self.port}"

        logger.info("通信器初始化完成，目标地址: %s", self._uri)

    @property
    def is_connected(self) -> bool:
        """返回当前是否已连接。"""
        return self._connected

    def on_message(self, callback: Callable[[dict], None]):
        """
        注册消息接收回调函数。

        Args:
            callback: 回调函数，接收解析后的 JSON 字典
        """
        self._message_callback = callback

    def on_status_change(self, callback: Callable[[bool], None]):
        """
        注册连接状态变化回调函数。

        Args:
            callback: 回调函数，参数为连接状态（True/False）
        """
        self._status_callback = callback

    async def connect(self) -> bool:
        """
        连接到 ESP32-S3 WebSocket 服务器。

        Returns:
            bool: 连接是否成功
        """
        logger.info("正在连接到 %s ...", self._uri)

        try:
            self._ws = await asyncio.wait_for(
                websockets.connect(
                    self._uri,
                    ping_interval=10,
                    ping_timeout=5,
                    close_timeout=3,
                ),
                timeout=config.WEBSOCKET_TIMEOUT,
            )
            self._connected = True
            self._retry_count = 0
            self._running = True

            # 启动后台接收任务
            self._receive_task = asyncio.create_task(self._receive_loop())

            logger.info("WebSocket 连接成功")
            if self._status_callback:
                self._status_callback(True)
            return True

        except asyncio.TimeoutError:
            logger.warning("连接超时: %s", self._uri)
        except ConnectionRefusedError:
            logger.warning("连接被拒绝，请检查 ESP32-S3 是否已启动 WebSocket 服务")
        except OSError as e:
            logger.warning("网络错误: %s", e)
        except Exception as e:
            logger.error("连接异常: %s", e)

        return False

    async def disconnect(self):
        """断开 WebSocket 连接。"""
        logger.info("正在断开连接...")
        self._running = False

        # 取消接收任务
        if self._receive_task and not self._receive_task.done():
            self._receive_task.cancel()
            try:
                await self._receive_task
            except asyncio.CancelledError:
                pass

        # 关闭 WebSocket 连接
        if self._ws:
            try:
                await self._ws.close()
            except Exception:
                pass
            self._ws = None

        self._connected = False
        logger.info("连接已断开")
        if self._status_callback:
            self._status_callback(False)

    async def _receive_loop(self):
        """
        后台消息接收循环。
        持续监听来自 ESP32-S3 的消息并触发回调。
        """
        while self._running and self._connected:
            try:
                message = await self._ws.recv()
                if message:
                    try:
                        data = json.loads(message)
                        logger.debug("收到消息: %s", data)

                        # 触发消息回调
                        if self._message_callback:
                            self._message_callback(data)

                    except json.JSONDecodeError:
                        logger.warning("收到无效的 JSON 数据: %s", message)

            except ConnectionClosedOK:
                logger.info("服务端关闭了连接")
                break
            except ConnectionClosedError as e:
                logger.warning("连接异常关闭: %s", e)
                break
            except asyncio.CancelledError:
                break
            except Exception as e:
                logger.error("接收消息异常: %s", e)
                await asyncio.sleep(0.1)

        # 如果还在运行状态但连接已断开，尝试重连
        if self._running and not self._connected:
            self._connected = False
            if self._status_callback:
                self._status_callback(False)
            await self._auto_reconnect()

    async def _auto_reconnect(self):
        """自动重连逻辑。"""
        while self._running:
            # 检查是否超过最大重连次数
            if self.max_retries > 0 and self._retry_count >= self.max_retries:
                logger.error(
                    "已达到最大重连次数 (%d)，停止重连",
                    self.max_retries,
                )
                break

            self._retry_count += 1
            logger.info(
                "尝试重连 (%d/%s)...",
                self._retry_count,
                "inf" if self.max_retries == 0 else str(self.max_retries),
            )

            success = await self.connect()
            if success:
                logger.info("重连成功")
                self._reconnect_task = None
                return

            # 等待重连间隔
            await asyncio.sleep(self.reconnect_interval)

    async def _send(self, data: dict) -> bool:
        """
        发送 JSON 数据到 ESP32-S3。

        Args:
            data: 要发送的字典数据

        Returns:
            bool: 发送是否成功
        """
        if not self._connected or self._ws is None:
            logger.warning("未连接，无法发送数据")
            return False

        async with self._send_lock:
            try:
                message = json.dumps(data, ensure_ascii=False)
                await self._ws.send(message)
                logger.debug("发送数据: %s", message)
                return True

            except ConnectionClosedError:
                logger.warning("发送时连接已断开")
                self._connected = False
                if self._status_callback:
                    self._status_callback(False)
                # 触发重连（检查是否已有重连任务在运行，避免重复创建）
                if self._reconnect_task is None or self._reconnect_task.done():
                    self._reconnect_task = asyncio.create_task(self._auto_reconnect())
                else:
                    logger.debug("重连任务已在运行中，跳过重复创建")
            except Exception as e:
                logger.error("发送数据异常: %s", e)

        return False

    # ============================================================
    # 坐标与绘图指令
    # ============================================================

    async def send_target(self, x: float, y: float, speed: float = 0.0) -> bool:
        """
        发送目标坐标到振镜。

        Args:
            x: X 坐标（振镜坐标范围，通常 0-4095）
            y: Y 坐标（振镜坐标范围，通常 0-4095）
            speed: 移动速度（0 = 最大速度）

        Returns:
            bool: 发送是否成功
        """
        return await self._send({
            "cmd": self.CMD_TARGET,
            "x": round(x, 2),
            "y": round(y, 2),
            "speed": round(speed, 2),
        })

    async def send_target_pixel(self, px: float, py: float) -> bool:
        """
        发送像素坐标（会自动转换为振镜坐标）。

        注意：此方法需要配合标定模块使用，
        调用前需确保标定数据已加载。

        Args:
            px: 像素 X 坐标
            py: 像素 Y 坐标

        Returns:
            bool: 发送是否成功
        """
        # 这里发送原始像素坐标，转换由调用方完成
        return await self._send({
            "cmd": self.CMD_TARGET,
            "px": round(px, 2),
            "py": round(py, 2),
        })

    async def draw_point(self, x: float, y: float, intensity: float = 1.0) -> bool:
        """
        在指定坐标绘制一个点。

        Args:
            x: X 坐标
            y: Y 坐标
            intensity: 激光强度（0.0 - 1.0）

        Returns:
            bool: 发送是否成功
        """
        return await self._send({
            "cmd": self.CMD_DRAW_POINT,
            "x": round(x, 2),
            "y": round(y, 2),
            "intensity": round(intensity, 2),
        })

    async def draw_line(
        self,
        x1: float, y1: float,
        x2: float, y2: float,
        speed: float = 50.0,
    ) -> bool:
        """
        绘制一条线段。

        Args:
            x1, y1: 起点坐标
            x2, y2: 终点坐标
            speed: 绘制速度

        Returns:
            bool: 发送是否成功
        """
        return await self._send({
            "cmd": self.CMD_DRAW_LINE,
            "x1": round(x1, 2),
            "y1": round(y1, 2),
            "x2": round(x2, 2),
            "y2": round(y2, 2),
            "speed": round(speed, 2),
        })

    async def draw_circle(
        self,
        cx: float, cy: float,
        radius: float,
        speed: float = 50.0,
    ) -> bool:
        """
        绘制一个圆形。

        Args:
            cx, cy: 圆心坐标
            radius: 半径
            speed: 绘制速度

        Returns:
            bool: 发送是否成功
        """
        return await self._send({
            "cmd": self.CMD_DRAW_CIRCLE,
            "cx": round(cx, 2),
            "cy": round(cy, 2),
            "radius": round(radius, 2),
            "speed": round(speed, 2),
        })

    async def draw_rect(
        self,
        x: float, y: float,
        width: float, height: float,
        speed: float = 50.0,
    ) -> bool:
        """
        绘制一个矩形。

        Args:
            x, y: 左上角坐标
            width: 宽度
            height: 高度
            speed: 绘制速度

        Returns:
            bool: 发送是否成功
        """
        return await self._send({
            "cmd": self.CMD_DRAW_RECT,
            "x": round(x, 2),
            "y": round(y, 2),
            "w": round(width, 2),
            "h": round(height, 2),
            "speed": round(speed, 2),
        })

    async def clear_display(self) -> bool:
        """
        清除振镜投射画面。

        Returns:
            bool: 发送是否成功
        """
        return await self._send({"cmd": self.CMD_CLEAR})

    async def go_home(self) -> bool:
        """
        振镜回到原点位置。

        Returns:
            bool: 发送是否成功
        """
        return await self._send({"cmd": self.CMD_HOME})

    # ============================================================
    # 激光控制指令
    # ============================================================

    async def laser_on(self) -> bool:
        """
        开启激光。

        Returns:
            bool: 发送是否成功
        """
        return await self._send({"cmd": self.CMD_LASER_ON})

    async def laser_off(self) -> bool:
        """
        关闭激光。

        Returns:
            bool: 发送是否成功
        """
        return await self._send({"cmd": self.CMD_LASER_OFF})

    async def set_safety_mode(self, enabled: bool) -> bool:
        """
        切换安全模式。

        Args:
            enabled: True 开启安全模式，False 关闭安全模式

        Returns:
            bool: 发送是否成功
        """
        return await self._send({
            "cmd": self.CMD_SAFETY,
            "enabled": enabled,
        })

    # ============================================================
    # 标定指令
    # ============================================================

    async def send_calibration_point(self, x: float, y: float) -> bool:
        """
        发送标定点投射指令。

        Args:
            x: 振镜 X 坐标
            y: 振镜 Y 坐标

        Returns:
            bool: 发送是否成功
        """
        return await self._send({
            "cmd": self.CMD_CALIBRATE,
            "x": round(x, 2),
            "y": round(y, 2),
        })

    async def request_status(self) -> bool:
        """
        请求设备状态。

        Returns:
            bool: 发送是否成功
        """
        return await self._send({"cmd": self.CMD_STATUS})

    # ============================================================
    # 辅助方法
    # ============================================================

    async def send_raw(self, message: str) -> bool:
        """
        发送原始字符串消息。

        Args:
            message: 原始消息字符串

        Returns:
            bool: 发送是否成功
        """
        if not self._connected or self._ws is None:
            logger.warning("未连接，无法发送数据")
            return False

        try:
            await self._ws.send(message)
            logger.debug("发送原始数据: %s", message)
            return True
        except Exception as e:
            logger.error("发送原始数据异常: %s", e)
            return False


async def test_connection(host: str = None, port: int = None) -> bool:
    """
    测试与 ESP32-S3 的 WebSocket 连接。

    Args:
        host: ESP32-S3 IP 地址
        port: WebSocket 端口

    Returns:
        bool: 连接是否成功
    """
    host = host or config.ESP32_IP
    port = port or config.WEBSOCKET_PORT
    uri = f"ws://{host}:{port}"

    logger.info("测试连接: %s", uri)
    try:
        ws = await asyncio.wait_for(
            websockets.connect(uri, close_timeout=3),
            timeout=config.WEBSOCKET_TIMEOUT,
        )
        await ws.close()
        logger.info("连接测试成功")
        return True
    except Exception as e:
        logger.error("连接测试失败: %s", e)
        return False
