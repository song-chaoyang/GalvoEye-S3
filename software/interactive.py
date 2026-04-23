# -*- coding: utf-8 -*-
"""
interactive.py - 交互式激光按钮模块
======================================
实现基于激光投射的虚拟按钮系统。
通过摄像头检测手指位置和运动，判断用户是否"点击"了
激光投射的虚拟按钮。

功能：
- 激光按钮形状投射（圆形/矩形）
- 手指检测（肤色检测 + 轮廓识别）
- 运动检测判断点击
- 按钮事件回调
- 多按钮管理
"""

import asyncio
import logging
import time
from typing import Optional, List, Tuple, Callable, Dict, Any

import cv2
import numpy as np

import config
from communicator import LaserCommunicator
from calibrator import Calibrator

logger = logging.getLogger(__name__)


class LaserButton:
    """
    激光虚拟按钮。

    定义一个虚拟按钮的属性，包括位置、大小、形状和状态。
    """

    # 按钮状态
    STATE_NORMAL = "normal"
    STATE_HOVER = "hover"
    STATE_PRESSED = "pressed"

    def __init__(
        self,
        button_id: str,
        center_x: float,
        center_y: float,
        size: float = None,
        shape: str = None,
        label: str = "",
        callback: Optional[Callable[[str, str], None]] = None,
    ):
        """
        初始化激光按钮。

        Args:
            button_id: 按钮唯一标识
            center_x: 按钮中心 X 坐标（像素）
            center_y: 按钮中心 Y 坐标（像素）
            size: 按钮大小（像素）
            shape: 按钮形状（"circle" 或 "rectangle"）
            label: 按钮标签
            callback: 按钮回调函数 callback(button_id, state)
        """
        self.button_id = button_id
        self.center_x = center_x
        self.center_y = center_y
        self.size = size or config.BUTTON_SIZE
        self.shape = shape or config.BUTTON_SHAPE
        self.label = label
        self.callback = callback

        # 按钮状态
        self.state = self.STATE_NORMAL
        self.is_pressed = False
        self.press_time = 0.0
        self.last_hover_time = 0.0

        # 振镜坐标（标定后填充）
        self.galvo_x: Optional[float] = None
        self.galvo_y: Optional[float] = None

    def contains_point(self, x: float, y: float) -> bool:
        """
        判断点是否在按钮范围内。

        Args:
            x: 点的 X 坐标
            y: 点的 Y 坐标

        Returns:
            bool: 点是否在按钮内
        """
        if self.shape == "circle":
            dx = x - self.center_x
            dy = y - self.center_y
            return (dx * dx + dy * dy) <= (self.size / 2) ** 2
        else:  # rectangle
            half = self.size / 2
            return (
                self.center_x - half <= x <= self.center_x + half
                and self.center_y - half <= y <= self.center_y + half
            )

    def update_galvo_coords(self, calibrator: Calibrator):
        """
        使用标定数据更新振镜坐标。

        Args:
            calibrator: 标定器实例
        """
        result = calibrator.pixel_to_galvo(self.center_x, self.center_y)
        if result:
            self.galvo_x, self.galvo_y = result

    def __repr__(self):
        return (
            f"LaserButton(id={self.button_id}, "
            f"pos=({self.center_x:.0f},{self.center_y:.0f}), "
            f"state={self.state})"
        )


class FingerDetector:
    """
    手指检测器。

    使用肤色检测和轮廓分析来定位手指位置。
    """

    def __init__(
        self,
        hsv_lower: List[int] = None,
        hsv_upper: List[int] = None,
        min_contour_area: int = None,
    ):
        """
        初始化手指检测器。

        Args:
            hsv_lower: 肤色 HSV 下界
            hsv_upper: 肤色 HSV 上界
            min_contour_area: 最小轮廓面积
        """
        self.hsv_lower = np.array(
            hsv_lower or config.SKIN_HSV_LOWER, dtype=np.uint8,
        )
        self.hsv_upper = np.array(
            hsv_upper or config.SKIN_HSV_UPPER, dtype=np.uint8,
        )
        self.min_contour_area = min_contour_area or config.FINGER_MIN_CONTOUR_AREA

        # 上一帧用于运动检测
        self._prev_frame: Optional[np.ndarray] = None

        # 运动检测参数
        self._motion_threshold = config.MOTION_THRESHOLD
        self._motion_min_pixels = config.MOTION_MIN_PIXELS

        # 手指位置历史（用于平滑）
        self._position_history: List[Tuple[float, float]] = []
        self._max_history = 5

    def detect_skin(self, frame: np.ndarray) -> np.ndarray:
        """
        检测图像中的肤色区域。

        Args:
            frame: BGR 格式图像

        Returns:
            肤色掩码（二值图像）
        """
        # 转换到 HSV 颜色空间
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # 肤色范围检测
        mask = cv2.inRange(hsv, self.hsv_lower, self.hsv_upper)

        # 形态学操作去噪
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        # 高斯模糊平滑
        mask = cv2.GaussianBlur(mask, (5, 5), 0)

        return mask

    def detect_finger(
        self,
        frame: np.ndarray,
    ) -> Optional[Tuple[float, float]]:
        """
        检测手指位置。

        通过肤色检测找到手部区域，然后分析轮廓找到指尖。

        Args:
            frame: BGR 格式图像

        Returns:
            手指位置 (x, y)，未检测到返回 None
        """
        # 获取肤色掩码
        skin_mask = self.detect_skin(frame)

        # 查找轮廓
        contours, _ = cv2.findContours(
            skin_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE,
        )

        if not contours:
            return None

        # 找到面积最大的轮廓（手部）
        hand_contour = max(contours, key=cv2.contourArea)

        if cv2.contourArea(hand_contour) < self.min_contour_area:
            return None

        # 获取凸包
        hull = cv2.convexHull(hand_contour)

        # 找到凸包中的极值点（指尖候选）
        # 使用凸缺陷来识别指尖
        try:
            hull_indices = cv2.convexHull(hand_contour, returnPoints=False)
            defects = cv2.convexityDefects(hand_contour, hull_indices)
        except cv2.error:
            defects = None

        if defects is not None and len(defects) > 0:
            # 从凸包顶点中找到最高的点（y 最小的点 = 最上方的指尖）
            finger_candidates = []

            for i in range(len(hull) - 1):
                pt = hull[i][0]
                finger_candidates.append((pt[0], pt[1]))

            if finger_candidates:
                # 选择 y 坐标最小的点（画面中最上方的指尖）
                finger_tip = min(finger_candidates, key=lambda p: p[1])

                # 平滑处理
                self._position_history.append(finger_tip)
                if len(self._position_history) > self._max_history:
                    self._position_history.pop(0)

                # 返回平滑后的位置
                avg_x = np.mean([p[0] for p in self._position_history])
                avg_y = np.mean([p[1] for p in self._position_history])

                return (round(avg_x, 1), round(avg_y, 1))

        # 降级方案：使用轮廓的最高点
        top_point = tuple(hand_contour[hand_contour[:, :, 1].argmin()][0])
        self._position_history.append(top_point)
        if len(self._position_history) > self._max_history:
            self._position_history.pop(0)

        avg_x = np.mean([p[0] for p in self._position_history])
        avg_y = np.mean([p[1] for p in self._position_history])

        return (round(avg_x, 1), round(avg_y, 1))

    def detect_motion(self, frame: np.ndarray) -> Tuple[bool, float]:
        """
        检测画面中的运动。

        通过帧差法检测运动区域的大小。

        Args:
            frame: BGR 格式图像

        Returns:
            (是否有运动, 运动强度 0.0-1.0)
        """
        if self._prev_frame is None:
            self._prev_frame = frame.copy()
            return (False, 0.0)

        # 转灰度
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        prev_gray = cv2.cvtColor(self._prev_frame, cv2.COLOR_BGR2GRAY)

        # 高斯模糊
        gray = cv2.GaussianBlur(gray, (21, 21), 0)
        prev_gray = cv2.GaussianBlur(prev_gray, (21, 21), 0)

        # 帧差
        diff = cv2.absdiff(prev_gray, gray)
        _, threshold = cv2.threshold(
            diff, self._motion_threshold, 255, cv2.THRESH_BINARY,
        )

        # 计算运动像素数
        motion_pixels = cv2.countNonZero(threshold)
        total_pixels = threshold.shape[0] * threshold.shape[1]
        motion_ratio = motion_pixels / total_pixels

        # 更新上一帧
        self._prev_frame = frame.copy()

        has_motion = motion_pixels > self._motion_min_pixels

        return (has_motion, motion_ratio)

    def detect_click(
        self,
        frame: np.ndarray,
        finger_pos: Tuple[float, float],
    ) -> bool:
        """
        判断手指是否执行了点击动作。

        通过检测手指的快速移动（向下运动）来判断点击。

        Args:
            frame: BGR 格式图像
            finger_pos: 当前手指位置

        Returns:
            bool: 是否检测到点击
        """
        if not self._position_history or len(self._position_history) < 3:
            return False

        # 计算最近几帧手指的 y 方向速度
        history = self._position_history[-3:]
        dy = history[-1][1] - history[0][1]

        # 检测快速向下运动（点击动作）
        if dy > 15:  # 向下移动超过 15 像素
            # 同时检测运动强度
            has_motion, motion_ratio = self.detect_motion(frame)
            if has_motion and motion_ratio > 0.005:
                return True

        return False

    def reset(self):
        """重置检测器状态。"""
        self._prev_frame = None
        self._position_history.clear()


class InteractiveManager:
    """
    交互式激光按钮管理器。

    管理多个激光按钮，检测手指交互，触发按钮事件。
    """

    def __init__(
        self,
        communicator: LaserCommunicator,
        calibrator: Optional[Calibrator] = None,
    ):
        """
        初始化交互管理器。

        Args:
            communicator: WebSocket 通信器
            calibrator: 标定器实例
        """
        self.comm = communicator
        self.calibrator = calibrator

        # 按钮列表
        self.buttons: List[LaserButton] = []

        # 手指检测器
        self.finger_detector = FingerDetector()

        # 当前手指位置
        self.current_finger_pos: Optional[Tuple[float, float]] = None

        # 上次按钮投射时间
        self._last_draw_time = 0.0
        self._draw_interval = 0.1  # 按钮投射刷新间隔（秒）

        # 点击冷却时间
        self._click_cooldown = 0.0
        self._click_cooldown_time = 0.5  # 点击冷却时间（秒）

        logger.info("交互管理器初始化完成")

    def add_button(
        self,
        button_id: str,
        center_x: float,
        center_y: float,
        size: float = None,
        shape: str = None,
        label: str = "",
        callback: Optional[Callable[[str, str], None]] = None,
    ) -> LaserButton:
        """
        添加一个激光按钮。

        Args:
            button_id: 按钮唯一标识
            center_x: 中心 X 坐标（像素）
            center_y: 中心 Y 坐标（像素）
            size: 按钮大小
            shape: 按钮形状
            label: 按钮标签
            callback: 回调函数

        Returns:
            创建的按钮对象
        """
        button = LaserButton(
            button_id=button_id,
            center_x=center_x,
            center_y=center_y,
            size=size,
            shape=shape,
            label=label,
            callback=callback,
        )

        # 如果有标定数据，更新振镜坐标
        if self.calibrator and self.calibrator.inverse_matrix is not None:
            button.update_galvo_coords(self.calibrator)

        self.buttons.append(button)
        logger.info(
            "添加按钮: id=%s, pos=(%.0f, %.0f), shape=%s",
            button_id, center_x, center_y, button.shape,
        )

        return button

    def remove_button(self, button_id: str) -> bool:
        """
        移除指定按钮。

        Args:
            button_id: 按钮标识

        Returns:
            bool: 是否成功移除
        """
        for i, btn in enumerate(self.buttons):
            if btn.button_id == button_id:
                self.buttons.pop(i)
                logger.info("移除按钮: id=%s", button_id)
                return True
        return False

    def create_default_buttons(self) -> List[LaserButton]:
        """
        创建默认的按钮布局。

        在画面中均匀分布按钮。

        Returns:
            创建的按钮列表
        """
        self.buttons.clear()

        count = config.BUTTON_COUNT
        spacing = config.BUTTON_SPACING
        size = config.BUTTON_SIZE
        shape = config.BUTTON_SHAPE

        # 计算起始位置（居中排列）
        total_width = (count - 1) * spacing
        start_x = (config.CAMERA_WIDTH - total_width) / 2
        center_y = config.CAMERA_HEIGHT / 2

        for i in range(count):
            cx = start_x + i * spacing
            btn = self.add_button(
                button_id=f"btn_{i + 1}",
                center_x=cx,
                center_y=center_y,
                size=size,
                shape=shape,
                label=f"Button {i + 1}",
            )

        logger.info("创建了 %d 个默认按钮", count)
        return self.buttons

    async def draw_buttons(self):
        """
        通过振镜投射所有按钮的轮廓。

        向 ESP32-S3 发送绘图指令来投射按钮形状。
        """
        current_time = time.time()
        if current_time - self._last_draw_time < self._draw_interval:
            return

        self._last_draw_time = current_time

        for btn in self.buttons:
            if btn.galvo_x is None or btn.galvo_y is None:
                continue

            # 根据按钮状态选择不同的投射方式和颜色
            if btn.state == LaserButton.STATE_PRESSED:
                # 按下状态：填充按钮（红色）
                if btn.shape == "circle":
                    await self.comm.draw_circle(
                        btn.galvo_x, btn.galvo_y,
                        btn.size / 2,
                        r=0, g=0, b=255,
                    )
                else:
                    await self.comm.draw_rect(
                        btn.galvo_x - btn.size / 2,
                        btn.galvo_y - btn.size / 2,
                        btn.size, btn.size,
                        r=0, g=0, b=255,
                    )
            else:
                # 正常/悬停状态：绘制轮廓
                if btn.shape == "circle":
                    # 使用 draw_circle 绘制轮廓（1次 WebSocket 消息代替36次）
                    if btn.state == LaserButton.STATE_HOVER:
                        # 悬停状态：黄色
                        color = (0, 255, 255)
                    else:
                        # 正常状态：绿色
                        color = (0, 255, 0)
                    await self.comm.draw_circle(
                        btn.galvo_x, btn.galvo_y,
                        btn.size / 2,
                        r=color[0], g=color[1], b=color[2],
                    )
                else:
                    # 绘制矩形轮廓
                    if btn.state == LaserButton.STATE_HOVER:
                        # 悬停状态：黄色
                        color = (0, 255, 255)
                    else:
                        # 正常状态：绿色
                        color = (0, 255, 0)
                    half = btn.size / 2
                    x1 = btn.galvo_x - half
                    y1 = btn.galvo_y - half
                    x2 = btn.galvo_x + half
                    y2 = btn.galvo_y + half
                    await self.comm.draw_line(x1, y1, x2, y1, r=color[0], g=color[1], b=color[2])
                    await self.comm.draw_line(x2, y1, x2, y2, r=color[0], g=color[1], b=color[2])
                    await self.comm.draw_line(x2, y2, x1, y2, r=color[0], g=color[1], b=color[2])
                    await self.comm.draw_line(x1, y2, x1, y1, r=color[0], g=color[1], b=color[2])

    def process_frame(self, frame: np.ndarray) -> np.ndarray:
        """
        处理一帧图像，检测手指交互。

        Args:
            frame: BGR 格式图像

        Returns:
            可视化后的图像
        """
        vis = frame.copy()
        current_time = time.time()

        # 检测手指位置
        finger_pos = self.finger_detector.detect_finger(frame)
        self.current_finger_pos = finger_pos

        # 检测运动
        has_motion, motion_ratio = self.finger_detector.detect_motion(frame)

        # 检测点击
        is_click = False
        if finger_pos:
            is_click = self.finger_detector.detect_click(frame, finger_pos)

        # 检查点击冷却
        if current_time - self._click_cooldown < self._click_cooldown_time:
            is_click = False

        # 更新按钮状态
        for btn in self.buttons:
            prev_state = btn.state

            if finger_pos and btn.contains_point(finger_pos[0], finger_pos[1]):
                if is_click:
                    # 点击事件
                    btn.state = LaserButton.STATE_PRESSED
                    btn.is_pressed = True
                    btn.press_time = current_time

                    # 触发回调
                    if btn.callback:
                        try:
                            btn.callback(btn.button_id, LaserButton.STATE_PRESSED)
                        except Exception as e:
                            logger.error("按钮回调异常: %s", e)

                    # 设置冷却
                    self._click_cooldown = current_time

                    logger.info("按钮点击: %s", btn.button_id)
                else:
                    # 悬停状态
                    btn.state = LaserButton.STATE_HOVER
                    btn.last_hover_time = current_time
            else:
                # 正常状态
                if btn.is_pressed and (current_time - btn.press_time) > 0.3:
                    btn.is_pressed = False
                if not btn.is_pressed:
                    btn.state = LaserButton.STATE_NORMAL

        # 可视化
        vis = self._visualize(vis, finger_pos, has_motion, motion_ratio)

        return vis

    def _visualize(
        self,
        frame: np.ndarray,
        finger_pos: Optional[Tuple[float, float]],
        has_motion: bool,
        motion_ratio: float,
    ) -> np.ndarray:
        """
        可视化交互状态。

        Args:
            frame: 输入图像
            finger_pos: 手指位置
            has_motion: 是否有运动
            motion_ratio: 运动强度

        Returns:
            可视化后的图像
        """
        vis = frame.copy()

        # 绘制按钮
        for btn in self.buttons:
            cx, cy = int(btn.center_x), int(btn.center_y)
            half = int(btn.size / 2)

            if btn.state == LaserButton.STATE_NORMAL:
                color = (0, 255, 0)    # 绿色
                thickness = 2
            elif btn.state == LaserButton.STATE_HOVER:
                color = (0, 255, 255)  # 黄色
                thickness = 3
            else:  # PRESSED
                color = (0, 0, 255)    # 红色
                thickness = -1

            if btn.shape == "circle":
                cv2.circle(vis, (cx, cy), half, color, thickness)
            else:
                cv2.rectangle(
                    vis,
                    (cx - half, cy - half),
                    (cx + half, cy + half),
                    color, thickness,
                )

            # 绘制按钮标签
            if btn.label:
                cv2.putText(
                    vis, btn.label,
                    (cx - 30, cy + half + 25),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                    (255, 255, 255), 1,
                )

        # 绘制手指位置
        if finger_pos:
            fx, fy = int(finger_pos[0]), int(finger_pos[1])
            cv2.circle(vis, (fx, fy), 10, (255, 0, 0), 2)
            cv2.circle(vis, (fx, fy), 3, (255, 0, 0), -1)
            cv2.putText(
                vis, "Finger",
                (fx + 15, fy - 10),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                (255, 0, 0), 1,
            )

        # 显示运动状态
        motion_text = f"Motion: {motion_ratio:.4f}"
        motion_color = (0, 255, 0) if not has_motion else (0, 0, 255)
        cv2.putText(
            vis, motion_text, (10, 30),
            cv2.FONT_HERSHEY_SIMPLEX, 0.6,
            motion_color, 1,
        )

        # 显示按钮数量
        cv2.putText(
            vis, f"Buttons: {len(self.buttons)}", (10, 55),
            cv2.FONT_HERSHEY_SIMPLEX, 0.6,
            (255, 255, 255), 1,
        )

        return vis

    def get_pressed_buttons(self) -> List[LaserButton]:
        """
        获取当前被按下的按钮列表。

        Returns:
            被按下的按钮列表
        """
        return [btn for btn in self.buttons if btn.is_pressed]

    def get_hovered_button(self) -> Optional[LaserButton]:
        """
        获取当前悬停的按钮。

        Returns:
            悬停的按钮，无悬停返回 None
        """
        for btn in self.buttons:
            if btn.state == LaserButton.STATE_HOVER:
                return btn
        return None

    def reset(self):
        """重置所有按钮状态和检测器。"""
        for btn in self.buttons:
            btn.state = LaserButton.STATE_NORMAL
            btn.is_pressed = False
        self.finger_detector.reset()
        self.current_finger_pos = None
        logger.info("交互管理器已重置")
