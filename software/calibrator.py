# -*- coding: utf-8 -*-
"""
calibrator.py - 坐标标定模块
================================
控制振镜投射标定点，通过摄像头捕捉标定点在画面中的位置，
计算摄像头像素坐标与振镜坐标之间的透视变换矩阵。

标定流程：
1. 在振镜坐标空间中生成网格标定点
2. 逐个投射标定点并等待稳定
3. 摄像头捕捉画面，检测激光亮点位置
4. 收集所有标定点的对应关系
5. 计算透视变换矩阵
6. 评估标定精度
7. 保存标定数据
"""

import asyncio
import json
import logging
import os
import time
from typing import Optional, Tuple, List

import cv2
import numpy as np

import config
from communicator import LaserCommunicator

logger = logging.getLogger(__name__)


class Calibrator:
    """
    激光振镜坐标标定器。

    通过投射标定点并检测其在摄像头画面中的位置，
    建立振镜坐标与像素坐标之间的映射关系。
    """

    # 振镜坐标范围（DAC 值）
    GALVO_X_MIN = 0
    GALVO_X_MAX = 4095
    GALVO_Y_MIN = 0
    GALVO_Y_MAX = 4095

    def __init__(
        self,
        communicator: LaserCommunicator,
        camera_index: int = None,
        grid_size: int = None,
    ):
        """
        初始化标定器。

        Args:
            communicator: WebSocket 通信器实例
            camera_index: 摄像头设备索引
            grid_size: 标定网格大小（3 = 3x3 = 9个点）
        """
        self.comm = communicator
        self.camera_index = camera_index or config.CAMERA_INDEX
        self.grid_size = grid_size or config.CALIBRATION_GRID_SIZE

        # 摄像头对象
        self._cap: Optional[cv2.VideoCapture] = None

        # 标定数据
        self.galvo_points: List[Tuple[float, float]] = []  # 振镜坐标
        self.pixel_points: List[Tuple[float, float]] = []  # 像素坐标
        self.transform_matrix: Optional[np.ndarray] = None  # 透视变换矩阵
        self.inverse_matrix: Optional[np.ndarray] = None    # 逆变换矩阵

        # 标定精度评估结果
        self.calibration_errors: List[float] = []
        self.mean_error: float = 0.0
        self.max_error: float = 0.0

        # 标定数据文件路径
        self._calib_file = config.get_calibration_file_path()

        logger.info(
            "标定器初始化完成 (网格: %dx%d, 摄像头: %d)",
            self.grid_size, self.grid_size, self.camera_index,
        )

    def _open_camera(self) -> bool:
        """
        打开摄像头。

        Returns:
            bool: 摄像头是否成功打开
        """
        self._cap = cv2.VideoCapture(self.camera_index)

        if not self._cap.isOpened():
            logger.error("无法打开摄像头 (索引: %d)", self.camera_index)
            return False

        # 设置摄像头分辨率
        self._cap.set(cv2.CAP_PROP_FRAME_WIDTH, config.CAMERA_WIDTH)
        self._cap.set(cv2.CAP_PROP_FRAME_HEIGHT, config.CAMERA_HEIGHT)
        self._cap.set(cv2.CAP_PROP_FPS, config.CAMERA_FPS)

        actual_w = int(self._cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        actual_h = int(self._cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        logger.info("摄像头已打开，分辨率: %dx%d", actual_w, actual_h)

        return True

    def _close_camera(self):
        """关闭摄像头。"""
        if self._cap and self._cap.isOpened():
            self._cap.release()
            self._cap = None
            logger.info("摄像头已关闭")

    def _generate_galvo_points(self) -> List[Tuple[float, float]]:
        """
        生成振镜坐标空间的标定点网格。

        Returns:
            标定点列表 [(x, y), ...]
        """
        # 防止 grid_size 过小导致除零错误
        if self.grid_size < 2:
            logger.error("grid_size 必须 >= 2，当前值: %d", self.grid_size)
            return []

        points = []
        n = self.grid_size

        # 计算标定点间距
        x_range = self.GALVO_X_MAX - self.GALVO_X_MIN
        y_range = self.GALVO_Y_MAX - self.GALVO_Y_MIN

        # 在坐标范围内均匀分布标定点
        for row in range(n):
            for col in range(n):
                x = self.GALVO_X_MIN + x_range * col / (n - 1)
                y = self.GALVO_Y_MIN + y_range * row / (n - 1)
                points.append((round(x, 1), round(y, 1)))

        logger.info("生成了 %d 个振镜标定点", len(points))
        return points

    def _capture_frame(self) -> Optional[np.ndarray]:
        """
        从摄像头捕获一帧图像。

        Returns:
            捕获的图像（BGR 格式），失败返回 None
        """
        if not self._cap or not self._cap.isOpened():
            logger.error("摄像头未打开")
            return None

        # 丢弃前几帧以获得稳定的图像
        for _ in range(5):
            self._cap.read()

        ret, frame = self._cap.read()
        if not ret or frame is None:
            logger.error("捕获帧失败")
            return None

        return frame

    def _detect_laser_spot(
        self,
        frame: np.ndarray,
        roi: Optional[Tuple[int, int, int, int]] = None,
    ) -> Optional[Tuple[float, float]]:
        """
        在图像中检测激光亮点的位置。

        使用亮度阈值 + 形态学操作 + 质心计算来定位激光点。

        Args:
            frame: BGR 格式图像
            roi: 感兴趣区域 (x, y, w, h)，None 表示全图

        Returns:
            激光亮点的像素坐标 (cx, cy)，未检测到返回 None
        """
        # 转为灰度图
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # 应用 ROI
        if roi is not None:
            x, y, w, h = roi
            gray = gray[y:y + h, x:x + w]
            offset_x, offset_y = x, y
        else:
            offset_x, offset_y = 0, 0

        # 二值化 - 寻找最亮的区域
        _, binary = cv2.threshold(
            gray,
            config.CALIBRATION_BINARY_THRESHOLD,
            255,
            cv2.THRESH_BINARY,
        )

        # 形态学操作去除噪声
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        binary = cv2.morphologyEx(binary, cv2.MORPH_OPEN, kernel)
        binary = cv2.morphologyEx(binary, cv2.MORPH_CLOSE, kernel)

        # 查找轮廓
        contours, _ = cv2.findContours(
            binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE,
        )

        if not contours:
            logger.warning("未检测到激光亮点")
            return None

        # 找到面积最大的轮廓（激光亮点）
        max_contour = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(max_contour)

        # 面积过小则忽略
        if area < 10:
            logger.warning("检测到的亮点面积过小: %.1f", area)
            return None

        # 计算质心
        moments = cv2.moments(max_contour)
        if moments["m00"] == 0:
            logger.warning("无法计算质心")
            return None

        cx = moments["m10"] / moments["m00"] + offset_x
        cy = moments["m01"] / moments["m00"] + offset_y

        return (round(cx, 2), round(cy, 2))

    def _detect_laser_spot_advanced(
        self,
        frame: np.ndarray,
    ) -> Optional[Tuple[float, float]]:
        """
        高级激光亮点检测（使用颜色过滤 + 亮度分析）。

        适用于红色激光的检测。

        Args:
            frame: BGR 格式图像

        Returns:
            激光亮点的像素坐标 (cx, cy)，未检测到返回 None
        """
        # 转换到 HSV 颜色空间
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # 红色激光的 HSV 范围（红色在 HSV 中分布在两端）
        # 范围 1: H 0-10
        lower_red1 = np.array([0, 100, 100])
        upper_red1 = np.array([10, 255, 255])
        mask1 = cv2.inRange(hsv, lower_red1, upper_red1)

        # 范围 2: H 160-180
        lower_red2 = np.array([160, 100, 100])
        upper_red2 = np.array([180, 255, 255])
        mask2 = cv2.inRange(hsv, lower_red2, upper_red2)

        # 合并两个范围
        mask = cv2.bitwise_or(mask1, mask2)

        # 同时使用亮度过滤
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        _, bright_mask = cv2.threshold(gray, 200, 255, cv2.THRESH_BINARY)
        mask = cv2.bitwise_and(mask, bright_mask)

        # 形态学操作
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        # 查找轮廓
        contours, _ = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE,
        )

        if not contours:
            # 降级到亮度检测
            return self._detect_laser_spot(frame)

        # 找到最亮最大的轮廓
        max_contour = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(max_contour)

        if area < 10:
            return self._detect_laser_spot(frame)

        moments = cv2.moments(max_contour)
        if moments["m00"] == 0:
            return self._detect_laser_spot(frame)

        cx = moments["m10"] / moments["m00"]
        cy = moments["m01"] / moments["m00"]

        return (round(cx, 2), round(cy, 2))

    async def _project_and_detect(
        self,
        galvo_x: float,
        galvo_y: float,
        point_index: int,
        total_points: int,
    ) -> Optional[Tuple[float, float]]:
        """
        投射一个标定点并检测其在摄像头画面中的位置。

        Args:
            galvo_x: 振镜 X 坐标
            galvo_y: 振镜 Y 坐标
            point_index: 当前点索引（用于显示进度）
            total_points: 总点数

        Returns:
            检测到的像素坐标 (px, py)，失败返回 None
        """
        logger.info(
            "投射标定点 %d/%d: 振镜坐标 (%.1f, %.1f)",
            point_index + 1, total_points, galvo_x, galvo_y,
        )

        # 发送标定点投射指令
        success = await self.comm.send_calibration_point(galvo_x, galvo_y)
        if not success:
            logger.error("发送标定点指令失败")
            return None

        # 等待振镜稳定和激光投射
        await asyncio.sleep(config.CALIBRATION_POINT_DURATION)

        # 捕获画面
        frame = self._capture_frame()
        if frame is None:
            logger.error("捕获画面失败")
            return None

        # 检测激光亮点
        pixel_pos = self._detect_laser_spot_advanced(frame)

        if pixel_pos is None:
            logger.warning(
                "标定点 %d/%d 检测失败",
                point_index + 1, total_points,
            )
            return None

        logger.info(
            "标定点 %d/%d 检测成功: 像素坐标 (%.1f, %.1f)",
            point_index + 1, total_points, pixel_pos[0], pixel_pos[1],
        )

        return pixel_pos

    def _compute_transform(self) -> bool:
        """
        计算透视变换矩阵。

        使用 cv2.getPerspectiveTransform 计算从振镜坐标到像素坐标的映射。
        需要 4 个或以上的对应点。

        Returns:
            bool: 计算是否成功
        """
        n = len(self.galvo_points)
        if n < 4:
            logger.error("标定点数量不足（需要至少 4 个，当前 %d 个）", n)
            return False

        # 转换为 numpy 数组
        src_pts = np.array(self.galvo_points, dtype=np.float32)
        dst_pts = np.array(self.pixel_points, dtype=np.float32)

        try:
            # 计算透视变换矩阵
            self.transform_matrix = cv2.getPerspectiveTransform(src_pts, dst_pts)

            # 计算逆变换矩阵（像素坐标 -> 振镜坐标）
            self.inverse_matrix = cv2.getPerspectiveTransform(dst_pts, src_pts)

            logger.info("透视变换矩阵计算成功")
            logger.debug("变换矩阵:\n%s", self.transform_matrix)
            logger.debug("逆变换矩阵:\n%s", self.inverse_matrix)

            return True

        except cv2.error as e:
            logger.error("计算透视变换矩阵失败: %s", e)
            return False

    def _evaluate_calibration(self) -> bool:
        """
        评估标定精度。

        使用变换矩阵对每个标定点进行重投影，
        计算重投影误差。

        Returns:
            bool: 评估是否成功
        """
        if self.transform_matrix is None:
            logger.error("变换矩阵未计算")
            return False

        self.calibration_errors = []

        for i, (gx, gy) in enumerate(self.galvo_points):
            # 振镜坐标 -> 像素坐标（通过变换矩阵）
            src_pt = np.array([[[gx, gy]]], dtype=np.float32)
            transformed = cv2.perspectiveTransform(src_pt, self.transform_matrix)
            pred_px, pred_py = transformed[0][0]

            # 实际检测到的像素坐标
            actual_px, actual_py = self.pixel_points[i]

            # 计算误差（欧氏距离）
            error = np.sqrt((pred_px - actual_px) ** 2 + (pred_py - actual_py) ** 2)
            self.calibration_errors.append(error)

            logger.debug(
                "标定点 %d: 预测(%.1f, %.1f) 实际(%.1f, %.1f) 误差=%.2f px",
                i + 1, pred_px, pred_py, actual_px, actual_py, error,
            )

        self.mean_error = np.mean(self.calibration_errors)
        self.max_error = np.max(self.calibration_errors)

        logger.info("标定精度评估:")
        logger.info("  平均误差: %.2f 像素", self.mean_error)
        logger.info("  最大误差: %.2f 像素", self.max_error)

        # 判断标定质量
        if self.mean_error < 5:
            logger.info("标定质量: 优秀")
        elif self.mean_error < 10:
            logger.info("标定质量: 良好")
        elif self.mean_error < 20:
            logger.warning("标定质量: 一般，建议重新标定")
        else:
            logger.warning("标定质量: 较差，请重新标定")

        return True

    def save_calibration(self, filepath: str = None) -> bool:
        """
        保存标定数据到 JSON 文件。

        Args:
            filepath: 保存路径，None 使用默认路径

        Returns:
            bool: 保存是否成功
        """
        filepath = filepath or self._calib_file

        if self.transform_matrix is None:
            logger.error("没有可保存的标定数据")
            return False

        data = {
            "version": "1.0",
            "timestamp": time.strftime("%Y-%m-%d %H:%M:%S"),
            "grid_size": self.grid_size,
            "camera_resolution": {
                "width": config.CAMERA_WIDTH,
                "height": config.CAMERA_HEIGHT,
            },
            "galvo_points": self.galvo_points,
            "pixel_points": self.pixel_points,
            "transform_matrix": self.transform_matrix.tolist(),
            "inverse_matrix": self.inverse_matrix.tolist(),
            "calibration_errors": self.calibration_errors,
            "mean_error": float(self.mean_error),
            "max_error": float(self.max_error),
        }

        try:
            os.makedirs(os.path.dirname(filepath), exist_ok=True)
            with open(filepath, "w", encoding="utf-8") as f:
                json.dump(data, f, indent=2, ensure_ascii=False)

            logger.info("标定数据已保存到: %s", filepath)
            return True

        except Exception as e:
            logger.error("保存标定数据失败: %s", e)
            return False

    def load_calibration(self, filepath: str = None) -> bool:
        """
        从 JSON 文件加载标定数据。

        Args:
            filepath: 文件路径，None 使用默认路径

        Returns:
            bool: 加载是否成功
        """
        filepath = filepath or self._calib_file

        if not os.path.exists(filepath):
            logger.warning("标定数据文件不存在: %s", filepath)
            return False

        try:
            with open(filepath, "r", encoding="utf-8") as f:
                data = json.load(f)

            self.grid_size = data.get("grid_size", self.grid_size)
            self.galvo_points = [tuple(p) for p in data["galvo_points"]]
            self.pixel_points = [tuple(p) for p in data["pixel_points"]]
            self.transform_matrix = np.array(data["transform_matrix"], dtype=np.float32)
            self.inverse_matrix = np.array(data["inverse_matrix"], dtype=np.float32)
            self.calibration_errors = data.get("calibration_errors", [])
            self.mean_error = data.get("mean_error", 0.0)
            self.max_error = data.get("max_error", 0.0)

            logger.info("标定数据已从 %s 加载", filepath)
            logger.info("  标定时间: %s", data.get("timestamp", "未知"))
            logger.info("  平均误差: %.2f 像素", self.mean_error)
            logger.info("  最大误差: %.2f 像素", self.max_error)

            return True

        except Exception as e:
            logger.error("加载标定数据失败: %s", e)
            return False

    async def run_calibration(self) -> bool:
        """
        执行完整的标定流程。

        流程：
        1. 打开摄像头
        2. 生成振镜标定点
        3. 逐个投射并检测
        4. 计算变换矩阵
        5. 评估精度
        6. 保存数据

        Returns:
            bool: 标定是否成功
        """
        logger.info("=" * 50)
        logger.info("开始坐标标定流程")
        logger.info("=" * 50)

        # 清空之前的标定数据
        self.galvo_points = []
        self.pixel_points = []
        self.transform_matrix = None
        self.inverse_matrix = None
        self.calibration_errors = []

        # 检查通信连接
        if not self.comm.is_connected:
            logger.error("WebSocket 未连接，请先连接 ESP32-S3")
            return False

        # 打开摄像头
        if not self._open_camera():
            return False

        try:
            # 生成振镜标定点
            galvo_points = self._generate_galvo_points()
            total = len(galvo_points)

            # 逐个投射并检测标定点
            for i, (gx, gy) in enumerate(galvo_points):
                pixel_pos = await self._project_and_detect(gx, gy, i, total)

                if pixel_pos is not None:
                    self.galvo_points.append((gx, gy))
                    self.pixel_points.append(pixel_pos)
                else:
                    logger.warning("跳过标定点 %d/%d", i + 1, total)

            # 检查是否收集到足够的标定点
            if len(self.galvo_points) < 4:
                logger.error(
                    "有效标定点不足 %d 个（需要至少 4 个）",
                    len(self.galvo_points),
                )
                return False

            logger.info(
                "成功收集 %d/%d 个标定点",
                len(self.galvo_points), total,
            )

            # 计算透视变换矩阵
            if not self._compute_transform():
                return False

            # 评估标定精度
            if not self._evaluate_calibration():
                return False

            # 保存标定数据
            if not self.save_calibration():
                logger.warning("标定数据保存失败，但标定已完成")

            # 关闭激光
            await self.comm.laser_off()
            await self.comm.clear_display()

            logger.info("=" * 50)
            logger.info("坐标标定完成!")
            logger.info("  平均误差: %.2f 像素", self.mean_error)
            logger.info("  最大误差: %.2f 像素", self.max_error)
            logger.info("=" * 50)

            return True

        except Exception as e:
            logger.error("标定过程中发生异常: %s", e)
            return False

        finally:
            self._close_camera()

    def pixel_to_galvo(self, px: float, py: float) -> Optional[Tuple[float, float]]:
        """
        将像素坐标转换为振镜坐标。

        Args:
            px: 像素 X 坐标
            py: 像素 Y 坐标

        Returns:
            振镜坐标 (gx, gy)，标定未完成返回 None
        """
        if self.inverse_matrix is None:
            logger.warning("标定数据未加载，无法进行坐标转换")
            return None

        src_pt = np.array([[[px, py]]], dtype=np.float32)
        result = cv2.perspectiveTransform(src_pt, self.inverse_matrix)
        gx, gy = result[0][0]

        # 限制在振镜坐标范围内
        gx = np.clip(gx, self.GALVO_X_MIN, self.GALVO_X_MAX)
        gy = np.clip(gy, self.GALVO_Y_MIN, self.GALVO_Y_MAX)

        return (round(float(gx), 1), round(float(gy), 1))

    def galvo_to_pixel(self, gx: float, gy: float) -> Optional[Tuple[float, float]]:
        """
        将振镜坐标转换为像素坐标。

        Args:
            gx: 振镜 X 坐标
            gy: 振镜 Y 坐标

        Returns:
            像素坐标 (px, py)，标定未完成返回 None
        """
        if self.transform_matrix is None:
            logger.warning("标定数据未加载，无法进行坐标转换")
            return None

        src_pt = np.array([[[gx, gy]]], dtype=np.float32)
        result = cv2.perspectiveTransform(src_pt, self.transform_matrix)
        px, py = result[0][0]

        return (round(float(px), 1), round(float(py), 1))

    def visualize_calibration(self, frame: np.ndarray) -> np.ndarray:
        """
        在图像上可视化标定结果。

        绘制标定点位置和编号。

        Args:
            frame: 输入图像

        Returns:
            绘制了标定信息的图像
        """
        vis = frame.copy()

        for i, (px, py) in enumerate(self.pixel_points):
            # 绘制标定点
            cv2.circle(vis, (int(px), int(py)), 8, (0, 255, 0), 2)
            cv2.circle(vis, (int(px), int(py)), 3, (0, 0, 255), -1)

            # 绘制编号
            cv2.putText(
                vis,
                str(i + 1),
                (int(px) + 10, int(py) - 10),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (255, 255, 255),
                1,
            )

        # 显示标定信息
        if self.mean_error > 0:
            info_text = f"Mean Error: {self.mean_error:.1f}px"
            cv2.putText(
                vis, info_text, (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2,
            )

        return vis
