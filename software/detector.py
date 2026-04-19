# -*- coding: utf-8 -*-
"""
detector.py - YOLO 目标检测模块
=================================
使用 Ultralytics YOLOv8 进行实时目标检测。
支持自定义模型加载、ByteTrack 目标跟踪、
像素坐标到振镜坐标的映射，以及检测结果可视化。
"""

import logging
import time
from typing import Optional, List, Tuple, Dict, Any

import cv2
import numpy as np
from ultralytics import YOLO

import config
from calibrator import Calibrator

logger = logging.getLogger(__name__)


class DetectionResult:
    """
    单个目标的检测结果。

    Attributes:
        class_id: 类别 ID
        class_name: 类别名称
        confidence: 检测置信度
        bbox: 边界框 (x1, y1, x2, y2) 像素坐标
        center: 中心点 (cx, cy) 像素坐标
        track_id: 跟踪 ID（启用跟踪时有效）
        galvo_center: 振镜坐标 (gx, gy)（标定完成后有效）
    """

    def __init__(
        self,
        class_id: int,
        class_name: str,
        confidence: float,
        bbox: Tuple[float, float, float, float],
        track_id: Optional[int] = None,
    ):
        self.class_id = class_id
        self.class_name = class_name
        self.confidence = confidence
        self.bbox = bbox  # (x1, y1, x2, y2)
        self.center = (
            (bbox[0] + bbox[2]) / 2,
            (bbox[1] + bbox[3]) / 2,
        )
        self.track_id = track_id
        self.galvo_center: Optional[Tuple[float, float]] = None

    def __repr__(self):
        return (
            f"DetectionResult(class={self.class_name}, "
            f"conf={self.confidence:.2f}, "
            f"center={self.center}, "
            f"track_id={self.track_id})"
        )


class ObjectDetector:
    """
    YOLO 目标检测器。

    封装 Ultralytics YOLOv8 模型，提供实时目标检测、
    目标跟踪和坐标映射功能。
    """

    def __init__(
        self,
        model_path: str = None,
        calibrator: Optional[Calibrator] = None,
        confidence_threshold: float = None,
        iou_threshold: float = None,
        enable_tracking: bool = None,
    ):
        """
        初始化目标检测器。

        Args:
            model_path: YOLO 模型文件路径
            calibrator: 标定器实例（用于坐标映射）
            confidence_threshold: 检测置信度阈值
            iou_threshold: IoU 阈值
            enable_tracking: 是否启用目标跟踪
        """
        self.model_path = model_path or config.get_model_path()
        self.calibrator = calibrator
        self.conf_threshold = confidence_threshold or config.YOLO_CONFIDENCE_THRESHOLD
        self.iou_threshold = iou_threshold or config.YOLO_IOU_THRESHOLD
        self.enable_tracking = enable_tracking if enable_tracking is not None else config.YOLO_ENABLE_TRACKING

        # YOLO 模型
        self.model: Optional[YOLO] = None

        # 检测统计
        self._frame_count = 0
        self._fps = 0.0
        self._last_fps_time = time.time()
        self._fps_frame_count = 0

        # 上次检测时间（用于安全超时）
        self._last_detection_time = 0.0

        logger.info("目标检测器初始化完成")
        logger.info("  模型路径: %s", self.model_path)
        logger.info("  置信度阈值: %.2f", self.conf_threshold)
        logger.info("  目标跟踪: %s", "开启" if self.enable_tracking else "关闭")

    def load_model(self) -> bool:
        """
        加载 YOLO 模型。

        Returns:
            bool: 模型是否加载成功
        """
        try:
            logger.info("正在加载 YOLO 模型: %s", self.model_path)
            self.model = YOLO(self.model_path)
            logger.info("YOLO 模型加载成功")
            logger.info("  模型类别: %s", list(self.model.names.values()))
            return True

        except FileNotFoundError:
            logger.error("模型文件不存在: %s", self.model_path)
            logger.error("请将模型文件放置在 %s 目录下", config.MODEL_DIR)
            return False
        except Exception as e:
            logger.error("加载 YOLO 模型失败: %s", e)
            return False

    def detect(self, frame: np.ndarray) -> List[DetectionResult]:
        """
        对单帧图像执行目标检测。

        Args:
            frame: BGR 格式图像

        Returns:
            检测结果列表
        """
        if self.model is None:
            logger.error("模型未加载")
            return []

        if frame is None:
            logger.warning("输入图像为空")
            return []

        try:
            # 执行推理
            results = self.model(
                frame,
                conf=self.conf_threshold,
                iou=self.iou_threshold,
                imgsz=config.YOLO_IMG_SIZE,
                verbose=False,
                tracker="bytetrack.yaml" if self.enable_tracking else None,
            )

            detections = self._parse_results(results)

            # 更新 FPS 计算
            self._update_fps()

            # 更新最后检测时间
            self._last_detection_time = time.time()

            return detections

        except Exception as e:
            logger.error("目标检测异常: %s", e)
            return []

    def detect_with_tracking(self, frame: np.ndarray) -> List[DetectionResult]:
        """
        使用 ByteTrack 对单帧图像执行目标检测与跟踪。

        Args:
            frame: BGR 格式图像

        Returns:
            检测结果列表（包含跟踪 ID）
        """
        if self.model is None:
            logger.error("模型未加载")
            return []

        if frame is None:
            return []

        try:
            # 使用内置跟踪器
            results = self.model.track(
                frame,
                conf=config.YOLO_TRACK_CONF,
                iou=config.YOLO_TRACK_IOU,
                imgsz=config.YOLO_IMG_SIZE,
                verbose=False,
                tracker=config.YOLO_TRACKER_TYPE,
                persist=True,
            )

            detections = self._parse_results(results, tracking=True)

            self._update_fps()
            self._last_detection_time = time.time()

            return detections

        except Exception as e:
            logger.error("目标跟踪异常: %s", e)
            # 降级到普通检测
            return self.detect(frame)

    def _parse_results(
        self,
        results,
        tracking: bool = False,
    ) -> List[DetectionResult]:
        """
        解析 YOLO 推理结果。

        Args:
            results: Ultralytics Results 对象
            tracking: 是否解析跟踪信息

        Returns:
            检测结果列表
        """
        detections = []

        for result in results:
            if result.boxes is None:
                continue

            boxes = result.boxes
            for i in range(len(boxes)):
                # 获取边界框坐标
                x1, y1, x2, y2 = boxes.xyxy[i].cpu().numpy()

                # 获取类别信息
                class_id = int(boxes.cls[i].cpu().numpy())
                class_name = self.model.names.get(class_id, f"class_{class_id}")

                # 获取置信度
                confidence = float(boxes.conf[i].cpu().numpy())

                # 获取跟踪 ID
                track_id = None
                if tracking and boxes.id is not None:
                    track_id = int(boxes.id[i].cpu().numpy())

                # 创建检测结果
                det = DetectionResult(
                    class_id=class_id,
                    class_name=class_name,
                    confidence=confidence,
                    bbox=(float(x1), float(y1), float(x2), float(y2)),
                    track_id=track_id,
                )

                # 如果有标定数据，进行坐标映射
                if self.calibrator and self.calibrator.inverse_matrix is not None:
                    galvo_pos = self.calibrator.pixel_to_galvo(
                        det.center[0], det.center[1],
                    )
                    det.galvo_center = galvo_pos

                detections.append(det)

        return detections

    def _update_fps(self):
        """更新 FPS 计算。"""
        self._frame_count += 1
        self._fps_frame_count += 1

        current_time = time.time()
        elapsed = current_time - self._last_fps_time

        if elapsed >= 1.0:
            self._fps = self._fps_frame_count / elapsed
            self._fps_frame_count = 0
            self._last_fps_time = current_time

    @property
    def fps(self) -> float:
        """获取当前检测 FPS。"""
        return self._fps

    @property
    def frame_count(self) -> int:
        """获取总检测帧数。"""
        return self._frame_count

    @property
    def last_detection_time(self) -> float:
        """获取最后一次检测的时间戳。"""
        return self._last_detection_time

    def is_target_lost(self, timeout: float = None) -> bool:
        """
        判断目标是否已丢失。

        Args:
            timeout: 超时时间（秒），None 使用配置值

        Returns:
            bool: 目标是否已丢失
        """
        timeout = timeout or config.TARGET_LOST_TIMEOUT
        if self._last_detection_time == 0:
            return True
        return (time.time() - self._last_detection_time) > timeout

    def get_primary_target(
        self,
        detections: List[DetectionResult],
        target_class: str = None,
    ) -> Optional[DetectionResult]:
        """
        获取主要目标（置信度最高的目标）。

        Args:
            detections: 检测结果列表
            target_class: 目标类别名称，None 表示任意类别

        Returns:
            主要目标，无目标返回 None
        """
        if not detections:
            return None

        # 按类别过滤
        if target_class:
            filtered = [d for d in detections if d.class_name == target_class]
            if not filtered:
                return None
            detections = filtered

        # 返回置信度最高的目标
        return max(detections, key=lambda d: d.confidence)

    def get_closest_target(
        self,
        detections: List[DetectionResult],
        reference_point: Tuple[float, float] = None,
    ) -> Optional[DetectionResult]:
        """
        获取距离参考点最近的目标。

        Args:
            detections: 检测结果列表
            reference_point: 参考点 (x, y)，None 使用画面中心

        Returns:
            最近的目标，无目标返回 None
        """
        if not detections:
            return None

        if reference_point is None:
            # 使用画面中心
            reference_point = (config.CAMERA_WIDTH / 2, config.CAMERA_HEIGHT / 2)

        def distance(det: DetectionResult) -> float:
            dx = det.center[0] - reference_point[0]
            dy = det.center[1] - reference_point[1]
            return dx * dx + dy * dy

        return min(detections, key=distance)

    def visualize(
        self,
        frame: np.ndarray,
        detections: List[DetectionResult],
        show_boxes: bool = None,
        show_track_ids: bool = None,
        show_confidence: bool = True,
        show_fps: bool = None,
    ) -> np.ndarray:
        """
        在图像上可视化检测结果。

        Args:
            frame: 输入图像
            detections: 检测结果列表
            show_boxes: 是否显示检测框
            show_track_ids: 是否显示跟踪 ID
            show_confidence: 是否显示置信度
            show_fps: 是否显示 FPS

        Returns:
            可视化后的图像
        """
        vis = frame.copy()

        show_boxes = show_boxes if show_boxes is not None else config.DISPLAY_SHOW_BOXES
        show_track_ids = show_track_ids if show_track_ids is not None else config.DISPLAY_SHOW_TRACK_IDS
        show_fps = show_fps if show_fps is not None else config.DISPLAY_SHOW_FPS

        # 颜色表（为不同类别分配不同颜色）
        colors = [
            (0, 255, 0),    # 绿色
            (255, 0, 0),    # 蓝色
            (0, 0, 255),    # 红色
            (255, 255, 0),  # 青色
            (0, 255, 255),  # 黄色
            (255, 0, 255),  # 品红
            (128, 255, 0),  # 浅绿
            (0, 128, 255),  # 橙色
        ]

        for det in detections:
            # 选择颜色
            color = colors[det.class_id % len(colors)]

            x1, y1, x2, y2 = [int(v) for v in det.bbox]
            cx, cy = int(det.center[0]), int(det.center[1])

            if show_boxes:
                # 绘制检测框
                cv2.rectangle(vis, (x1, y1), (x2, y2), color, 2)

                # 绘制中心十字
                cv2.line(vis, (cx - 10, cy), (cx + 10, cy), color, 1)
                cv2.line(vis, (cx, cy - 10), (cx, cy + 10), color, 1)

                # 绘制中心点
                cv2.circle(vis, (cx, cy), 4, color, -1)

            # 构建标签文本
            label_parts = [det.class_name]

            if show_confidence:
                label_parts.append(f"{det.confidence:.2f}")

            if show_track_ids and det.track_id is not None:
                label_parts.append(f"ID:{det.track_id}")

            label = " ".join(label_parts)

            # 绘制标签背景和文本
            font = cv2.FONT_HERSHEY_SIMPLEX
            font_scale = 0.6
            thickness = 1

            (text_w, text_h), baseline = cv2.getTextSize(
                label, font, font_scale, thickness,
            )

            # 标签背景
            cv2.rectangle(
                vis,
                (x1, y1 - text_h - baseline - 5),
                (x1 + text_w, y1),
                color,
                -1,
            )

            # 标签文本
            cv2.putText(
                vis, label,
                (x1, y1 - baseline - 5),
                font, font_scale,
                (0, 0, 0), thickness,
            )

            # 如果有振镜坐标，显示
            if det.galvo_center:
                galvo_text = f"G({det.galvo_center[0]:.0f},{det.galvo_center[1]:.0f})"
                cv2.putText(
                    vis, galvo_text,
                    (x1, y2 + 18),
                    font, 0.5,
                    (0, 255, 255), 1,
                )

        # 显示 FPS
        if show_fps and self._fps > 0:
            fps_text = f"FPS: {self._fps:.1f}"
            cv2.putText(
                vis, fps_text, (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7,
                (0, 255, 0), 2,
            )

        # 显示检测数量
        count_text = f"Detections: {len(detections)}"
        cv2.putText(
            vis, count_text, (10, 60),
            cv2.FONT_HERSHEY_SIMPLEX, 0.6,
            (0, 255, 0), 1,
        )

        return vis

    def filter_by_area(
        self,
        detections: List[DetectionResult],
        min_area: float,
        max_area: float = float("inf"),
    ) -> List[DetectionResult]:
        """
        按目标面积过滤检测结果。

        Args:
            detections: 检测结果列表
            min_area: 最小面积（像素）
            max_area: 最大面积（像素）

        Returns:
            过滤后的检测结果列表
        """
        filtered = []
        for det in detections:
            w = det.bbox[2] - det.bbox[0]
            h = det.bbox[3] - det.bbox[1]
            area = w * h
            if min_area <= area <= max_area:
                filtered.append(det)
        return filtered

    def filter_by_class(
        self,
        detections: List[DetectionResult],
        class_names: List[str],
    ) -> List[DetectionResult]:
        """
        按类别名称过滤检测结果。

        Args:
            detections: 检测结果列表
            class_names: 允许的类别名称列表

        Returns:
            过滤后的检测结果列表
        """
        return [d for d in detections if d.class_name in class_names]
