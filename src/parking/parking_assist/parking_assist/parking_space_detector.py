#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
realtime_metric_bev_global_mapping_v5_2_temporal_vector_tracker.py

fr_cam.py의 "실제 지면 좌표 -> fisheye RAW 픽셀" 방식의 BEV를 그대로 베이스로 사용하고,
그 BEV 위에서 HSV 기반 전처리 + White Top-hat + robust threshold + 행/열 스캔 벡터화를 수행한다.

처리 순서
---------
RAW camera
 -> metric ray BEV (K, D + camera height/pitch/roll/yaw)
 -> HSV
 -> White mask / Glare mask
 -> V channel illumination normalization
 -> White Top-hat
 -> robust threshold (상위 10% 배경 통계 제외 + median/MAD)
 -> HSV soft gating
 -> morphology
 -> arbitrary-angle Hough line candidates
 -> width/color/response quality scoring
 -> dominant parking orientation + orthogonal family clustering
 -> collinear merge / final vector fit
 -> BEV metric 좌표 -> ROS base_link 좌표 변환
 -> source frame timestamp/pose 보존
 -> 전역 좌표 temporal line primitive 연관/평활/3-of-5 확인
 -> 확인된 primitive만 Global Marking Evidence Map 누적
 -> pixel 좌표 + meter 좌표 저장

중요
----
- ROI 없음.
- new_K/alpha를 사용한 중간 undistorted 영상도 만들지 않는다.
- fr_cam.py처럼 BEV map을 시작 시 한 번 계산하고, 매 프레임 cv2.remap() 한 번으로 BEV를 만든다.
- HSV는 기본적으로 "강제 AND"가 아니라 soft gate로 사용한다.
  따라서 그림자 때문에 S/V 조건에서 약간 벗어난 차선도 강한 Top-hat 응답이면 살릴 수 있다.
- Hough는 기본 2프레임마다 실행하고 중간 프레임에는 전역 temporal primitive를
  현재 pose로 다시 투영한다. 실제 Hough 프레임과 프레임당 상각 시간을 따로 출력한다.
- 현재 global map pose는 임시 static/constant provider를 사용한다.
  최종 ROS2에서는 동일한 RobotPose2D 입력 자리에 /odom 또는 map->base_link TF를 연결한다.
"""

from __future__ import annotations

import argparse
import json
import math
import yaml
import subprocess
import threading
import time
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path

import cv2
import numpy as np

try:
    from ament_index_python.packages import get_package_share_directory
    import rclpy
    from cv_bridge import CvBridge
    from geometry_msgs.msg import Point, PointStamped, PoseStamped
    from nav_msgs.msg import Odometry
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Image
    from std_msgs.msg import Bool, Float32
    from visualization_msgs.msg import Marker, MarkerArray
except ImportError:
    rclpy = None


def default_calibration_path() -> Path:
    try:
        return Path(get_package_share_directory("monoscale_odometry")) / "config" / "vision_fisheye.param.yaml"
    except (NameError, LookupError):
        return Path("vision_fisheye.param.yaml")


# ---------------------------------------------------------------------------
# 기본값: 사용자가 제공한 fr_cam.py 기준
# ---------------------------------------------------------------------------

DEFAULT_DEVICE = (
    "/dev/v4l/by-id/"
    "usb-XC-TECH_2k_usb_Camera_Redeagle_"
    "20250820.1412-video-index0"
)

DEFAULT_WIDTH = 2560
DEFAULT_HEIGHT = 1440
DEFAULT_FPS = 30

DEFAULT_CAMERA_HEIGHT_M = 0.8
DEFAULT_CAMERA_X_M = 0.08
DEFAULT_CAMERA_Y_M = 0.0
DEFAULT_PITCH_DEG = 30.0
DEFAULT_ROLL_DEG = 0.0
DEFAULT_YAW_DEG = 0.0

DEFAULT_X_RANGE_M = (-7.0, 7.0)
DEFAULT_Y_RANGE_M = (0.0, 6.0)
DEFAULT_METERS_PER_PIXEL = 0.02


@dataclass
class VectorTrack:
    orientation: str
    raw_points: np.ndarray
    polyline: np.ndarray
    metric_polyline: np.ndarray
    span_px: float
    span_m: float
    mean_width_px: float
    mean_width_m: float
    density: float
    fit_rmse_m: float
    axis_slope: float
    representative_m: float

    # v5 arbitrary-angle vector quality
    angle_deg: float = 0.0
    quality_score: float = 0.0
    binary_support: float = 0.0
    color_support: float = 0.0
    response_score: float = 0.0
    glare_ratio: float = 0.0
    dominant_error_deg: float = 0.0
    family: str = ""

    appearance_score: float = 0.0
    parallel_support: float = 0.0
    orthogonal_support: float = 0.0
    inference_score: float = 0.0

    # v5.2 temporal metadata. -1/False이면 아직 temporal association 전이다.
    temporal_track_id: int = -1
    temporal_hits_total: int = 0
    temporal_hits_in_window: int = 0
    temporal_confirmed: bool = False
    temporal_reprojected: bool = False
    source_frame_id: int = -1
    source_timestamp: float = 0.0
    source_pose_x: float = 0.0
    source_pose_y: float = 0.0
    source_pose_yaw: float = 0.0


@dataclass
class LineCandidate:
    p0_px: np.ndarray
    p1_px: np.ndarray
    p0_m: np.ndarray
    p1_m: np.ndarray
    angle_deg: float
    length_m: float
    mean_width_m: float
    binary_support: float
    color_support: float
    response_score: float
    glare_ratio: float
    quality_score: float


@dataclass
class RobotPose2D:
    """
    임시/최종 공통 Pose 표현.
    ROS base_link/map convention:
      +x = forward
      +y = left
      +yaw = counter-clockwise
    """
    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0


class TemporaryPoseProvider:
    """
    실제 /odom 연결 전 지도화 구조 검증용 Pose 공급기.

    mode=static:
      pose 고정.

    mode=constant:
      body-frame vx, vy, yaw_rate를 시간 적분.

    나중에는 이 클래스만 ROS /odom 또는 TF 기반 provider로 교체하고
    GlobalMarkingMapper는 그대로 사용하면 된다.
    """

    def __init__(self, args: argparse.Namespace):
        self.mode = str(args.pose_mode)
        self.pose = RobotPose2D(
            x=float(args.temp_pose_x_m),
            y=float(args.temp_pose_y_m),
            yaw=math.radians(
                float(args.temp_pose_yaw_deg)
            ),
        )
        self.vx = float(args.temp_vx_mps)
        self.vy = float(args.temp_vy_mps)
        self.yaw_rate = math.radians(
            float(args.temp_yaw_rate_degps)
        )
        self.last_time: float | None = None

    def update(self, now: float) -> RobotPose2D:
        if self.last_time is None:
            self.last_time = now
            return RobotPose2D(
                self.pose.x,
                self.pose.y,
                self.pose.yaw,
            )

        dt = max(
            0.0,
            min(
                now - self.last_time,
                0.25,
            ),
        )
        self.last_time = now

        if self.mode == "constant":
            c = math.cos(self.pose.yaw)
            s = math.sin(self.pose.yaw)

            # body-frame velocity -> global frame
            dx = (
                c * self.vx
                - s * self.vy
            ) * dt
            dy = (
                s * self.vx
                + c * self.vy
            ) * dt

            self.pose.x += dx
            self.pose.y += dy
            self.pose.yaw += (
                self.yaw_rate * dt
            )
            self.pose.yaw = math.atan2(
                math.sin(self.pose.yaw),
                math.cos(self.pose.yaw),
            )

        return RobotPose2D(
            self.pose.x,
            self.pose.y,
            self.pose.yaw,
        )

    def nudge(
        self,
        dx_body: float = 0.0,
        dy_body: float = 0.0,
        dyaw_rad: float = 0.0,
    ) -> None:
        c = math.cos(self.pose.yaw)
        s = math.sin(self.pose.yaw)

        self.pose.x += (
            c * dx_body
            - s * dy_body
        )
        self.pose.y += (
            s * dx_body
            + c * dy_body
        )
        self.pose.yaw += dyaw_rad
        self.pose.yaw = math.atan2(
            math.sin(self.pose.yaw),
            math.cos(self.pose.yaw),
        )


class TemporalThresholdSmoother:
    """Threshold LPF + 저주기 갱신 상태."""

    def __init__(self, alpha: float = 0.20):
        self.alpha = float(np.clip(alpha, 0.0, 1.0))
        self.value: np.ndarray | None = None
        self.frame_index = 0
        self.last_background_median_mean = 0.0
        self.last_background_mad_mean = 0.0

    def should_recompute(self, interval: int) -> bool:
        interval = max(1, int(interval))
        recompute = (
            self.value is None
            or self.frame_index % interval == 0
        )
        self.frame_index += 1
        return recompute

    def update(
        self,
        current: np.ndarray,
        background_median_mean: float | None = None,
        background_mad_mean: float | None = None,
    ) -> np.ndarray:
        current = np.asarray(current, dtype=np.float32)

        if self.value is None or self.value.shape != current.shape:
            self.value = current.copy()
        else:
            self.value = (
                (1.0 - self.alpha) * self.value
                + self.alpha * current
            )

        if background_median_mean is not None:
            self.last_background_median_mean = float(
                background_median_mean
            )

        if background_mad_mean is not None:
            self.last_background_mad_mean = float(
                background_mad_mean
            )

        return self.value.copy()

    def current(
        self,
        bands: int,
        fallback: float,
    ) -> np.ndarray:
        if self.value is None or len(self.value) != bands:
            return np.full(
                bands,
                float(fallback),
                dtype=np.float32,
            )
        return self.value.copy()


class TimingProfiler:
    """최근 N 프레임의 단계별 ms를 저장하고 평균/P95를 출력한다."""

    def __init__(self, window: int = 60):
        self.window = max(5, int(window))
        self.data: dict[str, list[float]] = {}

    def add(self, name: str, milliseconds: float) -> None:
        values = self.data.setdefault(name, [])
        values.append(float(milliseconds))

        if len(values) > self.window:
            del values[:-self.window]

    def avg(self, name: str) -> float:
        values = self.data.get(name, [])
        return float(np.mean(values)) if values else 0.0

    def p95(self, name: str) -> float:
        values = self.data.get(name, [])
        return float(np.percentile(values, 95)) if values else 0.0

    def summary_line(self, names: list[str]) -> str:
        parts = []
        for name in names:
            parts.append(
                f"{name}={self.avg(name):5.2f}ms"
            )
        return " | ".join(parts)


# ---------------------------------------------------------------------------
# 인자
# ---------------------------------------------------------------------------

def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(
        description="metric ray BEV + HSV + Top-hat + vectorization"
    )

    # Camera
    p.add_argument("--device", type=str, default=DEFAULT_DEVICE)
    p.add_argument("--width", type=int, default=DEFAULT_WIDTH)
    p.add_argument("--height", type=int, default=DEFAULT_HEIGHT)
    p.add_argument("--fps", type=int, default=DEFAULT_FPS)
    p.add_argument(
        "--image-topic",
        type=str,
        default="",
        help="설정하면 카메라 장치를 열지 않고 ROS Image를 구독",
    )
    p.add_argument(
        "--odom-topic",
        type=str,
        default="/localization/kinematic_state",
        help="ROS 이미지 모드에서 사용할 nav_msgs/Odometry",
    )
    p.add_argument(
        "--parking-lines-topic",
        type=str,
        default="/parking/lines",
        help="확정 주차선 MarkerArray 출력 토픽",
    )
    p.add_argument("--parking-goal-topic", type=str, default="/parking/goal_pose")
    p.add_argument(
        "--parking-slot-lines-topic", type=str, default="/parking/slot_lines"
    )
    p.add_argument("--debug-bev-topic", type=str, default="/parking/debug/bev")
    p.add_argument(
        "--debug-goal-bev-topic", type=str, default="/parking/debug/goal_bev"
    )
    p.add_argument(
        "--parking-distance-topic",
        type=str,
        default="/parking_line",
        help="최근접 주차선 지점과 거리 출력 토픽 prefix",
    )
    p.add_argument(
        "--calib",
        type=Path,
        default=default_calibration_path(),
    )
    p.add_argument(
        "--no-camera-controls",
        action="store_true",
        help="v4l2 exposure/gain 설정을 건드리지 않음",
    )

    # Camera pose
    p.add_argument("--camera-height-m", type=float, default=DEFAULT_CAMERA_HEIGHT_M)
    p.add_argument("--camera-x-m", type=float, default=DEFAULT_CAMERA_X_M)
    p.add_argument("--camera-y-m", type=float, default=DEFAULT_CAMERA_Y_M)
    p.add_argument("--pitch-deg", type=float, default=DEFAULT_PITCH_DEG)
    p.add_argument("--roll-deg", type=float, default=DEFAULT_ROLL_DEG)
    p.add_argument("--yaw-deg", type=float, default=DEFAULT_YAW_DEG)

    # Metric BEV
    p.add_argument("--x-min-m", type=float, default=DEFAULT_X_RANGE_M[0])
    p.add_argument("--x-max-m", type=float, default=DEFAULT_X_RANGE_M[1])
    p.add_argument("--y-min-m", type=float, default=DEFAULT_Y_RANGE_M[0])
    p.add_argument("--y-max-m", type=float, default=DEFAULT_Y_RANGE_M[1])
    p.add_argument("--meters-per-pixel", type=float, default=DEFAULT_METERS_PER_PIXEL)

    # HSV
    p.add_argument("--white-s-max", type=int, default=60)
    p.add_argument("--white-v-min", type=int, default=140)
    p.add_argument("--white-dilate", type=int, default=3)

    # Yellow lane/parking marking support (OpenCV H: 0~179)
    p.add_argument("--yellow-h-min", type=int, default=15)
    p.add_argument("--yellow-h-max", type=int, default=40)
    p.add_argument("--yellow-s-min", type=int, default=70)
    p.add_argument("--yellow-v-min", type=int, default=70)

    p.add_argument("--glare-v-min", type=int, default=248)
    p.add_argument("--glare-s-max", type=int, default=95)
    p.add_argument("--glare-min-area-ratio", type=float, default=0.006)
    p.add_argument("--glare-dilate", type=int, default=9)

    # Illumination / top-hat: meter 단위로 설정
    p.add_argument("--illumination-sigma-m", type=float, default=0.60)
    p.add_argument("--illumination-scale", type=float, default=0.35)
    p.add_argument("--normalization-scale", type=float, default=128.0)
    p.add_argument("--pre-blur", type=int, default=3)
    p.add_argument("--tophat-size-m", type=float, default=0.30)

    # Threshold
    p.add_argument("--threshold-bands", type=int, default=6)
    p.add_argument("--background-trim-percentile", type=float, default=90.0)
    p.add_argument("--background-high-percentile", type=float, default=99.0)
    p.add_argument("--mad-k", type=float, default=5.5)
    p.add_argument("--threshold-margin", type=float, default=3.0)
    p.add_argument("--threshold-min", type=float, default=8.0)
    p.add_argument("--threshold-max", type=float, default=80.0)
    p.add_argument("--threshold-lpf-alpha", type=float, default=0.20)
    p.add_argument(
        "--threshold-update-interval",
        type=int,
        default=3,
        help="threshold 통계를 몇 프레임마다 다시 계산할지",
    )
    p.add_argument(
        "--threshold-sample-stride",
        type=int,
        default=4,
        help="threshold 통계용 픽셀 샘플링 간격",
    )

    # HSV gate
    p.add_argument(
        "--strict-white",
        action="store_true",
        help="켜면 binary를 white_mask와 강제 AND. 기본은 soft gate.",
    )
    p.add_argument(
        "--strong-response-margin",
        type=float,
        default=36.0,
        help="soft gate에서 white 조건을 우회할 수 있는 Top-hat 추가 응답값",
    )
    p.add_argument(
        "--hard-glare-mask",
        action="store_true",
        help="glare 영역을 완전히 삭제. 기본은 glare-aware recovery",
    )
    p.add_argument(
        "--glare-recovery-margin",
        type=float,
        default=20.0,
        help="glare 내부에서 복구 허용할 추가 Top-hat 응답값",
    )

    # Morphology
    p.add_argument("--open-kernel", type=int, default=3)
    p.add_argument("--close-kernel", type=int, default=3)

    # Vectorization, meter 단위
    p.add_argument(
        "--vector-mode",
        choices=("dominant_hough", "scan"),
        default="dominant_hough",
        help="v5 기본: 회전 불변 dominant_hough. scan은 비교용 legacy",
    )

    # legacy scan options
    p.add_argument("--scan-stride-m", type=float, default=0.05)
    p.add_argument("--line-min-width-m", type=float, default=0.05)
    p.add_argument("--line-max-width-m", type=float, default=0.22)
    p.add_argument("--track-max-jump-m", type=float, default=0.12)
    p.add_argument("--track-max-gap", type=int, default=1)
    p.add_argument("--vertical-min-span-m", type=float, default=1.20)
    p.add_argument("--horizontal-min-span-m", type=float, default=1.15)
    p.add_argument("--min-track-points", type=int, default=12)
    p.add_argument(
        "--fit-degree",
        type=int,
        default=1,
        choices=(1, 2),
        help="metric BEV에서는 직선이 기본이므로 1 권장",
    )
    p.add_argument("--fit-outlier-k", type=float, default=3.0)
    p.add_argument(
        "--min-track-density",
        type=float,
        default=0.60,
        help="선 길이 대비 실제 scan hit 비율. 낮은 단편 오검출 제거",
    )
    p.add_argument(
        "--max-fit-rmse-m",
        type=float,
        default=0.025,
        help="직선 fit RMSE 최대값(m). 차량 테두리/문자 등 불규칙 벡터 억제",
    )
    p.add_argument(
        "--max-axis-slope",
        type=float,
        default=0.35,
        help="legacy scan 전용. dominant_hough에서는 절대각 제한을 사용하지 않음",
    )
    p.add_argument(
        "--min-vector-separation-m",
        type=float,
        default=0.18,
        help="서로 너무 가까운 중복 벡터 제거 거리",
    )
    p.add_argument("--max-vectors-each", type=int, default=8)
    p.add_argument("--vector-thickness", type=int, default=3)

    # v5 arbitrary-angle Hough + dominant orientation
    p.add_argument("--hough-threshold", type=int, default=28)
    p.add_argument("--hough-min-line-length-m", type=float, default=0.65)
    p.add_argument("--hough-max-line-gap-m", type=float, default=0.18)
    p.add_argument("--hough-max-raw-segments", type=int, default=120)

    p.add_argument("--candidate-min-length-m", type=float, default=0.70)
    p.add_argument("--candidate-min-binary-support", type=float, default=0.72)
    p.add_argument("--candidate-min-color-support", type=float, default=0.08)
    p.add_argument("--candidate-min-score", type=float, default=0.46)
    p.add_argument("--candidate-max-glare-ratio", type=float, default=0.85)
    p.add_argument("--line-target-width-m", type=float, default=0.12)

    p.add_argument("--dominant-angle-tolerance-deg", type=float, default=12.0)
    p.add_argument("--dominant-min-confidence", type=float, default=0.35)

    p.add_argument("--merge-angle-deg", type=float, default=7.0)
    p.add_argument("--merge-distance-m", type=float, default=0.20)
    p.add_argument("--merge-gap-m", type=float, default=0.35)

    p.add_argument("--final-min-span-m", type=float, default=0.85)
    p.add_argument("--final-min-score", type=float, default=0.52)

    # v5.1 geometric inference
    p.add_argument("--parallel-spacing-min-m", type=float, default=1.50)
    p.add_argument("--parallel-spacing-max-m", type=float, default=3.50)
    p.add_argument("--orthogonal-near-distance-m", type=float, default=0.45)
    p.add_argument("--inference-parallel-weight", type=float, default=0.18)
    p.add_argument("--inference-orthogonal-weight", type=float, default=0.14)
    p.add_argument("--inference-appearance-weight", type=float, default=0.68)
    p.add_argument("--inference-min-score", type=float, default=0.50)
    p.add_argument("--strong-single-line-score", type=float, default=0.72)

    # -----------------------------------------------------------------
    # v5.2 source-aware temporal vector tracking
    # -----------------------------------------------------------------
    p.add_argument(
        "--vectorize-every-n",
        type=int,
        default=2,
        help=(
            "전처리+Hough 실행 주기. 기본 2이면 30 FPS 입력에서 15 Hz로 검출하고 "
            "중간 프레임은 전역 temporal primitive를 현재 pose로 재투영"
        ),
    )
    p.add_argument("--temporal-history-size", type=int, default=5)
    p.add_argument("--temporal-confirm-hits", type=int, default=3)
    p.add_argument("--temporal-angle-gate-deg", type=float, default=7.0)
    p.add_argument("--temporal-offset-gate-m", type=float, default=0.32)
    p.add_argument("--temporal-gap-gate-m", type=float, default=0.60)
    p.add_argument("--temporal-max-missed-updates", type=int, default=3)
    p.add_argument("--temporal-ema-alpha", type=float, default=0.35)

    # -----------------------------------------------------------------
    # Global marking map
    # -----------------------------------------------------------------
    p.add_argument(
        "--pose-mode",
        choices=("static", "constant"),
        default="static",
        help="실제 odom 전 임시 pose source",
    )
    p.add_argument("--temp-pose-x-m", type=float, default=0.0)
    p.add_argument("--temp-pose-y-m", type=float, default=0.0)
    p.add_argument("--temp-pose-yaw-deg", type=float, default=0.0)

    p.add_argument(
        "--temp-vx-mps",
        type=float,
        default=0.0,
        help="constant pose mode: base_link +x(전방) 속도",
    )
    p.add_argument(
        "--temp-vy-mps",
        type=float,
        default=0.0,
        help="constant pose mode: base_link +y(좌측) 속도",
    )
    p.add_argument(
        "--temp-yaw-rate-degps",
        type=float,
        default=0.0,
        help="constant pose mode: 반시계 yaw rate",
    )

    p.add_argument("--map-x-min-m", type=float, default=-2.0)
    p.add_argument("--map-x-max-m", type=float, default=20.0)
    p.add_argument("--map-y-min-m", type=float, default=-10.0)
    p.add_argument("--map-y-max-m", type=float, default=10.0)
    p.add_argument(
        "--map-resolution-m",
        type=float,
        default=0.03,
        help="global marking map 해상도(m/cell)",
    )
    p.add_argument(
        "--map-evidence-increment",
        type=float,
        default=1.0,
    )
    p.add_argument(
        "--map-evidence-max",
        type=float,
        default=20.0,
    )
    p.add_argument(
        "--map-confirm-evidence",
        type=float,
        default=3.0,
        help="반복 관측 후 확정 마킹으로 볼 evidence",
    )
    p.add_argument(
        "--map-test-accumulate-every-frame",
        action="store_true",
        help=(
            "TEST ONLY: pose 변화가 없어도 매 프레임 evidence를 누적. "
            "정지 카메라에서 누적 동작 확인용이며 실제 이동 지도에는 사용 금지"
        ),
    )
    p.add_argument(
        "--map-min-translation-m",
        type=float,
        default=0.05,
        help="이 거리 이상 pose가 변해야 evidence 누적",
    )
    p.add_argument(
        "--map-min-yaw-deg",
        type=float,
        default=1.0,
        help="이 각도 이상 pose가 변해야 evidence 누적",
    )
    p.add_argument(
        "--show-map",
        action="store_true",
        help="Global Marking Map UI 표시",
    )
    p.add_argument(
        "--show-video",
        action="store_true",
        help="원본 카메라 영상과 Metric BEV 벡터 영상을 표시",
    )
    p.add_argument(
        "--pose-key-step-m",
        type=float,
        default=0.05,
        help="UI 수동 pose nudge 거리",
    )
    p.add_argument(
        "--pose-key-step-deg",
        type=float,
        default=1.0,
        help="UI 수동 pose nudge yaw",
    )

    # UI / save
    p.add_argument(
        "--show-ui",
        action="store_true",
        help="벡터/이진화 OpenCV UI 표시. 기본 OFF",
    )
    p.add_argument(
        "--show-debug",
        action="store_true",
        help="UI + 상세 전처리 창 표시",
    )
    p.add_argument(
        "--profile-every",
        type=int,
        default=30,
        help="몇 개의 처리 프레임마다 단계별 평균 처리시간을 터미널에 출력할지",
    )
    p.add_argument(
        "--profile-window",
        type=int,
        default=60,
        help="프로파일 평균/P95 계산에 사용할 최근 프레임 수",
    )
    p.add_argument(
        "--save-dir",
        type=Path,
        default=Path("metric_bev_hsv_vector_captures"),
    )

    # launch_ros가 덧붙이는 --ros-args와 노드 이름 remap은 rclpy가 처리한다.
    return p.parse_known_args()[0]


# ---------------------------------------------------------------------------
# 카메라 / GStreamer
# ---------------------------------------------------------------------------

def build_gst_cmd(args: argparse.Namespace) -> list[str]:
    return [
        "gst-launch-1.0",
        "-q",
        "v4l2src",
        f"device={args.device}",
        "!",
        f"image/jpeg,width={args.width},height={args.height},framerate={args.fps}/1",
        "!",
        "queue",
        "max-size-buffers=1",
        "max-size-bytes=0",
        "max-size-time=0",
        "leaky=downstream",
        "!",
        "jpegdec",
        "!",
        "videoconvert",
        "!",
        f"video/x-raw,format=BGR,width={args.width},height={args.height},framerate={args.fps}/1",
        "!",
        "queue",
        "max-size-buffers=1",
        "max-size-bytes=0",
        "max-size-time=0",
        "leaky=downstream",
        "!",
        "fdsink",
        "fd=1",
        "sync=false",
        "async=false",
    ]


def set_camera_controls(device: str) -> None:
    result = subprocess.run(
        ["v4l2-ctl", "-d", device, "--list-ctrls"],
        capture_output=True,
        text=True,
        check=False,
    )

    if result.returncode != 0:
        print("[WARN] 카메라 제어 목록 확인 실패")
        return

    supported = set()

    for line in result.stdout.splitlines():
        line = line.strip()
        if line:
            supported.add(line.split()[0])

    def apply(names: list[str], value: int) -> None:
        for name in names:
            if name not in supported:
                continue

            setting = f"{name}={value}"

            set_result = subprocess.run(
                ["v4l2-ctl", "-d", device, "--set-ctrl", setting],
                capture_output=True,
                text=True,
                check=False,
            )

            if set_result.returncode == 0:
                print(f"[CAM] {setting}")
            else:
                print(f"[WARN] 카메라 설정 실패: {setting}")
            return

    apply(["exposure_auto", "auto_exposure"], 1)
    apply(["exposure_absolute", "exposure_time_absolute"], 50)
    apply(["gain"], 0)
    apply(["brightness"], 0)
    apply(["contrast"], 32)


def read_exact(pipe, size: int) -> bytearray | None:
    data = bytearray(size)
    view = memoryview(data)
    offset = 0

    while offset < size:
        read_count = pipe.readinto(view[offset:])

        if read_count is None:
            continue

        if read_count == 0:
            return None

        offset += read_count

    return data


class LatestFrameReader:
    def __init__(
        self,
        process: subprocess.Popen,
        width: int,
        height: int,
    ):
        if process.stdout is None:
            raise RuntimeError("GStreamer stdout이 없습니다.")

        self.process = process
        self.width = width
        self.height = height
        self.frame_bytes = width * height * 3

        self.lock = threading.Lock()
        self.latest_frame: np.ndarray | None = None
        self.latest_arrival_time: float | None = None
        self.latest_input_period_ms: float | None = None
        self.previous_arrival_time: float | None = None
        self.running = True

        self.thread = threading.Thread(
            target=self._read_loop,
            daemon=True,
        )
        self.thread.start()

    def _read_loop(self) -> None:
        assert self.process.stdout is not None

        while self.running:
            raw = read_exact(
                self.process.stdout,
                self.frame_bytes,
            )

            if raw is None:
                self.running = False
                break

            frame = np.frombuffer(
                raw,
                dtype=np.uint8,
            ).reshape(
                self.height,
                self.width,
                3,
            )

            arrival = time.perf_counter()

            if self.previous_arrival_time is None:
                input_period_ms = None
            else:
                input_period_ms = (
                    arrival
                    - self.previous_arrival_time
                ) * 1000.0

            self.previous_arrival_time = arrival

            with self.lock:
                self.latest_frame = frame
                self.latest_arrival_time = arrival
                self.latest_input_period_ms = input_period_ms

    def get_latest(
        self,
    ) -> tuple[np.ndarray | None, float | None, float | None]:
        with self.lock:
            frame = self.latest_frame
            arrival = self.latest_arrival_time
            input_period_ms = self.latest_input_period_ms

            self.latest_frame = None
            self.latest_arrival_time = None
            self.latest_input_period_ms = None

        return frame, arrival, input_period_ms

    def stop(self) -> None:
        self.running = False
        if self.thread.is_alive():
            self.thread.join(timeout=1.0)


class RosFrameAndPoseSource:
    """최신 ROS 이미지와 이미지 시각에 가장 가까운 VIO pose를 제공한다."""

    def __init__(self, args: argparse.Namespace):
        if rclpy is None:
            raise RuntimeError("ROS 이미지 모드에는 rclpy와 cv_bridge가 필요합니다.")

        rclpy.init(args=None)
        self.node = rclpy.create_node("parking_line_tracker")
        self.bridge = CvBridge()
        self.lock = threading.Lock()
        self.latest_frame: np.ndarray | None = None
        self.latest_arrival_time: float | None = None
        self.latest_stamp: float | None = None
        self.latest_input_period_ms: float | None = None
        self.previous_arrival_time: float | None = None
        self.poses: list[tuple[float, RobotPose2D]] = []
        self.goal_map: tuple[float, float, float] | None = None
        self.selected_lines_base: list[np.ndarray] = []
        self.running = True
        self.lines_publisher = self.node.create_publisher(
            MarkerArray, args.parking_lines_topic, 10
        )
        self.debug_bev_publisher = self.node.create_publisher(
            Image, args.debug_bev_topic, qos_profile_sensor_data
        )
        self.debug_goal_bev_publisher = self.node.create_publisher(
            Image, args.debug_goal_bev_topic, qos_profile_sensor_data
        )
        self.nearest_point_publisher = self.node.create_publisher(
            PointStamped, f"{args.parking_distance_topic}/nearest_point", 10
        )
        self.line_valid_publisher = self.node.create_publisher(
            Bool, f"{args.parking_distance_topic}/valid", 10
        )
        self.line_distance_publisher = self.node.create_publisher(
            Float32, f"{args.parking_distance_topic}/distance", 10
        )
        self.camera_x = float(args.camera_x_m)
        self.camera_y = float(args.camera_y_m)
        self.node.create_subscription(
            Image, args.image_topic, self._image_callback, qos_profile_sensor_data
        )
        self.node.create_subscription(
            Odometry, args.odom_topic, self._odom_callback, 50
        )
        self.node.create_subscription(
            PoseStamped, args.parking_goal_topic, self._goal_callback, 10
        )
        self.node.create_subscription(
            Bool, "/parking/goal_valid", self._goal_valid_callback, 10
        )
        self.node.create_subscription(
            MarkerArray,
            args.parking_slot_lines_topic,
            self._selected_lines_callback,
            10,
        )
        self.thread = threading.Thread(target=self._spin, daemon=True)
        self.thread.start()

    @staticmethod
    def _stamp_seconds(stamp) -> float:
        return float(stamp.sec) + float(stamp.nanosec) * 1e-9

    @staticmethod
    def _yaw(q) -> float:
        return math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )

    def _spin(self) -> None:
        while self.running and rclpy.ok():
            try:
                rclpy.spin_once(self.node, timeout_sec=0.05)
            except Exception:
                if not rclpy.ok():
                    break
                raise
        self.running = False

    def _image_callback(self, message: Image) -> None:
        try:
            frame = self.bridge.imgmsg_to_cv2(message, desired_encoding="bgr8")
        except Exception as error:
            self.node.get_logger().error(f"이미지 변환 실패: {error}")
            return
        arrival = time.perf_counter()
        period = None if self.previous_arrival_time is None else (
            arrival - self.previous_arrival_time
        ) * 1000.0
        self.previous_arrival_time = arrival
        with self.lock:
            self.latest_frame = np.asarray(frame).copy()
            self.latest_arrival_time = arrival
            self.latest_stamp = self._stamp_seconds(message.header.stamp)
            self.latest_input_period_ms = period

    def _odom_callback(self, message: Odometry) -> None:
        stamp = self._stamp_seconds(message.header.stamp)
        p = message.pose.pose.position
        pose = RobotPose2D(float(p.x), float(p.y), self._yaw(message.pose.pose.orientation))
        with self.lock:
            self.poses.append((stamp, pose))
            self.poses = self.poses[-200:]

    def _goal_callback(self, message: PoseStamped) -> None:
        position = message.pose.position
        yaw = self._yaw(message.pose.orientation)
        with self.lock:
            self.goal_map = (float(position.x), float(position.y), yaw)

    def _goal_valid_callback(self, message: Bool) -> None:
        if not message.data:
            with self.lock:
                self.goal_map = None
                self.selected_lines_base = []

    def _selected_lines_callback(self, message: MarkerArray) -> None:
        lines = []
        for marker in message.markers:
            if len(marker.points) < 2:
                continue
            lines.append(np.asarray([
                [marker.points[0].x, marker.points[0].y],
                [marker.points[-1].x, marker.points[-1].y],
            ], dtype=np.float64))
        with self.lock:
            self.selected_lines_base = lines

    def selected_lines_in_base(self) -> list[np.ndarray]:
        with self.lock:
            return [line.copy() for line in self.selected_lines_base]

    def get_latest(self):
        with self.lock:
            result = (
                self.latest_frame,
                self.latest_arrival_time,
                self.latest_input_period_ms,
                self.latest_stamp,
            )
            self.latest_frame = None
            self.latest_arrival_time = None
            self.latest_input_period_ms = None
            self.latest_stamp = None
        return result

    def pose_at(self, stamp: float) -> RobotPose2D | None:
        with self.lock:
            if not self.poses:
                return None
            nearest_stamp, nearest_pose = min(self.poses, key=lambda item: abs(item[0] - stamp))
            return RobotPose2D(nearest_pose.x, nearest_pose.y, nearest_pose.yaw)

    def goal_in_base(self, pose: RobotPose2D) -> tuple[np.ndarray, float] | None:
        with self.lock:
            goal = self.goal_map
        if goal is None:
            return None
        dx = goal[0] - pose.x
        dy = goal[1] - pose.y
        c, s = math.cos(pose.yaw), math.sin(pose.yaw)
        base_xy = np.asarray([c * dx + s * dy, -s * dx + c * dy])
        base_yaw = math.atan2(
            math.sin(goal[2] - pose.yaw), math.cos(goal[2] - pose.yaw)
        )
        return base_xy, base_yaw

    def publish_debug_bev(self, image: np.ndarray, stamp: float) -> None:
        self._publish_debug_image(self.debug_bev_publisher, image, stamp)

    def publish_debug_goal_bev(self, image: np.ndarray, stamp: float) -> None:
        self._publish_debug_image(self.debug_goal_bev_publisher, image, stamp)

    def _publish_debug_image(self, publisher, image: np.ndarray, stamp: float) -> None:
        message = self.bridge.cv2_to_imgmsg(image, encoding="bgr8")
        sec = int(stamp)
        nanosec = int(round((stamp - sec) * 1e9))
        if nanosec >= 1_000_000_000:
            sec += 1
            nanosec -= 1_000_000_000
        message.header.stamp.sec = sec
        message.header.stamp.nanosec = nanosec
        message.header.frame_id = "bev"
        publisher.publish(message)

    def publish_lines(self, tracks: list[VectorTrack], stamp: float) -> None:
        output = MarkerArray()
        sec = int(stamp)
        nanosec = int(round((stamp - sec) * 1e9))
        if nanosec >= 1_000_000_000:
            sec += 1
            nanosec -= 1_000_000_000
        nearest_point = self._find_nearest_line_point(tracks)
        valid = Bool()
        valid.data = nearest_point is not None
        self.line_valid_publisher.publish(valid)
        if nearest_point is not None:
            message = PointStamped()
            message.header.frame_id = "base_link"
            message.header.stamp.sec = sec
            message.header.stamp.nanosec = nanosec
            message.point.x = float(nearest_point[0])
            message.point.y = float(nearest_point[1])
            message.point.z = 0.0
            self.nearest_point_publisher.publish(message)
            distance = Float32()
            distance.data = float(
                np.linalg.norm(
                    nearest_point - np.asarray([self.camera_x, self.camera_y])
                )
            )
            self.line_distance_publisher.publish(distance)
        for track in tracks:
            points = bev_metric_to_base_xy(track.metric_polyline)
            points[:, 0] += self.camera_x
            points[:, 1] += self.camera_y
            if len(points) < 2:
                continue
            marker = Marker()
            marker.header.frame_id = "base_link"
            marker.header.stamp.sec = sec
            marker.header.stamp.nanosec = nanosec
            marker.ns = "parking_lines"
            marker.id = int(track.temporal_track_id)
            marker.type = Marker.LINE_STRIP
            marker.action = Marker.ADD
            marker.pose.orientation.w = 1.0
            marker.scale.x = max(0.03, float(track.mean_width_m))
            marker.color.r = 1.0
            marker.color.g = 1.0
            marker.color.a = float(
                np.clip(max(track.inference_score, track.quality_score), 0.05, 1.0)
            )
            for xy in (points[0], points[-1]):
                point = Point()
                point.x, point.y = float(xy[0]), float(xy[1])
                marker.points.append(point)
            output.markers.append(marker)
        self.lines_publisher.publish(output)

    def _find_nearest_line_point(self, tracks: list[VectorTrack]) -> np.ndarray | None:
        """확정된 도색 중앙선 선분 전체에서 카메라 최근접점을 찾는다."""
        camera = np.asarray([self.camera_x, self.camera_y], dtype=np.float64)
        candidates: list[np.ndarray] = []
        for track in tracks:
            points = bev_metric_to_base_xy(track.metric_polyline)
            if len(points) < 2:
                continue
            start = np.asarray(points[0], dtype=np.float64) + camera
            end = np.asarray(points[-1], dtype=np.float64) + camera
            direction = end - start
            length_squared = float(np.dot(direction, direction))
            if length_squared < 1e-9:
                closest = start
            else:
                ratio = float(np.dot(camera - start, direction) / length_squared)
                closest = start + np.clip(ratio, 0.0, 1.0) * direction
            if closest[0] >= self.camera_x - 0.20:
                candidates.append(closest)

        if not candidates:
            return None
        return min(candidates, key=lambda point: float(np.linalg.norm(point - camera)))

    def stop(self) -> None:
        self.running = False
        if self.thread.is_alive():
            self.thread.join(timeout=1.0)
        self.node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


# ---------------------------------------------------------------------------
# 캘리브레이션 / metric ray BEV
# ---------------------------------------------------------------------------

def load_calibration(
    path: Path,
) -> tuple[np.ndarray, np.ndarray, tuple[int, int]]:
    if not path.exists():
        raise FileNotFoundError(f"캘리브레이션 파일 없음: {path}")

    if path.suffix.lower() in {".yaml", ".yml"}:
        with path.open() as stream:
            parameters = yaml.safe_load(stream)["/**"]["ros__parameters"]
        camera = parameters["front"]
        K = np.asarray(camera["k"], dtype=np.float64).reshape(3, 3)
        D = np.asarray(camera["d"], dtype=np.float64).reshape(4, 1)
        return K, D, (
            int(camera["calibration_width"]),
            int(camera["calibration_height"]),
        )

    data = np.load(path)

    K = np.asarray(
        data["K"],
        dtype=np.float64,
    ).reshape(3, 3)

    D = np.asarray(
        data["D"],
        dtype=np.float64,
    ).reshape(4, 1)

    size = np.asarray(
        data["image_size"],
    ).astype(int).reshape(-1)

    return K, D, (int(size[0]), int(size[1]))


def scale_calibration(
    K: np.ndarray,
    calibration_size: tuple[int, int],
    frame_size: tuple[int, int],
) -> np.ndarray:
    cw, ch = calibration_size
    fw, fh = frame_size

    sx = fw / float(cw)
    sy = fh / float(ch)

    out = K.copy()
    out[0, 0] *= sx
    out[0, 2] *= sx
    out[1, 1] *= sy
    out[1, 2] *= sy
    return out


def rot_x(deg: float) -> np.ndarray:
    a = np.deg2rad(deg)
    c, s = np.cos(a), np.sin(a)

    return np.asarray(
        [
            [1.0, 0.0, 0.0],
            [0.0, c, -s],
            [0.0, s, c],
        ],
        dtype=np.float64,
    )


def rot_z(deg: float) -> np.ndarray:
    a = np.deg2rad(deg)
    c, s = np.cos(a), np.sin(a)

    return np.asarray(
        [
            [c, -s, 0.0],
            [s, c, 0.0],
            [0.0, 0.0, 1.0],
        ],
        dtype=np.float64,
    )


def make_fisheye_bev_map(
    image_size: tuple[int, int],
    camera_matrix: np.ndarray,
    distortion: np.ndarray,
    camera_height_m: float,
    pitch_deg: float,
    roll_deg: float,
    yaw_deg: float,
    x_range_m: tuple[float, float],
    y_range_m: tuple[float, float],
    meters_per_pixel: float,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    source_width, source_height = image_size

    bev_width = int(
        round(
            (x_range_m[1] - x_range_m[0])
            / meters_per_pixel
        )
    )

    bev_height = int(
        round(
            (y_range_m[1] - y_range_m[0])
            / meters_per_pixel
        )
    )

    x_coordinates = np.linspace(
        x_range_m[0],
        x_range_m[1],
        bev_width,
        endpoint=False,
    )

    y_coordinates = np.linspace(
        y_range_m[1],
        y_range_m[0],
        bev_height,
        endpoint=False,
    )

    grid_x, grid_y = np.meshgrid(
        x_coordinates,
        y_coordinates,
    )

    ground_points = np.stack(
        [
            grid_x,
            grid_y,
            np.zeros_like(grid_x),
        ],
        axis=-1,
    ).reshape(-1, 3)

    camera_position_world = np.asarray(
        [0.0, 0.0, camera_height_m],
        dtype=np.float64,
    )

    base_world_to_camera = np.asarray(
        [
            [1.0, 0.0, 0.0],
            [0.0, 0.0, -1.0],
            [0.0, 1.0, 0.0],
        ],
        dtype=np.float64,
    )

    yaw_world = rot_z(-yaw_deg)
    pitch_camera = rot_x(pitch_deg)
    roll_camera = rot_z(roll_deg)

    world_to_camera = (
        roll_camera
        @ pitch_camera
        @ base_world_to_camera
        @ yaw_world
    )

    camera_to_world = world_to_camera.T

    print(
        "[BEV] optical axis world:",
        camera_to_world @ np.asarray([0.0, 0.0, 1.0]),
    )

    points_camera = (
        world_to_camera
        @ (
            ground_points
            - camera_position_world
        ).T
    ).T

    valid_depth = points_camera[:, 2] > 1e-6

    object_points = (
        points_camera
        .reshape(-1, 1, 3)
        .astype(np.float64)
    )

    zero_rvec = np.zeros((3, 1), dtype=np.float64)
    zero_tvec = np.zeros((3, 1), dtype=np.float64)

    image_points, _ = cv2.fisheye.projectPoints(
        object_points,
        zero_rvec,
        zero_tvec,
        camera_matrix,
        distortion,
    )

    image_coordinates = image_points.reshape(-1, 2)

    map_x = (
        image_coordinates[:, 0]
        .reshape(bev_height, bev_width)
        .astype(np.float32)
    )

    map_y = (
        image_coordinates[:, 1]
        .reshape(bev_height, bev_width)
        .astype(np.float32)
    )

    valid_fov = (
        valid_depth.reshape(bev_height, bev_width)
        & (map_x >= 0)
        & (map_x < source_width - 1)
        & (map_y >= 0)
        & (map_y < source_height - 1)
    )

    valid_mask = valid_fov.astype(np.uint8) * 255

    valid_ratio = (
        np.count_nonzero(valid_mask)
        / float(valid_mask.size)
    )

    print(
        f"[BEV] size={bev_width}x{bev_height}, "
        f"valid={valid_ratio * 100.0:.1f}%"
    )

    return map_x, map_y, valid_mask


# ---------------------------------------------------------------------------
# 유틸
# ---------------------------------------------------------------------------

def odd_kernel_from_meters(
    meters: float,
    meters_per_pixel: float,
    minimum: int = 3,
) -> int:
    px = max(
        minimum,
        int(round(meters / meters_per_pixel)),
    )

    if px % 2 == 0:
        px += 1

    return px


def px_from_meters(
    meters: float,
    meters_per_pixel: float,
    minimum: int = 1,
) -> int:
    return max(
        minimum,
        int(round(meters / meters_per_pixel)),
    )


def pixel_to_metric(
    points: np.ndarray,
    x_range_m: tuple[float, float],
    y_range_m: tuple[float, float],
    meters_per_pixel: float,
) -> np.ndarray:
    points = np.asarray(points, dtype=np.float32)

    x = (
        x_range_m[0]
        + (points[:, 0] + 0.5) * meters_per_pixel
    )

    y = (
        y_range_m[1]
        - (points[:, 1] + 0.5) * meters_per_pixel
    )

    return np.column_stack([x, y]).astype(np.float32)



# ---------------------------------------------------------------------------
# Local BEV vector -> Global marking map
# ---------------------------------------------------------------------------

def bev_metric_to_base_xy(
    metric_xy: np.ndarray,
) -> np.ndarray:
    """
    현재 metric BEV 좌표:
      x_bev = camera/world right
      y_bev = forward

    ROS base_link:
      x_base = forward
      y_base = left

    따라서:
      x_base = y_bev
      y_base = -x_bev
    """
    metric_xy = np.asarray(
        metric_xy,
        dtype=np.float32,
    )

    base_x = metric_xy[:, 1]
    base_y = -metric_xy[:, 0]

    return np.column_stack(
        [base_x, base_y]
    ).astype(np.float32)


def base_to_global_xy(
    base_xy: np.ndarray,
    pose: RobotPose2D,
) -> np.ndarray:
    base_xy = np.asarray(
        base_xy,
        dtype=np.float32,
    )

    c = math.cos(pose.yaw)
    s = math.sin(pose.yaw)

    x = base_xy[:, 0]
    y = base_xy[:, 1]

    global_x = (
        pose.x
        + c * x
        - s * y
    )
    global_y = (
        pose.y
        + s * x
        + c * y
    )

    return np.column_stack(
        [global_x, global_y]
    ).astype(np.float32)


def global_to_base_xy(
    global_xy: np.ndarray,
    pose: RobotPose2D,
) -> np.ndarray:
    """base_to_global_xy()의 역변환."""
    global_xy = np.asarray(global_xy, dtype=np.float32)
    dx = global_xy[:, 0] - float(pose.x)
    dy = global_xy[:, 1] - float(pose.y)
    c = math.cos(pose.yaw)
    s = math.sin(pose.yaw)
    base_x = c * dx + s * dy
    base_y = -s * dx + c * dy
    return np.column_stack([base_x, base_y]).astype(np.float32)


def base_to_bev_metric_xy(base_xy: np.ndarray) -> np.ndarray:
    """ROS base_link(+x forward, +y left) -> BEV(+x right, +y forward)."""
    base_xy = np.asarray(base_xy, dtype=np.float32)
    return np.column_stack(
        [-base_xy[:, 1], base_xy[:, 0]]
    ).astype(np.float32)


def bev_metric_to_pixel_xy(
    metric_xy: np.ndarray,
    args: argparse.Namespace,
) -> np.ndarray:
    metric_xy = np.asarray(metric_xy, dtype=np.float32)
    px = (
        (metric_xy[:, 0] - float(args.x_min_m))
        / float(args.meters_per_pixel)
        - 0.5
    )
    py = (
        (float(args.y_max_m) - metric_xy[:, 1])
        / float(args.meters_per_pixel)
        - 0.5
    )
    return np.column_stack([px, py]).astype(np.float32)


def undirected_angle_error_deg(
    p0: np.ndarray,
    p1: np.ndarray,
    q0: np.ndarray,
    q1: np.ndarray,
) -> float:
    """방향 부호를 무시한 두 직선의 각도 차이(0~90 deg)."""
    a = np.asarray(p1, dtype=np.float64) - np.asarray(p0, dtype=np.float64)
    b = np.asarray(q1, dtype=np.float64) - np.asarray(q0, dtype=np.float64)
    na = float(np.linalg.norm(a))
    nb = float(np.linalg.norm(b))
    if na <= 1e-9 or nb <= 1e-9:
        return 90.0
    cosine = float(np.clip(abs(np.dot(a, b) / (na * nb)), 0.0, 1.0))
    return math.degrees(math.acos(cosine))


@dataclass
class TemporalLinePrimitive:
    """
    Lanelet2의 영속 line-string 개념을 작게 적용한 전역 선 primitive.

    source pose/timestamp를 보존하고, hit window는 Hough가 실제 실행된
    observation update만 센다. 카메라 중간 프레임은 miss로 취급하지 않는다.
    """

    track_id: int
    family: str
    global_endpoints: np.ndarray
    prototype: VectorTrack
    first_seen_timestamp: float
    last_seen_timestamp: float
    last_source_frame_id: int
    last_source_pose: RobotPose2D
    hit_history: list[int]
    hits_total: int = 1
    missed_updates: int = 0
    confirmed: bool = False


class TemporalVectorTracker:
    """전역 좌표에서 벡터를 연관하고 3-of-5 확인하는 경량 tracker."""

    def __init__(self, args: argparse.Namespace):
        self.history_size = max(1, int(args.temporal_history_size))
        self.confirm_hits = int(
            np.clip(args.temporal_confirm_hits, 1, self.history_size)
        )
        self.angle_gate_deg = max(0.1, float(args.temporal_angle_gate_deg))
        self.offset_gate_m = max(0.01, float(args.temporal_offset_gate_m))
        self.gap_gate_m = max(0.01, float(args.temporal_gap_gate_m))
        self.max_missed_updates = max(0, int(args.temporal_max_missed_updates))
        self.ema_alpha = float(np.clip(args.temporal_ema_alpha, 0.01, 1.0))
        self.next_track_id = 1
        self.primitives: list[TemporalLinePrimitive] = []
        self.observation_update_count = 0

    @staticmethod
    def _global_endpoints(track: VectorTrack, pose: RobotPose2D) -> np.ndarray:
        metric = np.asarray(track.metric_polyline, dtype=np.float32)
        endpoints = np.vstack([metric[0], metric[-1]])
        return base_to_global_xy(bev_metric_to_base_xy(endpoints), pose)

    def _association_cost(
        self,
        primitive: TemporalLinePrimitive,
        candidate_endpoints: np.ndarray,
    ) -> float | None:
        old = primitive.global_endpoints.astype(np.float64)
        new = candidate_endpoints.astype(np.float64)
        angle_error = undirected_angle_error_deg(old[0], old[1], new[0], new[1])
        if angle_error > self.angle_gate_deg:
            return None

        direction = old[1] - old[0]
        norm = float(np.linalg.norm(direction))
        if norm <= 1e-9:
            return None
        direction /= norm
        normal = np.asarray([-direction[1], direction[0]], dtype=np.float64)

        old_mid = 0.5 * (old[0] + old[1])
        new_mid = 0.5 * (new[0] + new[1])
        offset = abs(float(np.dot(new_mid - old_mid, normal)))
        if offset > self.offset_gate_m:
            return None

        old_along = old @ direction
        new_along = new @ direction
        old_min, old_max = float(np.min(old_along)), float(np.max(old_along))
        new_min, new_max = float(np.min(new_along)), float(np.max(new_along))
        gap = max(0.0, max(old_min, new_min) - min(old_max, new_max))
        if gap > self.gap_gate_m:
            return None

        return (
            angle_error / self.angle_gate_deg
            + offset / self.offset_gate_m
            + 0.25 * gap / self.gap_gate_m
        )

    def _smooth_endpoints(
        self,
        old_endpoints: np.ndarray,
        new_endpoints: np.ndarray,
    ) -> np.ndarray:
        old = old_endpoints.astype(np.float64)
        new = new_endpoints.astype(np.float64)
        old_direction = old[1] - old[0]
        new_direction = new[1] - new[0]
        if float(np.dot(old_direction, new_direction)) < 0.0:
            new = new[::-1].copy()

        alpha = self.ema_alpha
        smoothed = (1.0 - alpha) * old + alpha * new
        return smoothed.astype(np.float32)

    @staticmethod
    def _stamp_observation(
        track: VectorTrack,
        primitive: TemporalLinePrimitive,
        pose: RobotPose2D,
        timestamp: float,
        frame_id: int,
    ) -> None:
        track.temporal_track_id = int(primitive.track_id)
        track.temporal_hits_total = int(primitive.hits_total)
        track.temporal_hits_in_window = int(sum(primitive.hit_history))
        track.temporal_confirmed = bool(primitive.confirmed)
        track.temporal_reprojected = False
        track.source_frame_id = int(frame_id)
        track.source_timestamp = float(timestamp)
        track.source_pose_x = float(pose.x)
        track.source_pose_y = float(pose.y)
        track.source_pose_yaw = float(pose.yaw)

    def update(
        self,
        observations: list[VectorTrack],
        pose: RobotPose2D,
        timestamp: float,
        frame_id: int,
    ) -> None:
        """Hough가 실제 실행된 프레임에서만 호출한다."""
        self.observation_update_count += 1

        for primitive in self.primitives:
            primitive.hit_history.append(0)
            del primitive.hit_history[:-self.history_size]
            primitive.missed_updates += 1

        candidates = [
            (track, self._global_endpoints(track, pose))
            for track in observations
        ]

        possible: list[tuple[float, int, int]] = []
        for primitive_index, primitive in enumerate(self.primitives):
            for candidate_index, (_, endpoints) in enumerate(candidates):
                cost = self._association_cost(primitive, endpoints)
                if cost is not None:
                    possible.append((cost, primitive_index, candidate_index))

        used_primitives: set[int] = set()
        used_candidates: set[int] = set()
        for _, primitive_index, candidate_index in sorted(possible):
            if primitive_index in used_primitives or candidate_index in used_candidates:
                continue
            primitive = self.primitives[primitive_index]
            track, endpoints = candidates[candidate_index]
            primitive.global_endpoints = self._smooth_endpoints(
                primitive.global_endpoints, endpoints
            )
            primitive.family = track.family or track.orientation
            primitive.prototype = track
            primitive.last_seen_timestamp = float(timestamp)
            primitive.last_source_frame_id = int(frame_id)
            primitive.last_source_pose = RobotPose2D(pose.x, pose.y, pose.yaw)
            primitive.hit_history[-1] = 1
            primitive.hits_total += 1
            primitive.missed_updates = 0
            if sum(primitive.hit_history) >= self.confirm_hits:
                primitive.confirmed = True
            self._stamp_observation(track, primitive, pose, timestamp, frame_id)
            used_primitives.add(primitive_index)
            used_candidates.add(candidate_index)

        for candidate_index, (track, endpoints) in enumerate(candidates):
            if candidate_index in used_candidates:
                continue
            primitive = TemporalLinePrimitive(
                track_id=self.next_track_id,
                family=track.family or track.orientation,
                global_endpoints=endpoints,
                prototype=track,
                first_seen_timestamp=float(timestamp),
                last_seen_timestamp=float(timestamp),
                last_source_frame_id=int(frame_id),
                last_source_pose=RobotPose2D(pose.x, pose.y, pose.yaw),
                hit_history=[1],
            )
            primitive.confirmed = self.confirm_hits <= 1
            self.next_track_id += 1
            self.primitives.append(primitive)
            self._stamp_observation(track, primitive, pose, timestamp, frame_id)

        self.primitives = [
            primitive
            for primitive in self.primitives
            if primitive.missed_updates <= self.max_missed_updates
        ]

    def project(
        self,
        pose: RobotPose2D,
        args: argparse.Namespace,
        current_frame_id: int,
        confirmed_only: bool = False,
    ) -> tuple[list[VectorTrack], list[VectorTrack]]:
        family_a: list[VectorTrack] = []
        family_b: list[VectorTrack] = []

        for primitive in self.primitives:
            if confirmed_only and not primitive.confirmed:
                continue

            base_endpoints = global_to_base_xy(primitive.global_endpoints, pose)
            metric_endpoints = base_to_bev_metric_xy(base_endpoints)
            span_m = float(np.linalg.norm(metric_endpoints[1] - metric_endpoints[0]))
            sample_count = int(
                np.clip(
                    round(span_m / max(float(args.scan_stride_m), 0.03)),
                    2,
                    160,
                )
            )
            alpha = np.linspace(0.0, 1.0, sample_count, dtype=np.float32)[:, None]
            metric_polyline = (
                metric_endpoints[0][None, :]
                + alpha * (metric_endpoints[1] - metric_endpoints[0])[None, :]
            ).astype(np.float32)
            pixel_polyline = bev_metric_to_pixel_xy(metric_polyline, args)

            track = VectorTrack(
                **{
                    **primitive.prototype.__dict__,
                    "polyline": pixel_polyline,
                    "metric_polyline": metric_polyline,
                    "span_px": span_m / float(args.meters_per_pixel),
                    "span_m": span_m,
                    "temporal_track_id": int(primitive.track_id),
                    "temporal_hits_total": int(primitive.hits_total),
                    "temporal_hits_in_window": int(sum(primitive.hit_history)),
                    "temporal_confirmed": bool(primitive.confirmed),
                    "temporal_reprojected": (
                        int(current_frame_id) != primitive.last_source_frame_id
                    ),
                    "source_frame_id": int(primitive.last_source_frame_id),
                    "source_timestamp": float(primitive.last_seen_timestamp),
                    "source_pose_x": float(primitive.last_source_pose.x),
                    "source_pose_y": float(primitive.last_source_pose.y),
                    "source_pose_yaw": float(primitive.last_source_pose.yaw),
                }
            )
            if str(primitive.family).upper() == "A" or track.orientation == "family_a":
                family_a.append(track)
            else:
                family_b.append(track)

        return family_a, family_b

    def stats(self) -> dict[str, int]:
        return {
            "active": len(self.primitives),
            "confirmed": sum(1 for item in self.primitives if item.confirmed),
            "tentative": sum(1 for item in self.primitives if not item.confirmed),
            "observation_updates": self.observation_update_count,
        }

    def to_dict(self) -> list[dict]:
        return [
            {
                "track_id": item.track_id,
                "family": item.family,
                "confirmed": item.confirmed,
                "hits_total": item.hits_total,
                "hits_in_window": int(sum(item.hit_history)),
                "hit_history": list(item.hit_history),
                "missed_updates": item.missed_updates,
                "first_seen_timestamp": item.first_seen_timestamp,
                "last_seen_timestamp": item.last_seen_timestamp,
                "last_source_frame_id": item.last_source_frame_id,
                "last_source_pose": {
                    "x": item.last_source_pose.x,
                    "y": item.last_source_pose.y,
                    "yaw": item.last_source_pose.yaw,
                },
                "global_endpoints_xy_m": item.global_endpoints.astype(float).tolist(),
            }
            for item in self.primitives
        ]


class GlobalMarkingMapper:
    """
    차선 벡터를 전역 좌표로 변환하고 evidence grid에 누적한다.

    이 클래스는 pose의 출처를 모른다.
    현재는 TemporaryPoseProvider를 사용하지만,
    나중에는 /odom 또는 map->base_link TF로 얻은 RobotPose2D를
    그대로 update()에 넣으면 된다.
    """

    def __init__(self, args: argparse.Namespace):
        self.x_min = float(args.map_x_min_m)
        self.x_max = float(args.map_x_max_m)
        self.y_min = float(args.map_y_min_m)
        self.y_max = float(args.map_y_max_m)
        self.resolution = float(
            args.map_resolution_m
        )

        if self.x_max <= self.x_min:
            raise ValueError(
                "map-x-max-m은 map-x-min-m보다 커야 합니다."
            )
        if self.y_max <= self.y_min:
            raise ValueError(
                "map-y-max-m은 map-y-min-m보다 커야 합니다."
            )
        if self.resolution <= 0:
            raise ValueError(
                "map-resolution-m은 양수여야 합니다."
            )

        # 이미지에서 세로축은 global x(전방), 가로축은 global y(좌측)
        self.height = int(
            math.ceil(
                (self.x_max - self.x_min)
                / self.resolution
            )
        )
        self.width = int(
            math.ceil(
                (self.y_max - self.y_min)
                / self.resolution
            )
        )

        self.evidence = np.zeros(
            (self.height, self.width),
            dtype=np.float32,
        )

        self.increment = float(
            args.map_evidence_increment
        )
        self.max_evidence = float(
            args.map_evidence_max
        )
        self.confirm_evidence = float(
            args.map_confirm_evidence
        )
        self.test_accumulate_every_frame = bool(
            args.map_test_accumulate_every_frame
        )
        self.min_translation = float(
            args.map_min_translation_m
        )
        self.min_yaw = math.radians(
            float(args.map_min_yaw_deg)
        )

        self.last_update_pose: RobotPose2D | None = None
        self.update_count = 0
        self.last_global_vectors: list[np.ndarray] = []
        self.last_accumulated = False

    def global_to_pixel(
        self,
        xy: np.ndarray,
    ) -> np.ndarray:
        """
        global x(+forward) -> image 위쪽
        global y(+left)    -> image 왼쪽
        """
        xy = np.asarray(
            xy,
            dtype=np.float32,
        )

        u = (
            self.y_max
            - xy[:, 1]
        ) / self.resolution

        v = (
            self.x_max
            - xy[:, 0]
        ) / self.resolution

        return np.column_stack(
            [u, v]
        ).astype(np.float32)

    def pose_to_pixel(
        self,
        pose: RobotPose2D,
    ) -> tuple[int, int]:
        pt = self.global_to_pixel(
            np.asarray(
                [[pose.x, pose.y]],
                dtype=np.float32,
            )
        )[0]

        return (
            int(round(float(pt[0]))),
            int(round(float(pt[1]))),
        )

    def should_accumulate(
        self,
        pose: RobotPose2D,
    ) -> bool:
        if self.test_accumulate_every_frame:
            return True

        if self.last_update_pose is None:
            return True

        dx = (
            pose.x
            - self.last_update_pose.x
        )
        dy = (
            pose.y
            - self.last_update_pose.y
        )
        translation = math.hypot(
            dx,
            dy,
        )

        dyaw = math.atan2(
            math.sin(
                pose.yaw
                - self.last_update_pose.yaw
            ),
            math.cos(
                pose.yaw
                - self.last_update_pose.yaw
            ),
        )

        return (
            translation >= self.min_translation
            or abs(dyaw) >= self.min_yaw
        )

    def transform_track(
        self,
        track: VectorTrack,
        pose: RobotPose2D,
    ) -> np.ndarray:
        base_xy = bev_metric_to_base_xy(
            track.metric_polyline
        )

        return base_to_global_xy(
            base_xy,
            pose,
        )

    def update(
        self,
        vertical: list[VectorTrack],
        horizontal: list[VectorTrack],
        pose: RobotPose2D,
    ) -> bool:
        all_tracks = [
            *vertical,
            *horizontal,
        ]

        global_vectors = [
            self.transform_track(
                track,
                pose,
            )
            for track in all_tracks
        ]

        self.last_global_vectors = (
            global_vectors
        )

        if not self.should_accumulate(pose):
            self.last_accumulated = False
            return False

        frame_mask = np.zeros(
            self.evidence.shape,
            dtype=np.uint8,
        )

        for track, global_xy in zip(
            all_tracks,
            global_vectors,
        ):
            pixels = self.global_to_pixel(
                global_xy
            )

            finite = np.all(
                np.isfinite(pixels),
                axis=1,
            )
            pixels = pixels[finite]

            if len(pixels) < 2:
                continue

            pts = np.round(
                pixels
            ).astype(np.int32).reshape(
                -1,
                1,
                2,
            )

            thickness = max(
                1,
                int(
                    round(
                        max(
                            track.mean_width_m,
                            self.resolution,
                        )
                        / self.resolution
                    )
                ),
            )

            cv2.polylines(
                frame_mask,
                [pts],
                False,
                255,
                thickness,
                cv2.LINE_8,
            )

        hit = frame_mask > 0

        if np.any(hit):
            self.evidence[hit] += (
                self.increment
            )
            np.minimum(
                self.evidence,
                self.max_evidence,
                out=self.evidence,
            )

        self.last_update_pose = RobotPose2D(
            pose.x,
            pose.y,
            pose.yaw,
        )
        self.update_count += 1
        self.last_accumulated = True
        return True

    def render(
        self,
        pose: RobotPose2D,
    ) -> np.ndarray:
        scale = max(
            self.max_evidence,
            1e-6,
        )

        intensity = np.clip(
            self.evidence
            / scale
            * 255.0,
            0.0,
            255.0,
        ).astype(np.uint8)

        vis = cv2.cvtColor(
            intensity,
            cv2.COLOR_GRAY2BGR,
        )

        confirmed = (
            self.evidence
            >= self.confirm_evidence
        )

        # confirmed marking은 밝게 표시
        vis[confirmed] = (
            255,
            255,
            255,
        )

        # 현재 프레임의 global vector 표시
        for global_xy in self.last_global_vectors:
            pts = np.round(
                self.global_to_pixel(
                    global_xy
                )
            ).astype(np.int32).reshape(
                -1,
                1,
                2,
            )

            if len(pts) >= 2:
                cv2.polylines(
                    vis,
                    [pts],
                    False,
                    (0, 255, 255),
                    1,
                    cv2.LINE_AA,
                )

        # 로봇 위치/방향
        u, v = self.pose_to_pixel(
            pose
        )

        if (
            0 <= u < self.width
            and 0 <= v < self.height
        ):
            cv2.circle(
                vis,
                (u, v),
                5,
                (0, 255, 0),
                -1,
                cv2.LINE_AA,
            )

            head_global = np.asarray(
                [[
                    pose.x
                    + 0.6 * math.cos(pose.yaw),
                    pose.y
                    + 0.6 * math.sin(pose.yaw),
                ]],
                dtype=np.float32,
            )

            hu, hv = np.round(
                self.global_to_pixel(
                    head_global
                )[0]
            ).astype(int)

            cv2.arrowedLine(
                vis,
                (u, v),
                (int(hu), int(hv)),
                (0, 255, 0),
                2,
                cv2.LINE_AA,
                tipLength=0.25,
            )

        evidence_max = float(
            np.max(self.evidence)
        )
        confirmed_cells = int(
            np.count_nonzero(confirmed)
        )

        cv2.rectangle(
            vis,
            (0, 0),
            (vis.shape[1], 82),
            (0, 0, 0),
            -1,
        )

        cv2.putText(
            vis,
            (
                f"GLOBAL MARKING MAP "
                f"updates={self.update_count} "
                f"acc={'Y' if self.last_accumulated else 'N'}"
            ),
            (10, 22),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (255, 255, 255),
            1,
            cv2.LINE_AA,
        )

        cv2.putText(
            vis,
            (
                f"pose x={pose.x:.2f} y={pose.y:.2f} "
                f"yaw={math.degrees(pose.yaw):.1f}deg "
                f"res={self.resolution:.3f}m"
            ),
            (10, 45),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.48,
            (220, 220, 220),
            1,
            cv2.LINE_AA,
        )

        cv2.putText(
            vis,
            (
                f"evidence_max={evidence_max:.1f} "
                f"confirmed_cells={confirmed_cells} "
                f"confirm>={self.confirm_evidence:.1f}"
            ),
            (10, 68),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.46,
            (220, 220, 220),
            1,
            cv2.LINE_AA,
        )

        return vis

    def save(
        self,
        directory: Path,
        pose: RobotPose2D,
    ) -> None:
        directory.mkdir(
            parents=True,
            exist_ok=True,
        )

        stamp = datetime.now().strftime(
            "%Y%m%d_%H%M%S"
        )

        np.save(
            directory
            / f"{stamp}_global_marking_evidence.npy",
            self.evidence,
        )

        vis = self.render(pose)

        cv2.imwrite(
            str(
                directory
                / f"{stamp}_global_marking_map.png"
            ),
            vis,
        )

        confirmed = (
            self.evidence
            >= self.confirm_evidence
        ).astype(np.uint8) * 255

        cv2.imwrite(
            str(
                directory
                / f"{stamp}_global_marking_confirmed.png"
            ),
            confirmed,
        )

        metadata = {
            "coordinate_convention": {
                "global_x": "forward",
                "global_y": "left",
                "yaw_positive": "counter_clockwise",
                "bev_to_base": {
                    "x_base": "y_bev",
                    "y_base": "-x_bev",
                },
            },
            "pose": {
                "x": pose.x,
                "y": pose.y,
                "yaw_rad": pose.yaw,
                "yaw_deg": math.degrees(
                    pose.yaw
                ),
            },
            "map": {
                "x_min_m": self.x_min,
                "x_max_m": self.x_max,
                "y_min_m": self.y_min,
                "y_max_m": self.y_max,
                "resolution_m": self.resolution,
                "width_px": self.width,
                "height_px": self.height,
                "confirm_evidence": self.confirm_evidence,
                "max_evidence": self.max_evidence,
                "update_count": self.update_count,
            },
        }

        (
            directory
            / f"{stamp}_global_marking_map.json"
        ).write_text(
            json.dumps(
                metadata,
                ensure_ascii=False,
                indent=2,
            ),
            encoding="utf-8",
        )

        print(
            f"[MAP SAVE] {directory.resolve()} / {stamp}_global_marking_*"
        )


# ---------------------------------------------------------------------------
# HSV / illumination / top-hat
# ---------------------------------------------------------------------------

def build_glare_mask(
    hsv: np.ndarray,
    valid_mask: np.ndarray,
    args: argparse.Namespace,
) -> np.ndarray:
    s = hsv[:, :, 1]
    v = hsv[:, :, 2]

    seed = (
        (s <= args.glare_s_max)
        & (v >= args.glare_v_min)
        & (valid_mask > 0)
    ).astype(np.uint8) * 255

    count, labels, stats, _ = cv2.connectedComponentsWithStats(
        seed,
        connectivity=8,
    )

    output = np.zeros_like(seed)

    min_area = max(
        1,
        int(
            round(
                valid_mask.size
                * args.glare_min_area_ratio
            )
        ),
    )

    for label in range(1, count):
        area = int(
            stats[
                label,
                cv2.CC_STAT_AREA,
            ]
        )

        if area >= min_area:
            output[labels == label] = 255

    k = max(1, int(args.glare_dilate))

    if k > 1:
        if k % 2 == 0:
            k += 1

        kernel = cv2.getStructuringElement(
            cv2.MORPH_ELLIPSE,
            (k, k),
        )

        output = cv2.dilate(
            output,
            kernel,
        )

    output[valid_mask == 0] = 0
    return output


def illumination_normalize_v(
    value_channel: np.ndarray,
    valid_mask: np.ndarray,
    args: argparse.Namespace,
) -> tuple[np.ndarray, np.ndarray]:
    source = (
        value_channel.astype(np.float32)
        + 1.0
    )

    scale = float(
        np.clip(
            args.illumination_scale,
            0.05,
            1.0,
        )
    )

    small = cv2.resize(
        source,
        None,
        fx=scale,
        fy=scale,
        interpolation=cv2.INTER_AREA,
    )

    sigma_px = (
        args.illumination_sigma_m
        / args.meters_per_pixel
        * scale
    )

    illumination_small = cv2.GaussianBlur(
        small,
        (0, 0),
        sigmaX=max(1.0, sigma_px),
        sigmaY=max(1.0, sigma_px),
        borderType=cv2.BORDER_REFLECT,
    )

    illumination = cv2.resize(
        illumination_small,
        (
            source.shape[1],
            source.shape[0],
        ),
        interpolation=cv2.INTER_LINEAR,
    )

    normalized = cv2.divide(
        source,
        illumination + 1e-6,
        scale=float(args.normalization_scale),
    )

    normalized = np.clip(
        normalized,
        0,
        255,
    ).astype(np.uint8)

    normalized[valid_mask == 0] = 0

    return normalized, illumination


def robust_threshold(
    values: np.ndarray,
    args: argparse.Namespace,
) -> tuple[float, float, float]:
    values = np.asarray(
        values,
        dtype=np.float32,
    ).reshape(-1)

    if values.size < 32:
        return (
            float(args.threshold_min),
            0.0,
            0.0,
        )

    q_trim = float(
        np.percentile(
            values,
            args.background_trim_percentile,
        )
    )

    background = values[
        values <= q_trim
    ]

    if background.size < 32:
        background = values

    median = float(
        np.median(background)
    )

    mad = float(
        np.median(
            np.abs(
                background
                - median
            )
        )
    )

    robust_sigma = 1.4826 * mad

    t_mad = (
        median
        + args.mad_k * robust_sigma
    )

    t_high = (
        float(
            np.percentile(
                background,
                args.background_high_percentile,
            )
        )
        + args.threshold_margin
    )

    threshold = float(
        np.clip(
            max(
                t_mad,
                t_high,
                args.threshold_min,
            ),
            args.threshold_min,
            args.threshold_max,
        )
    )

    return threshold, median, mad


def preprocess_bev(
    bev: np.ndarray,
    valid_mask: np.ndarray,
    threshold_smoother: TemporalThresholdSmoother,
    args: argparse.Namespace,
) -> dict[str, np.ndarray | float | list[float]]:
    profile_t0 = time.perf_counter()

    t0 = time.perf_counter()
    hsv = cv2.cvtColor(
        bev,
        cv2.COLOR_BGR2HSV,
    )

    s = hsv[:, :, 1]
    v = hsv[:, :, 2]

    white_mask = (
        (s <= args.white_s_max)
        & (v >= args.white_v_min)
        & (valid_mask > 0)
    ).astype(np.uint8) * 255

    h = hsv[:, :, 0]

    yellow_mask = (
        (h >= int(args.yellow_h_min))
        & (h <= int(args.yellow_h_max))
        & (s >= int(args.yellow_s_min))
        & (v >= int(args.yellow_v_min))
        & (valid_mask > 0)
    ).astype(np.uint8) * 255

    lane_color_mask = cv2.bitwise_or(
        white_mask,
        yellow_mask,
    )

    white_dilate = max(
        1,
        int(args.white_dilate),
    )

    if white_dilate > 1:
        if white_dilate % 2 == 0:
            white_dilate += 1

        white_kernel = cv2.getStructuringElement(
            cv2.MORPH_RECT,
            (white_dilate, white_dilate),
        )

        white_support = cv2.dilate(
            lane_color_mask,
            white_kernel,
        )
    else:
        white_support = lane_color_mask.copy()

    t_hsv_white = time.perf_counter()

    glare_mask = build_glare_mask(
        hsv,
        valid_mask,
        args,
    )

    t_glare = time.perf_counter()

    normalized, illumination = illumination_normalize_v(
        v,
        valid_mask,
        args,
    )

    t_illumination = time.perf_counter()

    pre_blur = max(
        1,
        int(args.pre_blur),
    )

    if pre_blur > 1:
        if pre_blur % 2 == 0:
            pre_blur += 1

        normalized = cv2.GaussianBlur(
            normalized,
            (pre_blur, pre_blur),
            0,
        )

        normalized[valid_mask == 0] = 0

    t_pre_blur = time.perf_counter()

    tophat_size = odd_kernel_from_meters(
        args.tophat_size_m,
        args.meters_per_pixel,
        minimum=3,
    )

    top_kernel = cv2.getStructuringElement(
        cv2.MORPH_RECT,
        (tophat_size, tophat_size),
    )

    response = cv2.morphologyEx(
        normalized,
        cv2.MORPH_TOPHAT,
        top_kernel,
    )

    response[valid_mask == 0] = 0

    if args.hard_glare_mask:
        response[glare_mask > 0] = 0

    t_tophat = time.perf_counter()

    h = response.shape[0]
    bands = max(
        1,
        min(
            int(args.threshold_bands),
            h,
        ),
    )

    edges = np.linspace(
        0,
        h,
        bands + 1,
        dtype=np.int32,
    )

    recompute_threshold = threshold_smoother.should_recompute(
        args.threshold_update_interval
    )

    if recompute_threshold:
        current_thresholds: list[float] = []
        medians: list[float] = []
        mads: list[float] = []

        sample_stride = max(
            1,
            int(args.threshold_sample_stride),
        )

        for i in range(bands):
            y0 = int(edges[i])
            y1 = int(edges[i + 1])

            band_response = response[
                y0:y1:sample_stride,
                ::sample_stride,
            ]
            band_valid = valid_mask[
                y0:y1:sample_stride,
                ::sample_stride,
            ]
            band_glare = glare_mask[
                y0:y1:sample_stride,
                ::sample_stride,
            ]

            values = band_response[
                (band_valid > 0)
                & (band_glare == 0)
            ]

            threshold, median, mad = robust_threshold(
                values,
                args,
            )

            current_thresholds.append(threshold)
            medians.append(median)
            mads.append(mad)

        thresholds = threshold_smoother.update(
            np.asarray(
                current_thresholds,
                dtype=np.float32,
            ),
            background_median_mean=(
                float(np.mean(medians))
                if medians else 0.0
            ),
            background_mad_mean=(
                float(np.mean(mads))
                if mads else 0.0
            ),
        )
    else:
        thresholds = threshold_smoother.current(
            bands,
            args.threshold_min,
        )

    t_threshold_stats = time.perf_counter()

    threshold_map = np.zeros_like(
        response,
        dtype=np.float32,
    )

    binary_raw = np.zeros_like(
        response,
        dtype=np.uint8,
    )

    for i in range(bands):
        y0 = int(edges[i])
        y1 = int(edges[i + 1])

        t = float(thresholds[i])
        threshold_map[y0:y1] = t

        binary_raw[y0:y1] = np.where(
            response[y0:y1] > t,
            255,
            0,
        ).astype(np.uint8)

    binary_raw[valid_mask == 0] = 0

    if args.hard_glare_mask:
        binary_raw[glare_mask > 0] = 0

    t_threshold_binary = time.perf_counter()

    if args.strict_white:
        binary_fused = cv2.bitwise_and(
            binary_raw,
            white_support,
        )
    else:
        # HSV가 차선을 너무 많이 죽이지 않도록 soft gate 사용.
        # 1) low saturation + bright -> white support
        # 2) 또는 threshold보다 충분히 강한 Top-hat response면 살림.
        strong_response = (
            response.astype(np.float32)
            >= (
                threshold_map
                + float(args.strong_response_margin)
            )
        ).astype(np.uint8) * 255

        support = cv2.bitwise_or(
            white_support,
            strong_response,
        )

        binary_fused = cv2.bitwise_and(
            binary_raw,
            support,
        )

    binary_fused[valid_mask == 0] = 0

    if args.hard_glare_mask:
        binary_fused[glare_mask > 0] = 0
    else:
        glare_recovery = (
            (glare_mask > 0)
            & (
                response.astype(np.float32)
                >= (
                    threshold_map
                    + float(args.glare_recovery_margin)
                )
            )
        ).astype(np.uint8) * 255

        outside_glare = binary_fused.copy()
        outside_glare[glare_mask > 0] = 0

        binary_fused = cv2.bitwise_or(
            outside_glare,
            glare_recovery,
        )

    open_k = max(1, int(args.open_kernel))

    if open_k > 1:
        if open_k % 2 == 0:
            open_k += 1

        kernel = cv2.getStructuringElement(
            cv2.MORPH_RECT,
            (open_k, open_k),
        )

        binary_fused = cv2.morphologyEx(
            binary_fused,
            cv2.MORPH_OPEN,
            kernel,
        )

    close_k = max(1, int(args.close_kernel))

    if close_k > 1:
        if close_k % 2 == 0:
            close_k += 1

        kernel = cv2.getStructuringElement(
            cv2.MORPH_RECT,
            (close_k, close_k),
        )

        binary_fused = cv2.morphologyEx(
            binary_fused,
            cv2.MORPH_CLOSE,
            kernel,
        )

    binary_fused[valid_mask == 0] = 0

    t_final = time.perf_counter()

    preprocess_profile_ms = {
        "hsv_white": (t_hsv_white - t0) * 1000.0,
        "glare": (t_glare - t_hsv_white) * 1000.0,
        "illumination": (t_illumination - t_glare) * 1000.0,
        "pre_blur": (t_pre_blur - t_illumination) * 1000.0,
        "tophat": (t_tophat - t_pre_blur) * 1000.0,
        "threshold_stats": (
            t_threshold_stats
            - t_tophat
        ) * 1000.0,
        "threshold_binary": (
            t_threshold_binary
            - t_threshold_stats
        ) * 1000.0,
        "hsv_fuse_morph": (
            t_final
            - t_threshold_binary
        ) * 1000.0,
        "preprocess_total": (
            t_final
            - profile_t0
        ) * 1000.0,
    }

    return {
        "hsv": hsv,
        "white_mask": white_mask,
        "yellow_mask": yellow_mask,
        "lane_color_mask": lane_color_mask,
        "glare_mask": glare_mask,
        "normalized": normalized,
        "illumination": illumination,
        "tophat": response,
        "threshold_map": threshold_map,
        "binary_raw": binary_raw,
        "binary_fused": binary_fused,
        "thresholds": thresholds.tolist(),
        "threshold_mean": float(np.mean(thresholds)),
        "background_median_mean": float(
            threshold_smoother.last_background_median_mean
        ),
        "background_mad_mean": float(
            threshold_smoother.last_background_mad_mean
        ),
        "threshold_recomputed": bool(recompute_threshold),
        "tophat_size_px": int(tophat_size),
        "profile_ms": preprocess_profile_ms,
    }


# ---------------------------------------------------------------------------
# 행/열 스캔 벡터화
# ---------------------------------------------------------------------------

def find_white_runs(
    line: np.ndarray,
    min_run: int,
    max_run: int,
) -> list[tuple[float, int]]:
    foreground = (
        line > 0
    ).astype(np.int8)

    padded = np.pad(
        foreground,
        (1, 1),
        mode="constant",
    )

    diff = np.diff(padded)

    starts = np.flatnonzero(
        diff == 1
    )

    ends = (
        np.flatnonzero(
            diff == -1
        )
        - 1
    )

    output: list[tuple[float, int]] = []

    for start, end in zip(
        starts,
        ends,
    ):
        width = int(
            end
            - start
            + 1
        )

        if (
            width < min_run
            or width > max_run
        ):
            continue

        output.append(
            (
                0.5
                * (
                    float(start)
                    + float(end)
                ),
                width,
            )
        )

    return output


def finalize_track(
    points: list[tuple[float, float]],
    widths: list[float],
    orientation: str,
    min_points: int,
    min_span_px: float,
    fit_degree: int,
    outlier_k: float,
    sample_step_px: int,
    args: argparse.Namespace,
) -> VectorTrack | None:
    if len(points) < min_points:
        return None

    raw = np.asarray(
        points,
        dtype=np.float32,
    )

    if orientation == "vertical":
        independent = raw[:, 1]
        dependent = raw[:, 0]
    else:
        independent = raw[:, 0]
        dependent = raw[:, 1]

    span_px = float(
        np.max(independent)
        - np.min(independent)
    )

    if span_px < min_span_px:
        return None

    expected_hits = max(
        1.0,
        span_px / float(max(sample_step_px, 1)) + 1.0,
    )
    density = float(
        len(raw)
        / expected_hits
    )

    if density < float(args.min_track_density):
        return None

    degree = min(
        int(fit_degree),
        len(raw) - 1,
    )

    try:
        coeff = np.polyfit(
            independent,
            dependent,
            degree,
        )
    except np.linalg.LinAlgError:
        return None

    predicted = np.polyval(
        coeff,
        independent,
    )

    residuals = np.abs(
        dependent
        - predicted
    )

    residual_median = float(
        np.median(residuals)
    )

    residual_mad = float(
        np.median(
            np.abs(
                residuals
                - residual_median
            )
        )
    )

    limit = (
        residual_median
        + max(
            1.0,
            outlier_k
            * 1.4826
            * residual_mad,
        )
    )

    inliers = (
        residuals
        <= limit
    )

    if (
        np.count_nonzero(inliers)
        >= max(
            min_points,
            degree + 1,
        )
    ):
        try:
            coeff = np.polyfit(
                independent[inliers],
                dependent[inliers],
                degree,
            )
        except np.linalg.LinAlgError:
            pass

    start = float(
        np.min(independent)
    )

    end = float(
        np.max(independent)
    )

    sample_axis = np.arange(
        start,
        end + sample_step_px,
        sample_step_px,
        dtype=np.float32,
    )

    sample_dep = np.polyval(
        coeff,
        sample_axis,
    ).astype(np.float32)

    if orientation == "vertical":
        polyline = np.column_stack(
            [sample_dep, sample_axis]
        )
    else:
        polyline = np.column_stack(
            [sample_axis, sample_dep]
        )

    # 최종 직선성 / 축 정렬성 검사
    final_predicted = np.polyval(
        coeff,
        independent,
    )
    final_residuals = (
        dependent
        - final_predicted
    )
    fit_rmse_px = float(
        np.sqrt(
            np.mean(
                np.square(final_residuals)
            )
        )
    )
    fit_rmse_m = (
        fit_rmse_px
        * args.meters_per_pixel
    )

    if fit_rmse_m > float(args.max_fit_rmse_m):
        return None

    p0 = polyline[0]
    p1 = polyline[-1]

    if orientation == "vertical":
        denom = max(
            abs(float(p1[1] - p0[1])),
            1e-6,
        )
        axis_slope = abs(
            float(p1[0] - p0[0])
        ) / denom
        representative_px = float(
            np.median(polyline[:, 0])
        )
        representative_m = (
            args.x_min_m
            + (representative_px + 0.5)
            * args.meters_per_pixel
        )
    else:
        denom = max(
            abs(float(p1[0] - p0[0])),
            1e-6,
        )
        axis_slope = abs(
            float(p1[1] - p0[1])
        ) / denom
        representative_px = float(
            np.median(polyline[:, 1])
        )
        representative_m = (
            args.y_max_m
            - (representative_px + 0.5)
            * args.meters_per_pixel
        )

    if axis_slope > float(args.max_axis_slope):
        return None

    metric = pixel_to_metric(
        polyline,
        (
            args.x_min_m,
            args.x_max_m,
        ),
        (
            args.y_min_m,
            args.y_max_m,
        ),
        args.meters_per_pixel,
    )

    mean_width_px = (
        float(np.mean(widths))
        if widths
        else 0.0
    )

    return VectorTrack(
        orientation=orientation,
        raw_points=raw,
        polyline=polyline.astype(np.float32),
        metric_polyline=metric,
        span_px=span_px,
        span_m=span_px * args.meters_per_pixel,
        mean_width_px=mean_width_px,
        mean_width_m=mean_width_px * args.meters_per_pixel,
        density=density,
        fit_rmse_m=fit_rmse_m,
        axis_slope=axis_slope,
        representative_m=float(representative_m),
    )


def track_scanlines(
    binary: np.ndarray,
    orientation: str,
    args: argparse.Namespace,
) -> list[VectorTrack]:
    if orientation not in (
        "vertical",
        "horizontal",
    ):
        raise ValueError("orientation 오류")

    h, w = binary.shape

    stride = px_from_meters(
        args.scan_stride_m,
        args.meters_per_pixel,
        minimum=1,
    )

    min_run = px_from_meters(
        args.line_min_width_m,
        args.meters_per_pixel,
        minimum=1,
    )

    max_run = px_from_meters(
        args.line_max_width_m,
        args.meters_per_pixel,
        minimum=min_run,
    )

    max_jump = float(
        args.track_max_jump_m
        / args.meters_per_pixel
    )

    min_span_m = (
        args.vertical_min_span_m
        if orientation == "vertical"
        else args.horizontal_min_span_m
    )

    min_span_px = (
        min_span_m
        / args.meters_per_pixel
    )

    scan_positions = (
        range(0, h, stride)
        if orientation == "vertical"
        else range(0, w, stride)
    )

    active: list[dict] = []
    finished: list[dict] = []

    for axis_position in scan_positions:
        line = (
            binary[
                axis_position,
                :,
            ]
            if orientation == "vertical"
            else binary[
                :,
                axis_position,
            ]
        )

        runs = find_white_runs(
            line,
            min_run,
            max_run,
        )

        candidates: list[
            tuple[
                tuple[float, float],
                float,
            ]
        ] = []

        for center, run_width in runs:
            if orientation == "vertical":
                point = (
                    center,
                    float(axis_position),
                )
            else:
                point = (
                    float(axis_position),
                    center,
                )

            candidates.append(
                (
                    point,
                    float(run_width),
                )
            )

        possible: list[
            tuple[
                float,
                int,
                int,
            ]
        ] = []

        for ti, track in enumerate(active):
            last_x, last_y = track["points"][-1]

            last_center = (
                last_x
                if orientation == "vertical"
                else last_y
            )

            for ci, (
                point,
                run_width,
            ) in enumerate(candidates):
                del run_width

                center = (
                    point[0]
                    if orientation == "vertical"
                    else point[1]
                )

                distance = abs(
                    center
                    - last_center
                )

                if distance <= max_jump:
                    possible.append(
                        (
                            distance,
                            ti,
                            ci,
                        )
                    )

        possible.sort(
            key=lambda item: item[0]
        )

        used_tracks: set[int] = set()
        used_candidates: set[int] = set()

        for _, ti, ci in possible:
            if (
                ti in used_tracks
                or ci in used_candidates
            ):
                continue

            point, run_width = candidates[ci]

            active[ti]["points"].append(
                point
            )

            active[ti]["widths"].append(
                run_width
            )

            active[ti]["missed"] = 0

            used_tracks.add(ti)
            used_candidates.add(ci)

        next_active: list[dict] = []

        for ti, track in enumerate(active):
            if ti not in used_tracks:
                track["missed"] += 1

            if (
                track["missed"]
                > args.track_max_gap
            ):
                finished.append(track)
            else:
                next_active.append(track)

        for ci, (
            point,
            run_width,
        ) in enumerate(candidates):
            if ci in used_candidates:
                continue

            next_active.append(
                {
                    "points": [point],
                    "widths": [run_width],
                    "missed": 0,
                }
            )

        active = next_active

    finished.extend(active)

    vectors: list[VectorTrack] = []

    for track in finished:
        vector = finalize_track(
            points=track["points"],
            widths=track["widths"],
            orientation=orientation,
            min_points=args.min_track_points,
            min_span_px=min_span_px,
            fit_degree=args.fit_degree,
            outlier_k=args.fit_outlier_k,
            sample_step_px=stride,
            args=args,
        )

        if vector is not None:
            vectors.append(vector)

    vectors.sort(
        key=lambda item: (
            item.span_m,
            item.density,
            -item.fit_rmse_m,
        ),
        reverse=True,
    )

    # 동일한 차선 주변에서 여러 벡터가 생기는 것을 억제.
    deduplicated: list[VectorTrack] = []

    for vector in vectors:
        duplicate = any(
            abs(
                vector.representative_m
                - kept.representative_m
            )
            < float(args.min_vector_separation_m)
            for kept in deduplicated
        )

        if duplicate:
            continue

        deduplicated.append(vector)

        if len(deduplicated) >= max(
            1,
            int(args.max_vectors_each),
        ):
            break

    return deduplicated



# ---------------------------------------------------------------------------
# v5: arbitrary-angle Hough + dominant parking orientation vectorizer
# ---------------------------------------------------------------------------

def angle_diff_180_deg(
    a: float,
    b: float,
) -> float:
    """Undirected line angle difference: 0~90 deg."""
    d = abs(
        ((float(a) - float(b) + 90.0) % 180.0)
        - 90.0
    )
    return float(d)


def orthogonal_family_error_deg(
    angle: float,
    base_angle: float,
) -> float:
    """
    Parking markings are modeled by two orthogonal families.
    Returns angular error to either base or base+90.
    """
    d0 = angle_diff_180_deg(
        angle,
        base_angle,
    )
    d1 = angle_diff_180_deg(
        angle,
        (base_angle + 90.0) % 180.0,
    )
    return float(min(d0, d1))


def normalize_line_angle_deg(
    angle: float,
) -> float:
    return float(angle % 180.0)


def line_angle_metric_deg(
    p0_m: np.ndarray,
    p1_m: np.ndarray,
) -> float:
    delta = (
        np.asarray(p1_m, dtype=np.float64)
        - np.asarray(p0_m, dtype=np.float64)
    )
    angle = math.degrees(
        math.atan2(
            float(delta[1]),
            float(delta[0]),
        )
    )
    return normalize_line_angle_deg(angle)


def sample_candidate_quality(
    binary: np.ndarray,
    lane_color_mask: np.ndarray,
    glare_mask: np.ndarray,
    response: np.ndarray,
    threshold_map: np.ndarray,
    distance_map: np.ndarray,
    p0_px: np.ndarray,
    p1_px: np.ndarray,
    args: argparse.Namespace,
) -> tuple[float, float, float, float, float]:
    """
    Hough segment 주변을 법선 방향으로 벡터화 샘플링하여:
      width, binary support, lane-color support, response score, glare ratio
    를 계산한다.
    """
    p0 = np.asarray(
        p0_px,
        dtype=np.float32,
    )
    p1 = np.asarray(
        p1_px,
        dtype=np.float32,
    )

    delta = p1 - p0
    length_px = float(
        np.linalg.norm(delta)
    )

    if length_px < 2.0:
        return 0.0, 0.0, 0.0, 0.0, 1.0

    direction = delta / length_px
    normal = np.asarray(
        [-direction[1], direction[0]],
        dtype=np.float32,
    )

    # Along-line samples are capped to keep Python real-time usable.
    n_samples = int(
        np.clip(
            round(length_px / 8.0),
            12,
            64,
        )
    )

    t = np.linspace(
        0.0,
        1.0,
        n_samples,
        dtype=np.float32,
    )[:, None]

    centers = (
        p0[None, :]
        + t * delta[None, :]
    )

    max_width_px = max(
        2.0,
        float(
            args.line_max_width_m
            / args.meters_per_pixel
        ),
    )

    # Search enough to locate the center of a painted stripe even when
    # Hough detected one of its edges.
    search_radius = int(
        math.ceil(
            max_width_px * 0.75
        )
    )
    search_radius = max(
        3,
        min(search_radius, 24),
    )

    offsets = np.arange(
        -search_radius,
        search_radius + 1,
        1,
        dtype=np.float32,
    )

    sample_xy = (
        centers[:, None, :]
        + offsets[None, :, None]
        * normal[None, None, :]
    )

    xs = np.rint(
        sample_xy[:, :, 0]
    ).astype(np.int32)
    ys = np.rint(
        sample_xy[:, :, 1]
    ).astype(np.int32)

    h, w = binary.shape

    valid = (
        (xs >= 0)
        & (xs < w)
        & (ys >= 0)
        & (ys < h)
    )

    xs_clip = np.clip(
        xs,
        0,
        w - 1,
    )
    ys_clip = np.clip(
        ys,
        0,
        h - 1,
    )

    binary_samples = binary[
        ys_clip,
        xs_clip,
    ]
    color_samples = lane_color_mask[
        ys_clip,
        xs_clip,
    ]
    glare_samples = glare_mask[
        ys_clip,
        xs_clip,
    ]
    dist_samples = distance_map[
        ys_clip,
        xs_clip,
    ]
    response_samples = response[
        ys_clip,
        xs_clip,
    ].astype(np.float32)
    threshold_samples = threshold_map[
        ys_clip,
        xs_clip,
    ].astype(np.float32)

    binary_samples = np.where(
        valid,
        binary_samples,
        0,
    )
    color_samples = np.where(
        valid,
        color_samples,
        0,
    )
    glare_samples = np.where(
        valid,
        glare_samples,
        0,
    )
    dist_samples = np.where(
        valid,
        dist_samples,
        0.0,
    )
    response_samples = np.where(
        valid,
        response_samples,
        0.0,
    )
    threshold_samples = np.where(
        valid,
        threshold_samples,
        255.0,
    )

    # Is the line continuously supported along its length?
    binary_support = float(
        np.mean(
            np.any(
                binary_samples > 0,
                axis=1,
            )
        )
    )

    color_support = float(
        np.mean(
            np.any(
                color_samples > 0,
                axis=1,
            )
        )
    )

    glare_ratio = float(
        np.mean(
            np.any(
                glare_samples > 0,
                axis=1,
            )
        )
    )

    # Distance transform stores distance from white pixel to nearest zero.
    # Taking the largest radius across the normal cross-section estimates
    # the painted line's full width while rejecting broad white objects.
    width_radius_px = np.max(
        dist_samples,
        axis=1,
    )

    positive_width = width_radius_px[
        width_radius_px > 0.25
    ]

    if positive_width.size == 0:
        mean_width_m = 0.0
    else:
        mean_width_m = (
            2.0
            * float(
                np.median(
                    positive_width
                )
            )
            * float(
                args.meters_per_pixel
            )
        )

    excess = np.maximum(
        response_samples
        - threshold_samples,
        0.0,
    )

    along_response = np.max(
        excess,
        axis=1,
    )

    response_score = float(
        np.clip(
            np.mean(along_response)
            / max(
                float(args.strong_response_margin),
                1.0,
            ),
            0.0,
            1.0,
        )
    )

    return (
        mean_width_m,
        binary_support,
        color_support,
        response_score,
        glare_ratio,
    )


def candidate_quality_score(
    length_m: float,
    width_m: float,
    binary_support: float,
    color_support: float,
    response_score: float,
    glare_ratio: float,
    args: argparse.Namespace,
) -> float:
    min_w = float(
        args.line_min_width_m
    )
    max_w = float(
        args.line_max_width_m
    )

    if not (
        min_w <= width_m <= max_w
    ):
        return 0.0

    target_w = float(
        np.clip(
            args.line_target_width_m,
            min_w,
            max_w,
        )
    )

    half_range = max(
        target_w - min_w,
        max_w - target_w,
        0.02,
    )

    width_score = float(
        np.clip(
            1.0
            - abs(width_m - target_w)
            / half_range,
            0.0,
            1.0,
        )
    )

    length_score = float(
        np.clip(
            (
                length_m
                - float(args.candidate_min_length_m)
            )
            / 2.0,
            0.0,
            1.0,
        )
    )

    base = (
        0.28 * float(binary_support)
        + 0.20 * float(color_support)
        + 0.20 * float(width_score)
        + 0.14 * float(response_score)
        + 0.18 * float(length_score)
    )

    # Glare is a penalty, not a hard deletion.
    glare_factor = (
        1.0
        - 0.45
        * float(
            np.clip(
                glare_ratio,
                0.0,
                1.0,
            )
        )
    )

    return float(
        np.clip(
            base * glare_factor,
            0.0,
            1.0,
        )
    )


def extract_hough_candidates(
    binary: np.ndarray,
    prep: dict,
    args: argparse.Namespace,
) -> list[LineCandidate]:
    # Edge-based Hough handles T-junctions and arbitrary rotation better
    # than row/column scan tracking.
    edges = cv2.Canny(
        binary,
        50,
        150,
        apertureSize=3,
        L2gradient=False,
    )

    min_len_px = max(
        10,
        int(
            round(
                float(
                    args.hough_min_line_length_m
                )
                / args.meters_per_pixel
            )
        ),
    )

    max_gap_px = max(
        1,
        int(
            round(
                float(
                    args.hough_max_line_gap_m
                )
                / args.meters_per_pixel
            )
        ),
    )

    lines = cv2.HoughLinesP(
        edges,
        rho=1.0,
        theta=np.pi / 360.0,
        threshold=max(
            1,
            int(args.hough_threshold),
        ),
        minLineLength=min_len_px,
        maxLineGap=max_gap_px,
    )

    if lines is None:
        return []

    raw_segments = []

    for line in lines[:, 0, :]:
        x0, y0, x1, y1 = [
            float(v)
            for v in line
        ]

        length_px = math.hypot(
            x1 - x0,
            y1 - y0,
        )
        raw_segments.append(
            (
                length_px,
                np.asarray(
                    [x0, y0],
                    dtype=np.float32,
                ),
                np.asarray(
                    [x1, y1],
                    dtype=np.float32,
                ),
            )
        )

    raw_segments.sort(
        key=lambda item: item[0],
        reverse=True,
    )

    raw_segments = raw_segments[
        : max(
            1,
            int(
                args.hough_max_raw_segments
            ),
        )
    ]

    distance_map = cv2.distanceTransform(
        binary,
        cv2.DIST_L2,
        3,
    )

    candidates: list[
        LineCandidate
    ] = []

    x_range = (
        args.x_min_m,
        args.x_max_m,
    )
    y_range = (
        args.y_min_m,
        args.y_max_m,
    )

    for length_px, p0_px, p1_px in raw_segments:
        length_m = (
            float(length_px)
            * float(
                args.meters_per_pixel
            )
        )

        if length_m < float(
            args.candidate_min_length_m
        ):
            continue

        metric = pixel_to_metric(
            np.vstack(
                [p0_px, p1_px]
            ),
            x_range,
            y_range,
            args.meters_per_pixel,
        )

        p0_m = metric[0]
        p1_m = metric[1]

        (
            width_m,
            binary_support,
            color_support,
            response_score,
            glare_ratio,
        ) = sample_candidate_quality(
            binary=binary,
            lane_color_mask=prep[
                "lane_color_mask"
            ],
            glare_mask=prep[
                "glare_mask"
            ],
            response=prep[
                "tophat"
            ],
            threshold_map=prep[
                "threshold_map"
            ],
            distance_map=distance_map,
            p0_px=p0_px,
            p1_px=p1_px,
            args=args,
        )

        if (
            binary_support
            < float(
                args.candidate_min_binary_support
            )
        ):
            continue

        if (
            color_support
            < float(
                args.candidate_min_color_support
            )
            and response_score < 0.70
        ):
            continue

        if glare_ratio > float(
            args.candidate_max_glare_ratio
        ):
            continue

        score = candidate_quality_score(
            length_m=length_m,
            width_m=width_m,
            binary_support=binary_support,
            color_support=color_support,
            response_score=response_score,
            glare_ratio=glare_ratio,
            args=args,
        )

        if score < float(
            args.candidate_min_score
        ):
            continue

        candidates.append(
            LineCandidate(
                p0_px=p0_px,
                p1_px=p1_px,
                p0_m=p0_m,
                p1_m=p1_m,
                angle_deg=line_angle_metric_deg(
                    p0_m,
                    p1_m,
                ),
                length_m=length_m,
                mean_width_m=width_m,
                binary_support=binary_support,
                color_support=color_support,
                response_score=response_score,
                glare_ratio=glare_ratio,
                quality_score=score,
            )
        )

    return candidates


def estimate_dominant_orientation(
    candidates: list[LineCandidate],
    args: argparse.Namespace,
) -> tuple[float | None, float]:
    """
    Robust Manhattan-frame estimator for parking markings.

    A proposal angle explains both:
      theta
      theta + 90 deg

    so the vehicle/camera can be rotated arbitrarily relative to the slots.
    """
    if not candidates:
        return None, 0.0

    tolerance = float(
        args.dominant_angle_tolerance_deg
    )

    weights = np.asarray(
        [
            min(c.length_m, 3.0)
            * max(
                c.quality_score,
                0.05,
            ) ** 2
            for c in candidates
        ],
        dtype=np.float64,
    )

    total_weight = float(
        np.sum(weights)
    )

    if total_weight <= 1e-9:
        return (
            float(candidates[0].angle_deg),
            1.0,
        )

    best_base = float(
        candidates[0].angle_deg
    )
    best_support = -1.0
    best_inliers = None

    for proposal in candidates:
        errors = np.asarray(
            [
                orthogonal_family_error_deg(
                    c.angle_deg,
                    proposal.angle_deg,
                )
                for c in candidates
            ],
            dtype=np.float64,
        )

        inliers = errors <= tolerance

        support = float(
            np.sum(
                weights[inliers]
            )
        )

        if support > best_support:
            best_support = support
            best_base = float(
                proposal.angle_deg
            )
            best_inliers = inliers

    confidence = float(
        best_support
        / max(
            total_weight,
            1e-9,
        )
    )

    if best_inliers is None:
        return best_base % 90.0, confidence

    # Refine orientation modulo 90 deg using exp(j*4theta).
    angles_rad = np.radians(
        np.asarray(
            [
                c.angle_deg
                for c in candidates
            ],
            dtype=np.float64,
        )
    )

    inlier_weights = np.where(
        best_inliers,
        weights,
        0.0,
    )

    z_real = float(
        np.sum(
            inlier_weights
            * np.cos(
                4.0 * angles_rad
            )
        )
    )
    z_imag = float(
        np.sum(
            inlier_weights
            * np.sin(
                4.0 * angles_rad
            )
        )
    )

    if (
        abs(z_real)
        + abs(z_imag)
        > 1e-9
    ):
        refined = (
            math.degrees(
                0.25
                * math.atan2(
                    z_imag,
                    z_real,
                )
            )
            % 90.0
        )
    else:
        refined = best_base % 90.0

    return float(refined), confidence


def family_target_angle(
    angle_deg: float,
    dominant_deg: float,
) -> tuple[str, float, float]:
    a = normalize_line_angle_deg(
        dominant_deg
    )
    b = normalize_line_angle_deg(
        dominant_deg + 90.0
    )

    da = angle_diff_180_deg(
        angle_deg,
        a,
    )
    db = angle_diff_180_deg(
        angle_deg,
        b,
    )

    if da <= db:
        return "A", a, float(da)

    return "B", b, float(db)


def merge_family_candidates(
    candidates: list[
        tuple[LineCandidate, float]
    ],
    family: str,
    target_angle_deg: float,
    args: argparse.Namespace,
) -> list[VectorTrack]:
    if not candidates:
        return []

    theta = math.radians(
        target_angle_deg
    )

    direction = np.asarray(
        [
            math.cos(theta),
            math.sin(theta),
        ],
        dtype=np.float64,
    )

    normal = np.asarray(
        [
            -math.sin(theta),
            math.cos(theta),
        ],
        dtype=np.float64,
    )

    items = []

    for candidate, angle_error in candidates:
        pts = np.vstack(
            [
                candidate.p0_m,
                candidate.p1_m,
            ]
        ).astype(np.float64)

        along = pts @ direction
        across = pts @ normal

        items.append(
            {
                "candidate": candidate,
                "angle_error": float(
                    angle_error
                ),
                "along_min": float(
                    np.min(along)
                ),
                "along_max": float(
                    np.max(along)
                ),
                "across": float(
                    np.mean(across)
                ),
            }
        )

    items.sort(
        key=lambda item: (
            item["across"],
            item["along_min"],
        )
    )

    clusters: list[dict] = []

    merge_distance = float(
        args.merge_distance_m
    )
    merge_gap = float(
        args.merge_gap_m
    )

    for item in items:
        best_index = None
        best_cost = float("inf")

        for i, cluster in enumerate(clusters):
            across_error = abs(
                item["across"]
                - cluster["across_mean"]
            )

            if across_error > merge_distance:
                continue

            gap = max(
                cluster["along_min"]
                - item["along_max"],
                item["along_min"]
                - cluster["along_max"],
                0.0,
            )

            if gap > merge_gap:
                continue

            cost = (
                across_error
                + 0.25 * gap
            )

            if cost < best_cost:
                best_cost = cost
                best_index = i

        if best_index is None:
            candidate = item[
                "candidate"
            ]
            weight = max(
                candidate.quality_score
                * candidate.length_m,
                1e-3,
            )

            clusters.append(
                {
                    "items": [item],
                    "weights": [weight],
                    "across_mean": item[
                        "across"
                    ],
                    "along_min": item[
                        "along_min"
                    ],
                    "along_max": item[
                        "along_max"
                    ],
                }
            )
            continue

        cluster = clusters[
            best_index
        ]

        candidate = item[
            "candidate"
        ]
        weight = max(
            candidate.quality_score
            * candidate.length_m,
            1e-3,
        )

        cluster["items"].append(
            item
        )
        cluster["weights"].append(
            weight
        )

        all_weights = np.asarray(
            cluster["weights"],
            dtype=np.float64,
        )
        all_across = np.asarray(
            [
                x["across"]
                for x in cluster["items"]
            ],
            dtype=np.float64,
        )

        cluster["across_mean"] = float(
            np.average(
                all_across,
                weights=all_weights,
            )
        )
        cluster["along_min"] = min(
            cluster["along_min"],
            item["along_min"],
        )
        cluster["along_max"] = max(
            cluster["along_max"],
            item["along_max"],
        )

    vectors: list[VectorTrack] = []

    for cluster in clusters:
        span_m = (
            cluster["along_max"]
            - cluster["along_min"]
        )

        if span_m < float(
            args.final_min_span_m
        ):
            continue

        raw_candidates = [
            item["candidate"]
            for item in cluster["items"]
        ]

        weights = np.asarray(
            cluster["weights"],
            dtype=np.float64,
        )

        quality = float(
            np.average(
                np.asarray(
                    [
                        c.quality_score
                        for c in raw_candidates
                    ],
                    dtype=np.float64,
                ),
                weights=weights,
            )
        )

        binary_support = float(
            np.average(
                np.asarray(
                    [
                        c.binary_support
                        for c in raw_candidates
                    ],
                    dtype=np.float64,
                ),
                weights=weights,
            )
        )
        color_support = float(
            np.average(
                np.asarray(
                    [
                        c.color_support
                        for c in raw_candidates
                    ],
                    dtype=np.float64,
                ),
                weights=weights,
            )
        )
        response_score = float(
            np.average(
                np.asarray(
                    [
                        c.response_score
                        for c in raw_candidates
                    ],
                    dtype=np.float64,
                ),
                weights=weights,
            )
        )
        glare_ratio = float(
            np.average(
                np.asarray(
                    [
                        c.glare_ratio
                        for c in raw_candidates
                    ],
                    dtype=np.float64,
                ),
                weights=weights,
            )
        )
        width_m = float(
            np.average(
                np.asarray(
                    [
                        c.mean_width_m
                        for c in raw_candidates
                    ],
                    dtype=np.float64,
                ),
                weights=weights,
            )
        )
        dominant_error = float(
            np.average(
                np.asarray(
                    [
                        item["angle_error"]
                        for item in cluster[
                            "items"
                        ]
                    ],
                    dtype=np.float64,
                ),
                weights=weights,
            )
        )

        # Reward repeated collinear Hough support but avoid requiring it.
        repeated_bonus = min(
            0.10,
            0.025
            * max(
                len(raw_candidates) - 1,
                0,
            ),
        )

        angle_score = float(
            np.clip(
                1.0
                - dominant_error
                / max(
                    float(
                        args.dominant_angle_tolerance_deg
                    ),
                    1.0,
                ),
                0.0,
                1.0,
            )
        )

        final_score = float(
            np.clip(
                0.82 * quality
                + 0.18 * angle_score
                + repeated_bonus,
                0.0,
                1.0,
            )
        )

        if final_score < float(
            args.final_min_score
        ):
            continue

        # 색 지지가 매우 약한 경우에는 "강한 Top-hat + 충분한 길이"가
        # 동시에 있어야만 살린다. 차량 반사/휠 테두리처럼 밝고 얇지만
        # 실제 white/yellow marking이 아닌 짧은 선을 억제하기 위한 rescue gate.
        if color_support < 0.12:
            if not (
                span_m >= 1.50
                and response_score >= 0.80
                and final_score >= 0.62
            ):
                continue

        across_values = np.asarray(
            [
                item["across"]
                for item in cluster["items"]
            ],
            dtype=np.float64,
        )

        if len(across_values) > 1:
            fit_rmse_m = float(
                np.sqrt(
                    np.average(
                        np.square(
                            across_values
                            - cluster[
                                "across_mean"
                            ]
                        ),
                        weights=weights,
                    )
                )
            )
        else:
            fit_rmse_m = 0.0

        p0_m = (
            direction
            * cluster["along_min"]
            + normal
            * cluster["across_mean"]
        )
        p1_m = (
            direction
            * cluster["along_max"]
            + normal
            * cluster["across_mean"]
        )

        # metric -> pixel
        def metric_to_pixel_point(
            point_m: np.ndarray,
        ) -> np.ndarray:
            px = (
                (
                    float(point_m[0])
                    - float(args.x_min_m)
                )
                / float(
                    args.meters_per_pixel
                )
                - 0.5
            )
            py = (
                (
                    float(args.y_max_m)
                    - float(point_m[1])
                )
                / float(
                    args.meters_per_pixel
                )
                - 0.5
            )
            return np.asarray(
                [px, py],
                dtype=np.float32,
            )

        p0_px = metric_to_pixel_point(
            p0_m
        )
        p1_px = metric_to_pixel_point(
            p1_m
        )

        sample_count = int(
            np.clip(
                round(
                    span_m
                    / max(
                        float(
                            args.scan_stride_m
                        ),
                        0.03,
                    )
                ),
                2,
                160,
            )
        )

        alpha = np.linspace(
            0.0,
            1.0,
            sample_count,
            dtype=np.float32,
        )[:, None]

        metric_polyline = (
            p0_m[None, :]
            + alpha
            * (
                p1_m
                - p0_m
            )[None, :]
        ).astype(np.float32)

        polyline = (
            p0_px[None, :]
            + alpha
            * (
                p1_px
                - p0_px
            )[None, :]
        ).astype(np.float32)

        raw_points = np.vstack(
            [
                np.vstack(
                    [
                        c.p0_px,
                        c.p1_px,
                    ]
                )
                for c in raw_candidates
            ]
        ).astype(np.float32)

        representative_m = float(
            cluster["across_mean"]
        )

        vectors.append(
            VectorTrack(
                orientation=(
                    "family_a"
                    if family == "A"
                    else "family_b"
                ),
                raw_points=raw_points,
                polyline=polyline,
                metric_polyline=metric_polyline,
                span_px=(
                    span_m
                    / args.meters_per_pixel
                ),
                span_m=float(span_m),
                mean_width_px=(
                    width_m
                    / args.meters_per_pixel
                ),
                mean_width_m=width_m,
                density=binary_support,
                fit_rmse_m=fit_rmse_m,
                axis_slope=0.0,
                representative_m=representative_m,
                angle_deg=float(
                    target_angle_deg
                ),
                quality_score=final_score,
                binary_support=binary_support,
                color_support=color_support,
                response_score=response_score,
                glare_ratio=glare_ratio,
                dominant_error_deg=dominant_error,
                family=family,
                appearance_score=final_score,
                parallel_support=0.0,
                orthogonal_support=0.0,
                inference_score=final_score,
            )
        )

    vectors.sort(
        key=lambda v: (
            v.quality_score,
            v.span_m,
            v.binary_support,
        ),
        reverse=True,
    )

    return vectors[
        : max(
            1,
            int(
                args.max_vectors_each
            ),
        )
    ]



def point_segment_distance_m(
    point: np.ndarray,
    a: np.ndarray,
    b: np.ndarray,
) -> float:
    p = np.asarray(point, dtype=np.float64)
    a = np.asarray(a, dtype=np.float64)
    b = np.asarray(b, dtype=np.float64)

    ab = b - a
    denom = float(np.dot(ab, ab))
    if denom <= 1e-12:
        return float(np.linalg.norm(p - a))

    t = float(
        np.clip(
            np.dot(p - a, ab) / denom,
            0.0,
            1.0,
        )
    )
    closest = a + t * ab
    return float(np.linalg.norm(p - closest))


def segment_near_distance_m(
    a0: np.ndarray,
    a1: np.ndarray,
    b0: np.ndarray,
    b1: np.ndarray,
) -> float:
    return min(
        point_segment_distance_m(a0, b0, b1),
        point_segment_distance_m(a1, b0, b1),
        point_segment_distance_m(b0, a0, a1),
        point_segment_distance_m(b1, a0, a1),
    )


def vector_endpoints_m(
    track: VectorTrack,
) -> tuple[np.ndarray, np.ndarray]:
    return (
        np.asarray(track.metric_polyline[0], dtype=np.float64),
        np.asarray(track.metric_polyline[-1], dtype=np.float64),
    )


def add_geometric_inference(
    family_a: list[VectorTrack],
    family_b: list[VectorTrack],
    args: argparse.Namespace,
) -> tuple[list[VectorTrack], list[VectorTrack]]:
    min_spacing = float(args.parallel_spacing_min_m)
    max_spacing = float(args.parallel_spacing_max_m)
    near_limit = float(args.orthogonal_near_distance_m)

    appearance_w = float(args.inference_appearance_weight)
    parallel_w = float(args.inference_parallel_weight)
    orthogonal_w = float(args.inference_orthogonal_weight)

    def score_one(
        track: VectorTrack,
        same_family: list[VectorTrack],
        other_family: list[VectorTrack],
    ) -> None:
        track.appearance_score = float(track.quality_score)

        best_parallel = 0.0
        for other in same_family:
            if other is track:
                continue

            spacing = abs(
                float(track.representative_m)
                - float(other.representative_m)
            )
            if not (min_spacing <= spacing <= max_spacing):
                continue

            center = 0.5 * (min_spacing + max_spacing)
            half = max(0.5 * (max_spacing - min_spacing), 1e-6)
            spacing_score = float(
                np.clip(
                    1.0 - abs(spacing - center) / half,
                    0.0,
                    1.0,
                )
            )
            partner_score = float(
                np.clip(
                    0.55 * other.quality_score
                    + 0.45 * min(other.span_m / 2.0, 1.0),
                    0.0,
                    1.0,
                )
            )
            best_parallel = max(
                best_parallel,
                0.55 * spacing_score + 0.45 * partner_score,
            )

        track.parallel_support = float(best_parallel)

        best_orthogonal = 0.0
        a0, a1 = vector_endpoints_m(track)
        for other in other_family:
            b0, b1 = vector_endpoints_m(other)
            distance = segment_near_distance_m(a0, a1, b0, b1)
            if distance > near_limit:
                continue

            near_score = float(
                np.clip(
                    1.0 - distance / max(near_limit, 1e-6),
                    0.0,
                    1.0,
                )
            )
            partner_score = float(
                np.clip(
                    0.65 * other.quality_score
                    + 0.35 * min(other.span_m / 1.5, 1.0),
                    0.0,
                    1.0,
                )
            )
            best_orthogonal = max(
                best_orthogonal,
                0.60 * near_score + 0.40 * partner_score,
            )

        track.orthogonal_support = float(best_orthogonal)

        total_w = max(
            appearance_w + parallel_w + orthogonal_w,
            1e-6,
        )
        inference = (
            appearance_w * track.appearance_score
            + parallel_w * track.parallel_support
            + orthogonal_w * track.orthogonal_support
        ) / total_w

        continuity_bonus = min(
            0.08,
            0.04
            * min(track.span_m / 2.0, 1.0)
            * float(track.binary_support),
        )

        track.inference_score = float(
            np.clip(
                inference + continuity_bonus,
                0.0,
                1.0,
            )
        )

    for same_family, other_family in (
        (family_a, family_b),
        (family_b, family_a),
    ):
        for track in same_family:
            score_one(track, same_family, other_family)

    def keep(track: VectorTrack) -> bool:
        if track.inference_score >= float(args.inference_min_score):
            return True

        if (
            track.appearance_score >= float(args.strong_single_line_score)
            and track.span_m >= 1.20
            and track.binary_support >= 0.85
        ):
            return True

        return False

    filtered_a = [v for v in family_a if keep(v)]
    filtered_b = [v for v in family_b if keep(v)]

    filtered_a.sort(
        key=lambda v: (
            v.inference_score,
            v.appearance_score,
            v.span_m,
        ),
        reverse=True,
    )
    filtered_b.sort(
        key=lambda v: (
            v.inference_score,
            v.appearance_score,
            v.span_m,
        ),
        reverse=True,
    )

    return filtered_a, filtered_b


def dominant_hough_vectorize(
    binary: np.ndarray,
    prep: dict,
    args: argparse.Namespace,
) -> tuple[
    list[VectorTrack],
    list[VectorTrack],
    dict,
]:
    candidates = extract_hough_candidates(
        binary,
        prep,
        args,
    )

    dominant_deg, dominant_confidence = (
        estimate_dominant_orientation(
            candidates,
            args,
        )
    )

    debug = {
        "raw_candidate_count": len(
            candidates
        ),
        "dominant_angle_deg": (
            float(dominant_deg)
            if dominant_deg is not None
            else None
        ),
        "dominant_confidence": float(
            dominant_confidence
        ),
    }

    if dominant_deg is None:
        return [], [], debug

    tolerance = float(
        args.dominant_angle_tolerance_deg
    )

    family_a_candidates: list[
        tuple[
            LineCandidate,
            float,
        ]
    ] = []
    family_b_candidates: list[
        tuple[
            LineCandidate,
            float,
        ]
    ] = []

    for candidate in candidates:
        (
            family,
            _target,
            error_deg,
        ) = family_target_angle(
            candidate.angle_deg,
            dominant_deg,
        )

        # There is intentionally NO absolute 0/90-degree restriction.
        if error_deg > tolerance:
            continue

        if family == "A":
            family_a_candidates.append(
                (
                    candidate,
                    error_deg,
                )
            )
        else:
            family_b_candidates.append(
                (
                    candidate,
                    error_deg,
                )
            )

    family_a = merge_family_candidates(
        family_a_candidates,
        "A",
        dominant_deg,
        args,
    )

    family_b = merge_family_candidates(
        family_b_candidates,
        "B",
        (
            dominant_deg + 90.0
        ) % 180.0,
        args,
    )

    debug["family_a_candidate_count"] = len(
        family_a_candidates
    )
    debug["family_b_candidate_count"] = len(
        family_b_candidates
    )

    pre_inference_a = len(family_a)
    pre_inference_b = len(family_b)

    family_a, family_b = add_geometric_inference(
        family_a,
        family_b,
        args,
    )

    debug["family_a_pre_inference_count"] = pre_inference_a
    debug["family_b_pre_inference_count"] = pre_inference_b
    debug["family_a_vector_count"] = len(family_a)
    debug["family_b_vector_count"] = len(family_b)

    return family_a, family_b, debug


# ---------------------------------------------------------------------------
# 표시 / 저장
# ---------------------------------------------------------------------------

def draw_metric_grid(
    image: np.ndarray,
    args: argparse.Namespace,
) -> np.ndarray:
    out = image.copy()

    mpp = args.meters_per_pixel

    # 1 m grid
    x_values = np.arange(
        np.ceil(args.x_min_m),
        np.floor(args.x_max_m) + 1.0,
        1.0,
    )

    y_values = np.arange(
        np.ceil(args.y_min_m),
        np.floor(args.y_max_m) + 1.0,
        1.0,
    )

    for x_m in x_values:
        px = int(
            round(
                (
                    x_m
                    - args.x_min_m
                )
                / mpp
            )
        )

        cv2.line(
            out,
            (px, 0),
            (px, out.shape[0] - 1),
            (70, 70, 70),
            1,
            cv2.LINE_AA,
        )

    for y_m in y_values:
        py = int(
            round(
                (
                    args.y_max_m
                    - y_m
                )
                / mpp
            )
        )

        cv2.line(
            out,
            (0, py),
            (out.shape[1] - 1, py),
            (70, 70, 70),
            1,
            cv2.LINE_AA,
        )

    return out


def draw_vectors(
    bev: np.ndarray,
    vertical: list[VectorTrack],
    horizontal: list[VectorTrack],
    args: argparse.Namespace,
) -> np.ndarray:
    overlay = draw_metric_grid(
        bev,
        args,
    )

    for i, track in enumerate(vertical):
        color = (
            (0, 255, 0)
            if track.temporal_confirmed
            else (0, 165, 255)
        )
        pts = np.round(
            track.polyline
        ).astype(np.int32).reshape(-1, 1, 2)

        cv2.polylines(
            overlay,
            [pts],
            False,
            color,
            args.vector_thickness,
            cv2.LINE_AA,
        )

        anchor = tuple(
            pts[
                len(pts) // 2,
                0,
            ]
        )

        cv2.putText(
            overlay,
            (
                f"A#{track.temporal_track_id} {track.span_m:.1f}m "
                f"H={track.temporal_hits_in_window} "
                f"{'C' if track.temporal_confirmed else 'T'}"
                f"{'R' if track.temporal_reprojected else ''}"
            ),
            anchor,
            cv2.FONT_HERSHEY_SIMPLEX,
            0.45,
            color,
            1,
            cv2.LINE_AA,
        )

    for i, track in enumerate(horizontal):
        color = (
            (255, 255, 0)
            if track.temporal_confirmed
            else (0, 165, 255)
        )
        pts = np.round(
            track.polyline
        ).astype(np.int32).reshape(-1, 1, 2)

        cv2.polylines(
            overlay,
            [pts],
            False,
            color,
            args.vector_thickness,
            cv2.LINE_AA,
        )

        anchor = tuple(
            pts[
                len(pts) // 2,
                0,
            ]
        )

        cv2.putText(
            overlay,
            (
                f"B#{track.temporal_track_id} {track.span_m:.1f}m "
                f"H={track.temporal_hits_in_window} "
                f"{'C' if track.temporal_confirmed else 'T'}"
                f"{'R' if track.temporal_reprojected else ''}"
            ),
            anchor,
            cv2.FONT_HERSHEY_SIMPLEX,
            0.45,
            color,
            1,
            cv2.LINE_AA,
        )

    return overlay


def vector_to_dict(
    track: VectorTrack,
) -> dict:
    return {
        "orientation": track.orientation,
        "span_px": track.span_px,
        "span_m": track.span_m,
        "mean_width_px": track.mean_width_px,
        "mean_width_m": track.mean_width_m,
        "density": track.density,
        "fit_rmse_m": track.fit_rmse_m,
        "axis_slope": track.axis_slope,
        "representative_m": track.representative_m,
        "angle_deg": track.angle_deg,
        "quality_score": track.quality_score,
        "binary_support": track.binary_support,
        "color_support": track.color_support,
        "response_score": track.response_score,
        "glare_ratio": track.glare_ratio,
        "dominant_error_deg": track.dominant_error_deg,
        "family": track.family,
        "appearance_score": track.appearance_score,
        "parallel_support": track.parallel_support,
        "orthogonal_support": track.orthogonal_support,
        "inference_score": track.inference_score,
        "temporal": {
            "track_id": track.temporal_track_id,
            "hits_total": track.temporal_hits_total,
            "hits_in_window": track.temporal_hits_in_window,
            "confirmed": track.temporal_confirmed,
            "reprojected": track.temporal_reprojected,
            "source_frame_id": track.source_frame_id,
            "source_timestamp": track.source_timestamp,
            "source_pose": {
                "x": track.source_pose_x,
                "y": track.source_pose_y,
                "yaw": track.source_pose_yaw,
            },
        },
        "pixel_polyline": (
            track.polyline
            .astype(float)
            .tolist()
        ),
        "metric_polyline_xy_m": (
            track.metric_polyline
            .astype(float)
            .tolist()
        ),
    }


def save_capture(
    raw: np.ndarray,
    bev: np.ndarray,
    prep: dict,
    overlay: np.ndarray,
    vertical: list[VectorTrack],
    horizontal: list[VectorTrack],
    temporal_tracker: TemporalVectorTracker,
    current_frame_id: int,
    vectorized_this_frame: bool,
    args: argparse.Namespace,
) -> None:
    args.save_dir.mkdir(
        parents=True,
        exist_ok=True,
    )

    stamp = datetime.now().strftime(
        "%Y%m%d_%H%M%S"
    )

    hsv_vis = cv2.cvtColor(
        prep["hsv"],
        cv2.COLOR_HSV2BGR,
    )

    images = {
        "raw": raw,
        "bev": bev,
        "hsv_reconstructed": hsv_vis,
        "white_mask": prep["white_mask"],
        "yellow_mask": prep["yellow_mask"],
        "lane_color_mask": prep["lane_color_mask"],
        "glare_mask": prep["glare_mask"],
        "normalized_v": prep["normalized"],
        "tophat": prep["tophat"],
        "binary_raw": prep["binary_raw"],
        "binary_hsv": prep["binary_fused"],
        "vectors": overlay,
    }

    for name, image in images.items():
        cv2.imwrite(
            str(
                args.save_dir
                / f"{stamp}_{name}.png"
            ),
            image,
        )

    metadata = {
        "camera_pose": {
            "height_m": args.camera_height_m,
            "pitch_deg": args.pitch_deg,
            "roll_deg": args.roll_deg,
            "yaw_deg": args.yaw_deg,
        },
        "bev": {
            "x_range_m": [
                args.x_min_m,
                args.x_max_m,
            ],
            "y_range_m": [
                args.y_min_m,
                args.y_max_m,
            ],
            "meters_per_pixel": args.meters_per_pixel,
            "width_px": int(bev.shape[1]),
            "height_px": int(bev.shape[0]),
        },
        "preprocess": {
            "source_frame_id": int(prep.get("source_frame_id", -1)),
            "source_timestamp": float(prep.get("source_timestamp", 0.0)),
            "white_s_max": args.white_s_max,
            "white_v_min": args.white_v_min,
            "yellow_h_min": args.yellow_h_min,
            "yellow_h_max": args.yellow_h_max,
            "yellow_s_min": args.yellow_s_min,
            "yellow_v_min": args.yellow_v_min,
            "glare_s_max": args.glare_s_max,
            "glare_v_min": args.glare_v_min,
            "tophat_size_m": args.tophat_size_m,
            "tophat_size_px": prep["tophat_size_px"],
            "threshold_mean": prep["threshold_mean"],
            "threshold_bands": prep["thresholds"],
            "background_median_mean": prep["background_median_mean"],
            "background_mad_mean": prep["background_mad_mean"],
            "strict_white": bool(args.strict_white),
        },
        "vector_mode": args.vector_mode,
        "temporal_vectorization": {
            "current_frame_id": int(current_frame_id),
            "vectorized_this_frame": bool(vectorized_this_frame),
            "vectorize_every_n": int(args.vectorize_every_n),
            "history_size": int(args.temporal_history_size),
            "confirm_hits": int(args.temporal_confirm_hits),
            "active_primitives": temporal_tracker.to_dict(),
        },
        "family_a_vectors": [
            vector_to_dict(track)
            for track in vertical
        ],
        "family_b_vectors": [
            vector_to_dict(track)
            for track in horizontal
        ],
        # legacy aliases
        "vertical_vectors": [
            vector_to_dict(track)
            for track in vertical
        ],
        "horizontal_vectors": [
            vector_to_dict(track)
            for track in horizontal
        ],
    }

    (
        args.save_dir
        / f"{stamp}_vectors.json"
    ).write_text(
        json.dumps(
            metadata,
            ensure_ascii=False,
            indent=2,
        ),
        encoding="utf-8",
    )

    print(
        f"[SAVE] {args.save_dir.resolve()} / {stamp}_*"
    )


# ---------------------------------------------------------------------------
# 메인
# ---------------------------------------------------------------------------

def main() -> int:
    args = parse_args()

    if args.meters_per_pixel <= 0:
        raise ValueError("meters-per-pixel은 양수여야 합니다.")

    if args.x_max_m <= args.x_min_m:
        raise ValueError("x-max-m은 x-min-m보다 커야 합니다.")

    if args.y_max_m <= args.y_min_m:
        raise ValueError("y-max-m은 y-min-m보다 커야 합니다.")

    if args.map_resolution_m <= 0:
        raise ValueError("map-resolution-m은 양수여야 합니다.")

    cv2.setUseOptimized(True)

    if not args.image_topic and not args.no_camera_controls:
        set_camera_controls(args.device)

    K, D, calibration_size = load_calibration(
        args.calib
    )

    frame_size = (
        args.width,
        args.height,
    )

    if frame_size != calibration_size:
        K_used = scale_calibration(
            K,
            calibration_size,
            frame_size,
        )
    else:
        K_used = K.copy()

    bev_map_build_t0 = time.perf_counter()

    map_x, map_y, valid_mask = make_fisheye_bev_map(
        image_size=frame_size,
        camera_matrix=K_used,
        distortion=D,
        camera_height_m=args.camera_height_m,
        pitch_deg=args.pitch_deg,
        roll_deg=args.roll_deg,
        yaw_deg=args.yaw_deg,
        x_range_m=(
            args.x_min_m,
            args.x_max_m,
        ),
        y_range_m=(
            args.y_min_m,
            args.y_max_m,
        ),
        meters_per_pixel=args.meters_per_pixel,
    )

    bev_map_build_ms = (
        time.perf_counter()
        - bev_map_build_t0
    ) * 1000.0

    print(
        f"[PROFILE STARTUP] metric BEV map 생성: "
        f"{bev_map_build_ms:.2f} ms (시작 시 1회)"
    )

    invalid_mask = valid_mask == 0

    process: subprocess.Popen | None = None
    ros_source: RosFrameAndPoseSource | None = None
    if args.image_topic:
        ros_source = RosFrameAndPoseSource(args)
        reader = ros_source
    else:
        process = subprocess.Popen(
            build_gst_cmd(args),
            stdout=subprocess.PIPE,
            stderr=None,
            bufsize=0,
        )
        if process.stdout is None:
            raise RuntimeError("GStreamer stdout 생성 실패")
        reader = LatestFrameReader(process, args.width, args.height)

    threshold_smoother = TemporalThresholdSmoother(
        args.threshold_lpf_alpha
    )

    profiler = TimingProfiler(
        args.profile_window
    )
    processed_frame_count = 0

    pose_provider = TemporaryPoseProvider(
        args
    )
    global_mapper = GlobalMarkingMapper(
        args
    )
    temporal_tracker = TemporalVectorTracker(
        args
    )
    last_vector_debug: dict = {
        "raw_candidate_count": 0,
        "dominant_angle_deg": None,
        "dominant_confidence": 0.0,
        "family_a_pre_inference_count": 0,
        "family_b_pre_inference_count": 0,
    }
    last_prep: dict | None = None
    last_local_vertical: list[VectorTrack] = []
    last_local_horizontal: list[VectorTrack] = []

    show_ui = bool(
        args.show_ui
        or args.show_debug
    )
    show_map = bool(
        args.show_map
    )
    show_video = bool(
        args.show_video
        or args.show_ui
        or args.show_debug
    )
    debug = bool(args.show_debug)

    print("=== Metric BEV + Temporal Vector Tracking v5.2 / source-aware 3-of-5 ===")
    print(
        f"BEV: X={args.x_min_m:.1f}~{args.x_max_m:.1f}m, "
        f"Y={args.y_min_m:.1f}~{args.y_max_m:.1f}m, "
        f"{args.meters_per_pixel:.3f}m/px"
    )
    print(
        "HSV gate:",
        "STRICT" if args.strict_white else "SOFT",
    )
    print(
        "Threshold:",
        f"update={args.threshold_update_interval} frames, "
        f"sample_stride={args.threshold_sample_stride}",
    )
    print(
        "Glare:",
        (
            "HARD MASK"
            if args.hard_glare_mask
            else f"RECOVERY margin={args.glare_recovery_margin:.1f}"
        ),
    )
    print(
        "Video UI:",
        "ON" if show_video else "OFF",
    )
    print(
        "Global map:",
        f"{args.map_x_min_m:.1f}~{args.map_x_max_m:.1f}m forward, "
        f"{args.map_y_min_m:.1f}~{args.map_y_max_m:.1f}m left, "
        f"res={args.map_resolution_m:.3f}m/cell",
    )
    print(
        "Pose source:",
        (
            f"ROS VIO ({args.odom_topic})"
            if args.image_topic
            else (
                "STATIC temporary pose"
                if args.pose_mode == "static"
                else (
                    f"CONSTANT temporary pose "
                    f"vx={args.temp_vx_mps:.2f}m/s "
                    f"vy={args.temp_vy_mps:.2f}m/s "
                    f"yaw_rate={args.temp_yaw_rate_degps:.2f}deg/s"
                )
            )
        ),
    )
    print(
        "Map accumulation:",
        (
            "TEST EVERY FRAME (실제 이동 지도에는 사용 금지)"
            if args.map_test_accumulate_every_frame
            else (
                f"POSE GATED: translation>={args.map_min_translation_m:.2f}m "
                f"or yaw>={args.map_min_yaw_deg:.1f}deg"
            )
        ),
    )
    print(
        "Vectorizer:",
        args.vector_mode,
    )
    if args.vector_mode == "dominant_hough":
        print(
            "  arbitrary angle / no absolute axis limit | "
            f"dominant tol={args.dominant_angle_tolerance_deg:.1f}deg | "
            f"width={args.line_min_width_m:.2f}~{args.line_max_width_m:.2f}m | "
            f"appearance>={args.final_min_score:.2f}"
        )
        print(
            "  inference: "
            f"parallel={args.parallel_spacing_min_m:.2f}~{args.parallel_spacing_max_m:.2f}m | "
            f"orthogonal_near<={args.orthogonal_near_distance_m:.2f}m | "
            f"final>={args.inference_min_score:.2f}"
        )
    else:
        print(
            "  legacy scan | "
            f"axis_slope<={args.max_axis_slope:.2f}"
        )
    print(
        "Vector filter:",
        f"span V>={args.vertical_min_span_m:.2f}m, "
        f"H>={args.horizontal_min_span_m:.2f}m, "
        f"density>={args.min_track_density:.2f}, "
        f"RMSE<={args.max_fit_rmse_m:.3f}m, "
        f"max_each={args.max_vectors_each}",
    )
    print(
        "Temporal vectors:",
        f"Hough every {max(1, args.vectorize_every_n)} frame(s) | "
        f"confirm={args.temporal_confirm_hits}/{args.temporal_history_size} | "
        f"angle<={args.temporal_angle_gate_deg:.1f}deg | "
        f"offset<={args.temporal_offset_gate_m:.2f}m | "
        f"gap<={args.temporal_gap_gate_m:.2f}m",
    )
    if show_video or show_map:
        print(
            "keys: Q/Esc quit | S save | D debug | "
            "I/K forward/back | J/L left/right | U/O yaw"
        )

    if show_video:
        cv2.namedWindow(
            "Raw Camera",
            cv2.WINDOW_NORMAL,
        )

        cv2.namedWindow(
            "Metric BEV Vectors",
            cv2.WINDOW_NORMAL,
        )

    if show_ui:
        cv2.namedWindow(
            "HSV Binary",
            cv2.WINDOW_NORMAL,
        )

    if show_map:
        cv2.namedWindow(
            "Global Marking Map",
            cv2.WINDOW_NORMAL,
        )

    last_time = time.perf_counter()
    fps_value = 0.0

    try:
        while True:
            received = reader.get_latest()
            if ros_source is not None:
                frame, frame_arrival_time, input_period_ms, ros_stamp = received
            else:
                frame, frame_arrival_time, input_period_ms = received
                ros_stamp = None

            if frame is None:
                if (
                    not reader.running
                    or (process is not None and process.poll() is not None)
                ):
                    print("카메라 프레임 종료")
                    break

                time.sleep(0.001)
                continue

            frame_processing_t0 = time.perf_counter()

            if input_period_ms is not None:
                profiler.add(
                    "input_period",
                    input_period_ms,
                )

            if frame_arrival_time is not None:
                profiler.add(
                    "queue_age",
                    (
                        frame_processing_t0
                        - frame_arrival_time
                    ) * 1000.0,
                )

            t0 = time.perf_counter()

            bev = cv2.remap(
                frame,
                map_x,
                map_y,
                interpolation=cv2.INTER_LINEAR,
                borderMode=cv2.BORDER_CONSTANT,
                borderValue=(0, 0, 0),
            )

            bev[invalid_mask] = 0

            t_bev_done = time.perf_counter()
            profiler.add(
                "bev_remap",
                (t_bev_done - t0) * 1000.0,
            )

            source_timestamp = (
                float(ros_stamp)
                if ros_stamp is not None
                else (
                    float(frame_arrival_time)
                    if frame_arrival_time is not None
                    else float(frame_processing_t0)
                )
            )
            if ros_source is not None:
                pose = ros_source.pose_at(source_timestamp)
            else:
                pose = pose_provider.update(source_timestamp)
            current_frame_id = int(processed_frame_count)
            vectorized_this_frame = (
                current_frame_id % max(1, int(args.vectorize_every_n)) == 0
            )

            # 전처리 결과는 Hough가 쓰는 입력이므로 같은 주기로 계산한다.
            # 중간 카메라 프레임은 최신 BEV에 전역 primitive만 재투영한다.
            if vectorized_this_frame or last_prep is None:
                prep = preprocess_bev(
                    bev,
                    valid_mask,
                    threshold_smoother,
                    args,
                )
                prep["source_frame_id"] = current_frame_id
                prep["source_timestamp"] = source_timestamp
                last_prep = prep
                for name, value in prep["profile_ms"].items():
                    profiler.add(name, value)
                profiler.add(
                    "preprocess_active",
                    float(prep["profile_ms"].get("preprocess_total", 0.0)),
                )
            else:
                prep = last_prep
                for name in prep["profile_ms"]:
                    profiler.add(name, 0.0)

            binary = prep["binary_fused"]

            t_vector_v0 = time.perf_counter()

            raw_vertical: list[VectorTrack] = []
            raw_horizontal: list[VectorTrack] = []

            if vectorized_this_frame:
                if args.vector_mode == "dominant_hough":
                    (
                        raw_vertical,
                        raw_horizontal,
                        vector_debug,
                    ) = dominant_hough_vectorize(
                        binary,
                        prep,
                        args,
                    )
                else:
                    raw_vertical = track_scanlines(binary, "vertical", args)
                    raw_horizontal = track_scanlines(binary, "horizontal", args)
                    vector_debug = {
                        "raw_candidate_count": len(raw_vertical) + len(raw_horizontal),
                        "dominant_angle_deg": None,
                        "dominant_confidence": 0.0,
                        "family_a_vector_count": len(raw_vertical),
                        "family_b_vector_count": len(raw_horizontal),
                        "family_a_pre_inference_count": len(raw_vertical),
                        "family_b_pre_inference_count": len(raw_horizontal),
                    }

                t_vector_v1 = time.perf_counter()
                vector_active_ms = (t_vector_v1 - t_vector_v0) * 1000.0
                last_vector_debug = dict(vector_debug)
                last_local_vertical = raw_vertical
                last_local_horizontal = raw_horizontal
                if pose is not None:
                    temporal_tracker.update(
                        [*raw_vertical, *raw_horizontal],
                        pose=pose,
                        timestamp=source_timestamp,
                        frame_id=current_frame_id,
                    )
                profiler.add("vector_active", vector_active_ms)
            else:
                t_vector_v1 = time.perf_counter()
                vector_active_ms = 0.0
                vector_debug = dict(last_vector_debug)
                vector_debug["scheduled_skip"] = True

            if args.vector_mode == "dominant_hough":
                profiler.add("vector_dominant_hough", vector_active_ms)
            else:
                profiler.add("vector_legacy_scan", vector_active_ms)

            t_temporal0 = time.perf_counter()
            if pose is not None:
                vertical, horizontal = temporal_tracker.project(
                    pose,
                    args,
                    current_frame_id=current_frame_id,
                    confirmed_only=False,
                )
                confirmed_vertical, confirmed_horizontal = temporal_tracker.project(
                    pose,
                    args,
                    current_frame_id=current_frame_id,
                    confirmed_only=True,
                )
                if ros_source is not None:
                    ros_source.publish_lines(
                        [*confirmed_vertical, *confirmed_horizontal],
                        source_timestamp,
                    )
            else:
                vertical = last_local_vertical
                horizontal = last_local_horizontal
                confirmed_vertical = []
                confirmed_horizontal = []
            t_temporal1 = time.perf_counter()
            profiler.add(
                "temporal_tracking",
                (t_temporal1 - t_temporal0) * 1000.0,
            )

            # compatibility profiler slots
            profiler.add(
                "vector_vertical",
                0.0,
            )
            profiler.add(
                "vector_horizontal",
                0.0,
            )

            t_mapping0 = time.perf_counter()

            if pose is not None and vectorized_this_frame and (
                confirmed_vertical or confirmed_horizontal
            ):
                map_accumulated = global_mapper.update(
                    confirmed_vertical,
                    confirmed_horizontal,
                    pose,
                )
            else:
                map_accumulated = False

            t_mapping1 = time.perf_counter()
            profiler.add(
                "global_mapping",
                (t_mapping1 - t_mapping0) * 1000.0,
            )

            t_draw0 = time.perf_counter()

            overlay = None
            key = 255

            if show_video or ros_source is not None:
                temporal_stats = temporal_tracker.stats()
                overlay = draw_vectors(
                    bev,
                    vertical,
                    horizontal,
                    args,
                )

                now = time.perf_counter()
                dt = now - last_time
                last_time = now

                if dt > 1e-6:
                    instant = 1.0 / dt
                    fps_value = (
                        instant
                        if fps_value == 0.0
                        else 0.9 * fps_value
                        + 0.1 * instant
                    )

                cv2.rectangle(
                    overlay,
                    (0, 0),
                    (overlay.shape[1], 58),
                    (0, 0, 0),
                    -1,
                )

                cv2.putText(
                    overlay,
                    (
                        f"FPS={fps_value:.1f}  "
                        f"T={prep['threshold_mean']:.1f}  "
                        f"TRK={temporal_stats['active']} "
                        f"C={temporal_stats['confirmed']}"
                    ),
                    (12, 25),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.62,
                    (0, 255, 255),
                    2,
                    cv2.LINE_AA,
                )

                cv2.putText(
                    overlay,
                    (
                        f"BEV {args.meters_per_pixel:.3f}m/px  "
                        f"HSV={'strict' if args.strict_white else 'soft'}  "
                        f"H={'RUN' if vectorized_this_frame else 'REPROJECT'}"
                    ),
                    (12, 49),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.52,
                    (220, 220, 220),
                    1,
                    cv2.LINE_AA,
                )

                if (
                    args.vector_mode == "dominant_hough"
                    and vector_debug.get(
                        "dominant_angle_deg"
                    ) is not None
                ):
                    cv2.putText(
                        overlay,
                        (
                            f"DOM={vector_debug['dominant_angle_deg']:.1f}deg "
                            f"conf={vector_debug['dominant_confidence']:.2f} "
                            f"raw={vector_debug['raw_candidate_count']}"
                        ),
                        (300, 49),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.46,
                        (220, 220, 220),
                        1,
                        cv2.LINE_AA,
                    )

                if ros_source is not None:
                    goal_overlay = bev.copy()
                    for selected_line in ros_source.selected_lines_in_base():
                        selected_bev = base_to_bev_metric_xy(selected_line)
                        selected_pixels = np.round(
                            bev_metric_to_pixel_xy(selected_bev, args)
                        ).astype(np.int32)
                        cv2.line(
                            overlay,
                            tuple(selected_pixels[0]),
                            tuple(selected_pixels[-1]),
                            (255, 0, 255),
                            6,
                            cv2.LINE_AA,
                        )

                    goal = (
                        ros_source.goal_in_base(pose)
                        if pose is not None else None
                    )
                    if goal is not None:
                        goal_xy, goal_yaw = goal
                        arrow_end = goal_xy + 0.8 * np.asarray(
                            [math.cos(goal_yaw), math.sin(goal_yaw)]
                        )
                        goal_bev = base_to_bev_metric_xy(
                            np.vstack([goal_xy, arrow_end])
                        )
                        goal_pixels = np.round(
                            bev_metric_to_pixel_xy(goal_bev, args)
                        ).astype(np.int32)
                        start_pixel = tuple(goal_pixels[0])
                        end_pixel = tuple(goal_pixels[1])
                        for target in (overlay, goal_overlay):
                            cv2.circle(
                                target, start_pixel, 8, (255, 0, 255), -1
                            )
                            cv2.arrowedLine(
                                target,
                                start_pixel,
                                end_pixel,
                                (0, 255, 0),
                                4,
                                cv2.LINE_AA,
                                tipLength=0.3,
                            )
                            cv2.putText(
                                target,
                                "GOAL",
                                (start_pixel[0] + 10, start_pixel[1] - 10),
                                cv2.FONT_HERSHEY_SIMPLEX,
                                0.60,
                                (255, 0, 255),
                                2,
                                cv2.LINE_AA,
                            )

                    ros_source.publish_debug_bev(overlay, source_timestamp)
                    ros_source.publish_debug_goal_bev(
                        goal_overlay, source_timestamp
                    )

            t_draw1 = time.perf_counter()
            profiler.add(
                "draw",
                (t_draw1 - t_draw0) * 1000.0,
            )

            t_ui0 = time.perf_counter()

            if show_video:
                assert overlay is not None

                # 2560x1440 원본은 화면을 너무 크게 차지하므로
                # 표시용으로만 축소한다. 실제 처리 해상도에는 영향 없음.
                raw_preview = cv2.resize(
                    frame,
                    (1280, 720),
                    interpolation=cv2.INTER_AREA,
                )

                cv2.imshow(
                    "Raw Camera",
                    raw_preview,
                )
                cv2.imshow(
                    "Metric BEV Vectors",
                    overlay,
                )

            if show_ui:
                cv2.imshow(
                    "HSV Binary",
                    binary,
                )

                if debug:
                    cv2.imshow(
                        "HSV White Mask",
                        prep["white_mask"],
                    )
                    cv2.imshow(
                        "HSV Yellow Mask",
                        prep["yellow_mask"],
                    )
                    cv2.imshow(
                        "Lane Color Mask",
                        prep["lane_color_mask"],
                    )
                    cv2.imshow(
                        "HSV Glare Mask",
                        prep["glare_mask"],
                    )
                    cv2.imshow(
                        "Normalized V",
                        prep["normalized"],
                    )
                    cv2.imshow(
                        "White TopHat",
                        prep["tophat"],
                    )
                    cv2.imshow(
                        "Binary RAW before HSV",
                        prep["binary_raw"],
                    )

            if show_map and pose is not None:
                map_vis = global_mapper.render(
                    pose
                )

                cv2.imshow(
                    "Global Marking Map",
                    map_vis,
                )

            if show_video or show_map:
                key = cv2.waitKey(1) & 0xFF

            t_ui1 = time.perf_counter()

            profiler.add(
                "ui",
                (t_ui1 - t_ui0) * 1000.0,
            )

            total_processing_ms = (
                t_ui1
                - frame_processing_t0
            ) * 1000.0

            profiler.add(
                "total_processing",
                total_processing_ms,
            )

            processed_frame_count += 1

            if (
                args.profile_every > 0
                and processed_frame_count
                % args.profile_every == 0
            ):
                input_avg = profiler.avg(
                    "input_period"
                )
                input_fps = (
                    1000.0 / input_avg
                    if input_avg > 1e-6
                    else 0.0
                )

                process_avg = profiler.avg(
                    "total_processing"
                )
                process_capacity_fps = (
                    1000.0 / process_avg
                    if process_avg > 1e-6
                    else 0.0
                )

                print()
                print(
                    f"[PROFILE {processed_frame_count}] "
                    f"입력={input_avg:.2f}ms "
                    f"({input_fps:.1f}fps), "
                    f"처리평균={process_avg:.2f}ms "
                    f"(이론 {process_capacity_fps:.1f}fps), "
                    f"P95={profiler.p95('total_processing'):.2f}ms"
                )

                print(
                    "  [A] "
                    + profiler.summary_line(
                        [
                            "bev_remap",
                            "preprocess_total",
                            "preprocess_active",
                            "vector_dominant_hough",
                            "vector_active",
                            "temporal_tracking",
                            "vector_legacy_scan",
                            "global_mapping",
                            "draw",
                            "ui",
                        ]
                    )
                )

                print(
                    "  [B preprocess] "
                    + profiler.summary_line(
                        [
                            "hsv_white",
                            "glare",
                            "illumination",
                            "pre_blur",
                            "tophat",
                            "threshold_stats",
                            "threshold_binary",
                            "hsv_fuse_morph",
                        ]
                    )
                )

                print(
                    f"  queue_age={profiler.avg('queue_age'):.2f}ms | "
                    f"30FPS 예산=33.33ms | "
                    f"여유={33.33 - process_avg:+.2f}ms"
                )

                pose_text = "NONE" if pose is None else (
                    f"({pose.x:.2f}, {pose.y:.2f}, {math.degrees(pose.yaw):.1f}deg)"
                )
                print(
                    f"  map_updates={global_mapper.update_count} | "
                    f"pose={pose_text} | "
                    f"last_accumulate={'Y' if map_accumulated else 'N'}"
                )

                temporal_stats = temporal_tracker.stats()
                print(
                    "  [TEMPORAL] "
                    f"active={temporal_stats['active']} | "
                    f"confirmed={temporal_stats['confirmed']} | "
                    f"tentative={temporal_stats['tentative']} | "
                    f"updates={temporal_stats['observation_updates']} | "
                    f"schedule=1/{max(1, args.vectorize_every_n)}"
                )

                if args.vector_mode == "dominant_hough":
                    print(
                        "  [VECTOR] "
                        f"raw={vector_debug.get('raw_candidate_count', 0)} | "
                        f"dominant={vector_debug.get('dominant_angle_deg')} | "
                        f"conf={vector_debug.get('dominant_confidence', 0.0):.2f} | "
                        f"A={len(vertical)} B={len(horizontal)} "
                        f"(pre={vector_debug.get('family_a_pre_inference_count', 0)}/"
                        f"{vector_debug.get('family_b_pre_inference_count', 0)})"
                    )

            if key in (
                27,
                ord("q"),
                ord("Q"),
            ):
                break

            if key in (
                ord("s"),
                ord("S"),
            ):
                save_capture(
                    raw=frame,
                    bev=bev,
                    prep=prep,
                    overlay=(
                        overlay
                        if overlay is not None
                        else bev
                    ),
                    vertical=vertical,
                    horizontal=horizontal,
                    temporal_tracker=temporal_tracker,
                    current_frame_id=current_frame_id,
                    vectorized_this_frame=vectorized_this_frame,
                    args=args,
                )

                if pose is not None:
                    global_mapper.save(
                        args.save_dir,
                        pose,
                    )

            if show_video or show_map:
                step_m = float(
                    args.pose_key_step_m
                )
                step_yaw = math.radians(
                    float(args.pose_key_step_deg)
                )

                if key in (ord("i"), ord("I")):
                    pose_provider.nudge(
                        dx_body=step_m
                    )
                elif key in (ord("k"), ord("K")):
                    pose_provider.nudge(
                        dx_body=-step_m
                    )
                elif key in (ord("j"), ord("J")):
                    pose_provider.nudge(
                        dy_body=step_m
                    )
                elif key in (ord("l"), ord("L")):
                    pose_provider.nudge(
                        dy_body=-step_m
                    )
                elif key in (ord("u"), ord("U")):
                    pose_provider.nudge(
                        dyaw_rad=step_yaw
                    )
                elif key in (ord("o"), ord("O")):
                    pose_provider.nudge(
                        dyaw_rad=-step_yaw
                    )

            if show_ui and key in (
                ord("d"),
                ord("D"),
            ):
                debug = not debug

                if not debug:
                    for name in (
                        "HSV White Mask",
                        "HSV Glare Mask",
                        "Normalized V",
                        "White TopHat",
                        "Binary RAW before HSV",
                    ):
                        try:
                            cv2.destroyWindow(name)
                        except cv2.error:
                            pass

    except KeyboardInterrupt:
        pass

    finally:
        if process is not None and process.poll() is None:
            process.terminate()

            try:
                process.wait(timeout=2)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait()

        if ros_source is not None:
            ros_source.stop()
        else:
            reader.stop()

        if process is not None and process.stdout is not None:
            process.stdout.close()

        cv2.destroyAllWindows()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
