#!/usr/bin/env python3
"""CARLA 없이 HUD 를 몰아 보는 임시 조작 패널. CAN 계층만 흉내낸다.

GPU 렌더링이 죽은 기계에서(드라이버 커널 모듈과 라이브러리가 어긋나면 UE4 가 뜨지
못한다) HUD 의 코리도가 조향·기어를 따라오는지 보여주려고 만들었다. **시뮬레이터가
아니다** — 자전거 모델도, 지면도, 인지도 없다. 키 입력을 CAN 신호로 바꿀 뿐이다.

    python3 keyboard_can.py            # 차량 상태만 (HUD 는 Basic = 줄무늬)
    python3 keyboard_can.py --scene    # 인지가 살아 있고 아무것도 못 봤다고도 발행

--scene 은 빈 장면이다. 장애물을 지어내지 않는다 — "인지는 살아 있는데 본 것이
없다"는 상태이고, 그래야 HUD 가 Normal 어법(실선 코리도·거리 눈금)으로 그린다.

조작: ← → 조향, ↑ ↓ 속도, 스페이스 정지, F 전진 R 후진 P 주차 N 중립, ESC 종료.
"""

import argparse
import math
import os
import time

os.environ.setdefault('PYGAME_HIDE_SUPPORT_PROMPT', '1')

from hero_hud.msg import SceneOverlay, SourceHealth, VehicleState  # noqa: E402
import pygame  # noqa: E402
import rclpy  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.qos import QoSProfile, ReliabilityPolicy  # noqa: E402

UNKNOWN, PARK, REVERSE, NEUTRAL, DRIVE = 0, 1, 2, 3, 4
GEAR_NAME = {UNKNOWN: '?', PARK: 'P', REVERSE: 'R', NEUTRAL: 'N', DRIVE: 'D'}
GEAR_KEYS = {pygame.K_p: PARK, pygame.K_r: REVERSE, pygame.K_n: NEUTRAL, pygame.K_f: DRIVE}

# CARLA Prius 실측(get_physics_control): 자전거 등가 최대 조향각.
MAX_STEER_RAD = 0.9925
STEER_RATE = 1.2      # rad/s, 핸들을 돌리는 속도
MAX_SPEED = 8.0       # m/s
ACCEL = 1.5           # m/s^2


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--scene', action='store_true',
                        help='빈 장면도 발행한다 (인지 살아 있음, 본 것 없음)')
    parser.add_argument('--rate', type=float, default=50.0)
    args = parser.parse_args()

    rclpy.init()
    node = Node('keyboard_can')
    qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
    vehicle_pub = node.create_publisher(VehicleState, '/hero/vehicle_state', qos)
    scene_pub = (node.create_publisher(SceneOverlay, '/hero/scene_overlay', qos)
                 if args.scene else None)

    pygame.init()
    screen = pygame.display.set_mode((900, 420))
    pygame.display.set_caption('HERO keyboard CAN (no CARLA)')
    font = pygame.font.SysFont('monospace', 22)
    big = pygame.font.SysFont('monospace', 96, bold=True)

    gear = DRIVE
    steer = 0.0
    speed = 0.0
    target = 0.0
    last = time.monotonic()
    period = 1.0 / max(args.rate, 1.0)

    running = True
    while running and rclpy.ok():
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    running = False
                elif event.key in GEAR_KEYS:
                    gear = GEAR_KEYS[event.key]
                    if gear in (PARK, NEUTRAL):
                        target = 0.0
                elif event.key == pygame.K_SPACE:
                    target = 0.0

        now = time.monotonic()
        dt = max(1e-3, now - last)
        last = now

        keys = pygame.key.get_pressed()
        if keys[pygame.K_LEFT]:
            steer = min(MAX_STEER_RAD, steer + STEER_RATE * dt)
        elif keys[pygame.K_RIGHT]:
            steer = max(-MAX_STEER_RAD, steer - STEER_RATE * dt)
        else:
            # 손을 놓으면 핸들이 돌아온다.
            steer -= math.copysign(min(abs(steer), STEER_RATE * dt * 0.8), steer)
        if keys[pygame.K_UP]:
            target = min(MAX_SPEED, target + 2.0 * dt)
        if keys[pygame.K_DOWN]:
            target = max(0.0, target - 3.0 * dt)
        if gear in (PARK, NEUTRAL):
            target = 0.0

        speed += math.copysign(min(abs(target - speed), ACCEL * dt), target - speed)
        signed = -speed if gear == REVERSE else (speed if gear == DRIVE else 0.0)

        message = VehicleState()
        message.header.stamp = node.get_clock().now().to_msg()
        message.gear = gear
        message.speed_mps = float(signed)
        message.steering_angle_rad = float(steer)
        # 자전거 모델의 요레이트. 표시에는 안 쓰이지만 계약을 비워 두지 않는다.
        message.yaw_rate_rps = float(signed * math.tan(steer) / 2.819)
        vehicle_pub.publish(message)

        if scene_pub is not None:
            scene = SceneOverlay()
            scene.header.stamp = message.header.stamp
            scene.header.frame_id = 'base_link'
            scene.advised_gear = UNKNOWN
            for name in ('odometry', 'occupancy'):
                health = SourceHealth()
                health.name = name
                health.age_sec = 0.02
                health.ok = True
                scene.health.append(health)
            scene_pub.publish(scene)

        screen.fill((16, 16, 18))
        screen.blit(big.render(GEAR_NAME[gear], True, (230, 230, 120)), (40, 30))
        lines = [
            f'속도   {signed:+6.2f} m/s   (목표 {target:4.1f})',
            f'조향   {math.degrees(steer):+6.1f} deg  ({steer:+.3f} rad)',
            f'장면   {"발행함 (빈 장면)" if scene_pub else "없음 - HUD 는 Basic"}',
            '',
            '← →  조향     ↑ ↓  속도     스페이스  정지',
            'F 전진   R 후진   P 주차   N 중립   ESC 종료',
        ]
        for i, line in enumerate(lines):
            screen.blit(font.render(line, True, (200, 200, 205)), (200, 40 + i * 34))
        pygame.display.flip()

        rclpy.spin_once(node, timeout_sec=0.0)
        time.sleep(max(0.0, period - (time.monotonic() - now)))

    pygame.quit()
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
