"""운전 입력. 키보드와 휠, 그리고 그 중 누가 모는가."""

import math

import pytest

from hero_coach.driver_input import (
    MAX_ROAD_WHEEL_DEG, STEER_DEADZONE, DriverInput, KeyboardSource, WheelSource, pick_source,
)


class Keys:
    """pygame.key.get_pressed() 흉내. 눌린 키만 담는다."""

    def __init__(self, *pressed):
        self.pressed = set(pressed)

    def __getitem__(self, key):
        return key in self.pressed


UP, DOWN, LEFT, RIGHT, SPACE = 1, 2, 3, 4, 5
BINDING = {'accelerate': (UP,), 'brake': (DOWN,), 'left': (LEFT,), 'right': (RIGHT,),
           'hand_brake': (SPACE,)}


# ------------------------------------------------------------------ 키보드

def test_keyboard_accelerates_and_brakes():
    source = KeyboardSource(BINDING)
    assert source.poll(Keys(UP), 0.02).throttle == 1.0
    assert source.poll(Keys(DOWN), 0.02).brake == 1.0
    assert source.poll(Keys(SPACE), 0.02).hand_brake is True


def test_keyboard_steer_winds_up_and_returns():
    # 키보드에는 각도가 없다. 누르고 있는 동안 조향이 쌓이고, 놓으면 돌아온다.
    source = KeyboardSource(BINDING)
    first = source.poll(Keys(LEFT), 0.1).steer
    second = source.poll(Keys(LEFT), 0.1).steer
    assert second < first < 0.0
    assert source.poll(Keys(), 0.1).steer == 0.0


def test_keyboard_steer_is_bounded():
    source = KeyboardSource(BINDING)
    for _ in range(200):
        steer = source.poll(Keys(RIGHT), 0.1).steer
    assert steer == pytest.approx(0.7)


# --------------------------------------------------------------------- 휠

def test_wheel_centre_is_straight():
    # 데드존 안은 0이다. 손을 놓은 휠이 미세하게 떨어도 차가 흐르지 않는다.
    source = WheelSource()
    assert source.poll([STEER_DEADZONE * 0.5, 1.0, 1.0], [0] * 13).steer == 0.0


def test_wheel_full_lock_is_full_command():
    # T300RS 끝(1080도의 절반)에서 노면각이 37.8도, 그것이 CARLA 의 ±1 이다.
    source = WheelSource()
    assert source.poll([1.0, 1.0, 1.0], [0] * 13).steer == pytest.approx(1.0)
    assert source.poll([-1.0, 1.0, 1.0], [0] * 13).steer == pytest.approx(-1.0)


def test_wheel_half_turn_is_half_the_road_angle():
    source = WheelSource()
    steer = source.poll([0.5, 1.0, 1.0], [0] * 13).steer
    assert steer * MAX_ROAD_WHEEL_DEG == pytest.approx(0.5 * 1080.0 * 0.5 / 14.2857, abs=0.2)


def test_wheel_pedals_are_released_at_plus_one():
    # T300RS 페달은 **놓으면 +1, 끝까지 밟으면 -1** 로 온다. 곡선도 선형이 아니다
    # (HERO_Calib wheel-drive-seated-geometry 의 pedal_command 와 같은 식).
    source = WheelSource()
    rest = source.poll([0.0, 1.0, 1.0], [0] * 13)
    assert rest.throttle == 0.0 and rest.brake == 0.0
    floored = source.poll([0.0, -1.0, -1.0], [0] * 13)
    assert floored.throttle == pytest.approx(1.0) and floored.brake == pytest.approx(1.0)
    half = source.poll([0.0, 0.0, 0.0], [0] * 13).throttle
    assert 0.3 < half < 0.7      # 중간에서 중간쯤. 로그 곡선이라 정확히 0.5 는 아니다


def test_wheel_handbrake_button():
    source = WheelSource()
    buttons = [0] * 13
    buttons[5] = 1
    assert source.poll([0.0, 1.0, 1.0], buttons).hand_brake is True


# ----------------------------------------------------------------- 누가 모나

def test_explicit_modes_pick_one():
    assert pick_source('keyboard', Keys(), BINDING) == 'keyboard'
    assert pick_source('wheel', Keys(UP), BINDING) == 'wheel'


def test_auto_gives_the_frame_to_whoever_pressed_a_key():
    # 휠이 꽂힌 채로도 키보드가 먹어야 한다. 휠 파싱이 키 입력을 매 프레임 덮어쓰던
    # 것이 2026-11-05 에 "키보드가 안 된다"로 나타났다.
    assert pick_source('auto', Keys(UP), BINDING) == 'keyboard'
    assert pick_source('auto', Keys(), BINDING) == 'wheel'


def test_none_means_nobody_drives():
    # 대본 주행(ego_pilot)이 모는 동안 창은 보기만 한다. 창이 매 프레임 0 을 쓰면
    # 다른 노드가 아무리 명령해도 차가 서 있다 -- 2026-11-05 에 그렇게 막혔다.
    assert pick_source('none', Keys(UP), BINDING) == 'none'
    assert pick_source('none', Keys(), BINDING, has_wheel=False) == 'none'


def test_auto_without_a_wheel_is_keyboard():
    assert pick_source('auto', Keys(), BINDING, has_wheel=False) == 'keyboard'


# ------------------------------------------------------------------- 기하

def test_driver_input_is_plain_data():
    one = DriverInput(steer=0.1, throttle=0.2, brake=0.0, hand_brake=False)
    assert one.steer == 0.1 and one.throttle == 0.2
    assert math.isclose(one.brake, 0.0)
