"""운전 입력 하나. 키보드와 T300RS 휠, 그리고 그 프레임을 누가 모는가.

주행 창(`~/carla_wheel/drive_ego.py`)이 이것을 쓴다. CARLA 도 pygame 도 import 하지
않으므로 단독으로 시험된다 — 키 상태와 축 값만 받고 명령만 돌려준다.

휠 쪽 수치는 HERO_Calib 의 `wheel-drive-seated-geometry` 브랜치에서 가져왔다
(`prius_wheel_based_driver_eye.py`). 3세대 프리우스의 조향비 17.6:1 과 락투락 3.7 회전이
노면각 37.8도를 주고, T300RS 의 1080도가 그 끝에 닿는다 — 휠 기준 14.29:1.

**왜 모듈인가.** 전에는 주행 창이 키 처리와 휠 처리를 차례로 부르면서 휠 값이 키
입력을 매 프레임 덮어썼다. 휠을 꽂은 채로는 키보드가 죽은 것처럼 보였고, 그 사실이
코드 어디에도 적혀 있지 않았다. 누가 모는가를 한 함수로 꺼내 두면 그 질문에 답이
하나만 존재한다.
"""

from dataclasses import dataclass
import math
from typing import Dict, Mapping, Sequence, Tuple

# --- 휠 (Thrustmaster T300RS GT) ---
STEER_AXIS, BRAKE_AXIS, THROTTLE_AXIS = 0, 1, 2
REVERSE_BUTTON, HANDBRAKE_BUTTON = 4, 5
STEER_DEADZONE = 0.01
WHEEL_RANGE_DEG = 1080.0

# --- 3세대 프리우스(ZVW30) ---
STEERING_RATIO = 17.6            # 2010 유럽 사양, 15인치
LOCK_TO_LOCK_TURNS = 3.7         # 같은 자료. 3.7 * 180 / 17.6 = 37.8도
MAX_ROAD_WHEEL_DEG = LOCK_TO_LOCK_TURNS * 180.0 / STEERING_RATIO
WHEEL_TO_ROAD_RATIO = WHEEL_RANGE_DEG * 0.5 / MAX_ROAD_WHEEL_DEG

# --- 키보드 ---
# 각도가 없는 입력이라 누르고 있는 동안 쌓고 놓으면 돌아온다. 0.7 은 조향 명령의
# 상한이지 노면각이 아니다 — 키보드로 끝까지 꺾을 일은 없다고 보는 쪽이 안전하다.
KEY_STEER_RATE = 0.5             # 초당 조향 명령
KEY_STEER_LIMIT = 0.7


@dataclass(frozen=True)
class DriverInput:
    """한 프레임의 운전 명령. CARLA VehicleControl 로 그대로 옮겨진다."""

    steer: float = 0.0
    throttle: float = 0.0
    brake: float = 0.0
    hand_brake: bool = False
    reverse_pressed: bool = False


def _clamp(value: float, low: float = -1.0, high: float = 1.0) -> float:
    return max(low, min(high, value))


class KeyboardSource:
    """키 상태 → 명령. 조향만 상태를 들고 있다(누르는 동안 쌓인다)."""

    def __init__(self, binding: Mapping[str, Sequence[int]]):
        self.binding = binding
        self.steer = 0.0

    def _held(self, keys, action: str) -> bool:
        return any(keys[key] for key in self.binding.get(action, ()))

    def poll(self, keys, dt: float) -> DriverInput:
        step = KEY_STEER_RATE * max(dt, 0.0)
        if self._held(keys, 'left'):
            self.steer = max(-KEY_STEER_LIMIT, self.steer - step)
        elif self._held(keys, 'right'):
            self.steer = min(KEY_STEER_LIMIT, self.steer + step)
        else:
            self.steer = 0.0
        return DriverInput(
            steer=self.steer,
            throttle=1.0 if self._held(keys, 'accelerate') else 0.0,
            brake=1.0 if self._held(keys, 'brake') else 0.0,
            hand_brake=self._held(keys, 'hand_brake'),
            reverse_pressed=self._held(keys, 'reverse'))


class WheelSource:
    """축과 버튼 → 명령. 상태가 없다 — 휠 자신이 각도를 들고 있다."""

    @staticmethod
    def steer_from_axis(axis: float) -> float:
        axis = _clamp(axis)
        if abs(axis) <= STEER_DEADZONE:
            return 0.0
        road_wheel_deg = axis * WHEEL_RANGE_DEG * 0.5 / WHEEL_TO_ROAD_RATIO
        return _clamp(road_wheel_deg / MAX_ROAD_WHEEL_DEG)

    @staticmethod
    def pedal(axis: float) -> float:
        """T300RS 페달 축(놓으면 +1, 끝까지 밟으면 -1)을 0..1 명령으로.

        선형이 아니라 로그다. HERO_Calib 의 pedal_command 와 같은 식을 쓴다 —
        페달 감각은 그쪽에서 맞춘 것이고, 여기서 다시 고르면 같은 리그가 두 가지
        답을 갖게 된다.
        """
        value = 1.6 + (2.05 * math.log10(-0.7 * _clamp(axis) + 1.4) - 1.2) / 0.92
        return _clamp(value, 0.0, 1.0)

    def poll(self, axes: Sequence[float], buttons: Sequence[int]) -> DriverInput:
        def button(index: int) -> bool:
            return index < len(buttons) and bool(buttons[index])

        return DriverInput(
            steer=self.steer_from_axis(axes[STEER_AXIS]),
            throttle=self.pedal(axes[THROTTLE_AXIS]),
            brake=self.pedal(axes[BRAKE_AXIS]),
            hand_brake=button(HANDBRAKE_BUTTON),
            reverse_pressed=button(REVERSE_BUTTON))


def pick_source(mode: str, keys, binding: Mapping[str, Sequence[int]],
                has_wheel: bool = True) -> str:
    """이 프레임을 누가 모는가. 'keyboard' 또는 'wheel'.

    auto 는 **주행 키를 누른 프레임만** 키보드다. 상태를 두지 않으므로 키를 놓는
    순간 휠로 돌아간다 — 페달에서 발을 뗀 휠은 0 을 주므로 차는 그냥 타력으로 간다.
    """
    # none 은 아무도 안 몬다. 대본 주행(ego_pilot)처럼 다른 노드가 작동 토픽으로 몰
    # 때 쓴다 -- 창이 매 프레임 0 을 쓰면 그 명령이 덮여 차가 서 있다.
    if mode == 'none':
        return 'none'
    if mode == 'keyboard' or not has_wheel:
        return 'keyboard'
    if mode == 'wheel':
        return 'wheel'
    pressed = any(keys[key] for action in binding for key in binding[action])
    return 'keyboard' if pressed else 'wheel'


def pygame_binding(locals_module) -> Dict[str, Tuple[int, ...]]:
    """pygame.locals 에서 기본 키 묶음을 만든다. 주행 창이 부른다."""
    key = locals_module
    return {
        'accelerate': (key.K_UP, key.K_w),
        'brake': (key.K_DOWN, key.K_s),
        'left': (key.K_LEFT, key.K_a),
        'right': (key.K_RIGHT, key.K_d),
        'hand_brake': (key.K_SPACE,),
    }
