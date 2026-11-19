# 한 프레임에 그릴 것 전부. 원자적으로 발행된다.
#
# HUD가 여섯 개 토픽을 서로 다른 주기와 서로 다른 신선도로 렌더 시점에
# 화해시키는 일을 없애기 위한 설계다. 시간 동기는 상류(hero_coach)에서 끝난다.
# docs/HUD_전략.md 7절.
#
# 좌표는 전부 base_link 지면(z=0). HUD는 TF를 조회하지 않는다.

std_msgs/Header header

# 계획된 이상궤적. 인지 + 목표 pose + 계획이 모두 있어야 나오므로
# 없을 수 있다. 현재궤적은 여기 없다 — 그것은 CAN에서 HUD가 직접 만든다.
bool has_ideal
TrajectoryRibbon ideal

ObstacleMarker[] obstacles

# 우측 고정 자리에 띄울 표지판. 우선순위가 가장 높은 하나만 보낸다 —
# 여러 개를 늘어놓을 화소 예산이 없다.
bool has_sign
SignMarker sign

# 상단 고정 자리의 신호 인디케이터.
bool has_signal
SignalIndicator signal

# 좌측 고정 자리. 지금 넣어야 할 기어. GEAR_UNKNOWN 이면 표시하지 않는다.
# VehicleState 의 기어 상수와 같은 값을 쓴다.
uint8 advised_gear

bool has_coach
CoachingMessage coach

SourceHealth[] health
