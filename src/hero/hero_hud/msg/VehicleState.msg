# CAN에서 오는 차량 상태. HUD가 직접 구독한다 (hero_coach를 경유하지 않는다).
#
# 이 토픽은 HUD의 "항상 살아 있는" 계층이다. 인지 스택이 전부 죽어도 이것만
# 있으면 현재궤적·기어·속도를 계속 그릴 수 있다. 항상 살아 있어야 하는 신호를
# 가장 잘 죽는 노드에 묶지 않기 위한 의도적인 분리다. docs/HUD_전략.md 5절.

std_msgs/Header header

uint8 GEAR_UNKNOWN = 0
uint8 GEAR_PARK    = 1
uint8 GEAR_REVERSE = 2
uint8 GEAR_NEUTRAL = 3
uint8 GEAR_DRIVE   = 4

# 기어는 HUD의 모드 선택자다. 인지에서 추론하지 않는다.
uint8 gear

# 부호 있는 종방향 속도. 후진 시 음수.
float32 speed_mps

# 조향륜 각도(라디안), 좌회전이 양수.
# CAN이 주는 것은 보통 스티어링 휠 각도이므로 어댑터가 조향비로 나눠서 채운다.
# 조향비는 아직 미측정이다.
float32 steering_angle_rad

# IMU 또는 CAN 유래 요레이트(rad/s), 좌회전이 양수. 현재는 표시에 쓰지 않는다.
float32 yaw_rate_rps
