# 표지판 하나. 세 번째 카메라(전방 수평)에서 온다고 가정한다 — 미확정.

uint8 KIND_UNKNOWN   = 0
uint8 KIND_STOP      = 1
uint8 KIND_SPEED     = 2
uint8 KIND_NO_ENTRY  = 3
uint8 KIND_PARKING   = 4
uint8 KIND_CROSSING  = 5

# ROS 메시지에는 공유 열거형이 없어서 ObstacleMarker와 같은 값을 되풀이한다.
uint8 SOURCE_UNKNOWN = 0
uint8 SOURCE_FRONT   = 1
uint8 SOURCE_REAR    = 2
uint8 SOURCE_WIDE    = 3
uint8 SOURCE_FUSED   = 4

uint8 kind

# 종류에 딸린 수치. 속도 표지판이면 제한속도, 아니면 0.
float32 value

# base_link 지면 위치(표지판 기둥 밑점).
geometry_msgs/Point32 position

float32 confidence
uint8 source
