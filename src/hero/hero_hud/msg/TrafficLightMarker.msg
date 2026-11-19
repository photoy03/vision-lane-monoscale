# 신호등 하나.

uint8 STATE_UNKNOWN = 0
uint8 STATE_RED     = 1
uint8 STATE_YELLOW  = 2
uint8 STATE_GREEN   = 3

# ROS 메시지에는 공유 열거형이 없어서 ObstacleMarker와 같은 값을 되풀이한다.
uint8 SOURCE_UNKNOWN = 0
uint8 SOURCE_FRONT   = 1
uint8 SOURCE_REAR    = 2
uint8 SOURCE_WIDE    = 3
uint8 SOURCE_FUSED   = 4

uint8 state

# base_link 지면 위치(신호등 기둥 밑점). 빌보드로 그 위에 심볼을 얹는다.
geometry_msgs/Point32 position

float32 confidence
uint8 source
