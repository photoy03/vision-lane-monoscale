# 장애물 하나. 표시 의미를 담지 인지 쪽 분류 체계를 담지 않는다.
#
# class_id 같은 것을 그대로 흘리면 인지의 분류 체계가 바뀔 때마다 HUD가 흔들린다.
# HUD가 알아야 하는 것은 "얼마나 위험한가"뿐이다.

uint8 THREAT_INFO     = 0
uint8 THREAT_CAUTION  = 1
uint8 THREAT_WARN     = 2
uint8 THREAT_CRITICAL = 3

uint8 SOURCE_UNKNOWN = 0
uint8 SOURCE_FRONT   = 1
uint8 SOURCE_REAR    = 2
uint8 SOURCE_WIDE    = 3
uint8 SOURCE_FUSED   = 4

# base_link 지면 위치.
geometry_msgs/Point32 position

# 발자국 반경(m).
float32 extent_m

# 높이(m). 0이면 미상. 빌보드 심볼 크기에 쓴다.
float32 height_m

uint8 threat

# 어느 카메라에서 왔는지. 상위 문서 16-8/17-5가 전후방 정밀도 격차를
# 0.497 대 0.924로 재놓았으므로 표현에 반영할 통로를 열어둔다.
uint8 source

# 0..1.
float32 confidence
