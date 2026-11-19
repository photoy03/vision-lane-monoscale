# 상단 신호 인디케이터. 월드 좌표가 아니라 **화면 상단 고정 자리**에 그린다.
#
# 신호등은 위치가 아니라 상태가 정보다. 지면에 심볼로 놓으면 74 x 46 셀에서
# 읽히지 않고, 고정 자리에 두면 위치가 학습되어 흘끗 보기로 읽힌다.

uint8 STATE_NONE   = 0
uint8 STATE_RED    = 1
uint8 STATE_YELLOW = 2
uint8 STATE_GREEN  = 3

uint8 state

# 화살표 신호. 상태와 독립이다 (적색 + 좌회전 화살표가 동시에 켜질 수 있다).
bool arrow_left
bool arrow_right
