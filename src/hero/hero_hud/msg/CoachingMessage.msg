# 사용자에게 보여줄 코칭 문구. 문장 생성은 상류(hero_coach)의 일이다.

uint8 SEVERITY_INFO     = 0
uint8 SEVERITY_CAUTION  = 1
uint8 SEVERITY_WARN     = 2
uint8 SEVERITY_CRITICAL = 3

# 짧게. 저시력 사용자에게 큰 글씨로 보여야 하므로 화면이 감당하는 길이가 짧다.
string text

uint8 severity

# 이 문구를 얼마나 유지할지(초). 0이면 다음 메시지까지.
float32 ttl_sec
