# 입력 소스 하나의 건강도. 저하 상태 넷이 여기서 나온다.
#
# HUD가 스스로 신선도를 판정하지 않는 이유: 소스마다 정상 주기가 다르고
# (오도메트리용 카메라 60 Hz, 표지판 검출 10 Hz), 그 지식은 상류에 있다.

string name      # "front_camera", "rear_camera", "wide_camera", "imu", "can", "odometry", "occupancy"
float32 age_sec  # 마지막 갱신 이후 경과
bool ok          # 상류가 내리는 판정. 이것이 거짓이면 HUD는 해당 요소를 철회한다
