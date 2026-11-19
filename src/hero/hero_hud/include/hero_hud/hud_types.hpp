// HUD 표시 모델. Qt 의존도 ROS 의존도 없다.
//
// 여기 있는 것은 "화면에 그릴 것"이지 "인지가 관측한 것"이 아니다. 인지 쪽
// 분류 체계가 바뀌어도 이 파일은 안 바뀌어야 한다. docs/HUD_전략.md 7절.
#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace hero_hud {

// base_link 지면 좌표. x 전방, y 좌측. 단위 m.
struct Vec2 {
  float x{0.0f};
  float y{0.0f};
};

enum class Gear : uint8_t { Unknown = 0, Park = 1, Reverse = 2, Neutral = 3, Drive = 4 };
enum class Threat : uint8_t { Info = 0, Caution = 1, Warn = 2, Critical = 3 };
enum class Source : uint8_t { Unknown = 0, Front = 1, Rear = 2, Wide = 3, Fused = 4 };
enum class LightState : uint8_t { Unknown = 0, Red = 1, Yellow = 2, Green = 3 };
enum class Severity : uint8_t { Info = 0, Caution = 1, Warn = 2, Critical = 3 };

struct Ribbon {
  std::vector<Vec2> points;   // 중심선, 차량에서 먼 쪽으로 진행
  float width_m{1.82f};
  float confidence{1.0f};
};

struct Obstacle {
  Vec2 position;
  float extent_m{0.30f};
  float height_m{0.0f};
  Threat threat{Threat::Info};
  Source source{Source::Unknown};
  float confidence{1.0f};
};

struct Sign {
  uint8_t kind{0};
  float value{0.0f};
  Vec2 position;
  float confidence{1.0f};
  Source source{Source::Unknown};
};

struct TrafficLight {
  LightState state{LightState::Unknown};
  Vec2 position;
  float confidence{1.0f};
  Source source{Source::Unknown};
};

// 상단 고정 자리. 신호등은 위치가 아니라 상태가 정보다.
enum class SignalState : uint8_t { None = 0, Red = 1, Yellow = 2, Green = 3 };

struct Signal {
  SignalState state{SignalState::None};
  bool arrow_left{false};
  bool arrow_right{false};
};

struct Coaching {
  std::string text;
  Severity severity{Severity::Info};
  double expires_at{0.0};   // monotonic sec, 0이면 무기한
};

struct Health {
  std::string name;
  float age_sec{0.0f};
  bool ok{false};
};

// CAN 계층. 인지가 전부 죽어도 이것은 살아 있다.
struct VehicleState {
  Gear gear{Gear::Unknown};
  float speed_mps{0.0f};
  float steering_angle_rad{0.0f};   // 조향륜 각도, 좌회전 양수
  float yaw_rate_rps{0.0f};
  double stamp{0.0};                // 수신 시각 (monotonic sec). 0이면 미수신
};

// 인지 계층. 없거나 낡을 수 있다.
struct RoadLine {
  std::vector<Vec2> points;
  bool left{false};
  bool predicted{false};
};

// vision-road /road/road_markings: 정지선은 두 점, 횡단보도·노면 표시는 닫힌 외곽선.
// 내용(글자, 화살표 방향)은 없다. "여기에 표시가 있다"까지만 온다.
enum class RoadMarkingKind { StopLine, Crosswalk, Marking };

struct RoadMarking {
  RoadMarkingKind kind{RoadMarkingKind::Marking};
  std::vector<Vec2> points;
  bool predicted{false};  // 확정됐지만 이번 프레임에는 안 보임
};

struct RoadPaint {
  std::vector<RoadLine> lines;       // measured paint, kept as fragments
  std::vector<RoadLine> boundaries;  // fitted display curves
  std::vector<RoadMarking> markings;
  double marking_stamp{0.0};
  bool dashed_left{false};
  bool dashed_right{false};
  double stamp{0.0};                 // latest measured paint receipt
  double boundary_stamp{0.0};
  double left_style_stamp{0.0};
  double right_style_stamp{0.0};
};

// 주차 평면도(plan view) 한 프레임. 상류의 표시 필터가 뒤축 기준 지면 좌표로 보낸다.
enum class ParkingShape : uint8_t {
  SlotFree, SlotOccupied, SlotUnknown, SlotSelected, Goal,
  PathForward, PathReverse, PathUnverified
};

struct ParkingItem {
  ParkingShape kind{ParkingShape::SlotUnknown};
  std::vector<Vec2> points;
};

struct ParkingView {
  std::vector<ParkingItem> items;
  double stamp{0.0};
};

struct SceneModel {
  bool has_ideal{false};
  Ribbon ideal;
  std::vector<Obstacle> obstacles;
  // 고정 자리 셋. 지면이 아니라 화면 가장자리에 그린다.
  bool has_sign{false};
  Sign sign;                       // 우측
  bool has_signal{false};
  Signal signal;                   // 상단
  Gear advised_gear{Gear::Unknown};  // 좌측
  bool has_coach{false};
  Coaching coach;
  std::vector<Health> health;
  double stamp{0.0};
};

// 주변광. CAN과 같은 "항상 살아 있어야 하는" 계층이다.
struct Ambient {
  float relative{0.85f};   // 신호 없으면 밝은 쪽으로 가정한다
  double stamp{0.0};
};

// 저하 상태 넷. docs/HUD_전략.md 6절.
//
//   Waiting   CAN 없음        — 정적 프레임 + 연결 대기. 검은 화면 금지
//   Basic     CAN만 살아있음  — 현재궤적 + 기어 + 속도
//   Normal    전 소스 신선    — 전체 표시
//   Untrusted 인지 신뢰 불가  — 이상궤적·장애물 철회, 명시적 경고
enum class HudMode : uint8_t { Waiting, Basic, Normal, Untrusted };

}  // namespace hero_hud
