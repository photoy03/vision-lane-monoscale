// 테스트 시나리오 발행 노드.
//
// hero_coach가 아직 없고, 인지 스택도 CARLA도 bag도 이 워크스페이스에 없다.
// 그런데 HUD는 그것들 없이도 전체가 구동되어야 한다 — 저시력 사용자 피드백
// 세션을 인지 스택 없이 돌릴 수 있어야 하기 때문이다. docs/HUD_전략.md 7절.
//
// 월드에 장애물을 고정해 두고 자전거 모델로 자차를 적분한 뒤, 매 프레임
// base_link로 변환해서 발행한다. 상대 운동이 실제와 같은 방식으로 생긴다.
//
//   ros2 run hero_hud scenario_publisher --ros-args -p scenario:=approach
//   ros2 run hero_hud scenario_publisher --ros-args -p scenario:=degraded
#include <chrono>
#include <cmath>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "hero_hud/msg/ambient_light.hpp"
#include "hero_hud/msg/scene_overlay.hpp"
#include "hero_hud/msg/vehicle_state.hpp"

using namespace std::chrono_literals;

namespace {

struct WorldObject {
  float x{0.0f};
  float y{0.0f};
  float extent{0.5f};
  float height{1.0f};
};

struct Command {
  float speed_mps{0.0f};
  float steering_rad{0.0f};
  uint8_t gear{hero_hud::msg::VehicleState::GEAR_PARK};
};

class ScenarioPublisher : public rclcpp::Node {
 public:
  ScenarioPublisher() : rclcpp::Node("hero_hud_scenario") {
    scenario_ = declare_parameter<std::string>("scenario", "approach");
    wheelbase_ = static_cast<float>(declare_parameter<double>("vehicle.wheelbase_m", 2.700));
    loop_sec_ = declare_parameter<double>("loop_sec", 24.0);
    // 주변광. 실차·시뮬에서는 전방 카메라 자동노출이 이 자리를 채운다.
    ambient_ = static_cast<float>(declare_parameter<double>("ambient", 0.85));

    rclcpp::QoS qos(rclcpp::KeepLast(1));
    qos.best_effort();
    pub_vehicle_ = create_publisher<hero_hud::msg::VehicleState>("/hero/vehicle_state", qos);
    pub_scene_ = create_publisher<hero_hud::msg::SceneOverlay>("/hero/scene_overlay", qos);
    rclcpp::QoS ambient_qos(rclcpp::KeepLast(1));
    ambient_qos.reliable().transient_local();
    pub_ambient_ =
        create_publisher<hero_hud::msg::AmbientLight>("/hero/ambient", ambient_qos);

    buildWorld();
    start_ = now_sec();

    // CAN 계층은 인지보다 빠르다. 실차의 비율을 흉내낸다.
    timer_vehicle_ = create_wall_timer(20ms, [this]() { tickVehicle(); });
    timer_scene_ = create_wall_timer(50ms, [this]() { tickScene(); });

    RCLCPP_INFO(get_logger(), "시나리오 '%s', 주기 %.1f s", scenario_.c_str(), loop_sec_);
  }

 private:
  // ------------------------------------------------------------------ 월드

  void buildWorld() {
    // 주차된 차와 배리어, 콘. 상위 문서 21-1이 "주차 보조가 부딪히는 것은
    // 15 m 밖 건물이 아니다"라고 적어놓은 그 물체들이다.
    obstacles_ = {
        {11.0f, 1.30f, 0.25f, 0.70f},   // 콘
        {14.0f, 2.60f, 0.90f, 1.45f},   // 주차된 차
        {17.5f, -1.70f, 0.50f, 0.90f},  // 배리어
        {20.5f, -3.20f, 0.90f, 1.45f},  // 주차된 차
        {25.0f, 2.40f, 0.90f, 1.45f},   // 주차된 차
    };

    // 목표 자세. 주차 슬롯 입구라고 보면 된다. 월드 고정이다.
    // 슬롯 양옆 주차차량.
    arrive_obstacles_ = {
        {100.5f, -3.4f, 0.90f, 1.45f},   // 슬롯 뒤차
        // 슬롯 앞차. **접근 호가 스치지 않는 자리**로 뺀다 — 전에는 전진 중
        // 코리도가 이 차를 스치며 지나가 붉음이 한 번 붙었다 떨어졌다 했다.
        {111.5f, -3.6f, 0.90f, 1.45f},   // 슬롯 앞차
        {124.0f, 0.2f, 0.50f, 0.90f},    // 통로 끝 배리어 (정지 지점에서 사거리 밖)
        // **슬롯 안쪽 끝(연석).** 후진이 끝나는 자리 바로 뒤에 둔다.
        //
        // 이것이 있어야 "주차 완료"가 화면에 나온다 — 정지선이 가장 가까운
        // 눈금까지 내려오고 거기서 기어가 P 로 간다. 새 심볼이 필요 없다.
        // 눈금이 자이고 정지선이 지금 값이고 P 가 상태다.
        {102.98f, -2.77f, 0.30f, 0.35f},
    };

    target_x_ = 7.0f;
    target_y_ = -0.9f;
    target_yaw_ = -0.18f;
  }

  static double now_sec() {
    const auto d = std::chrono::steady_clock::now().time_since_epoch();
    return std::chrono::duration<double>(d).count();
  }

  double phase() const {
    double t = std::fmod(now_sec() - start_, loop_sec_);
    if (t < 0.0) {
      t += loop_sec_;
    }
    return t;
  }

  // 주행 대본. 일반 도로 50 km/h 언저리.
  //
  // 차선 유지가 과제이므로 **횡오차를 일부러 만든다** — 조향을 느린 사인으로
  // 흔들면 자차가 차선 중심에서 벗어나고, 목표선이 되돌아오는 길을 보여준다.
  // 코리도(지금 조향)와 목표선(차선으로 되돌아가는 길)이 벌어진 만큼이
  // "핸들을 얼마나 되돌려야 하는가"다.
  //
  // 중간에 선행차가 가까워져 감속하고, 신호 앞에서 한 번 선다.
  Command cruiseAt(double t) const {
    Command c;
    using VS = hero_hud::msg::VehicleState;
    c.gear = VS::GEAR_DRIVE;
    const float cruise = 13.9f;   // 50 km/h

    if (t < 3.0) {
      c.speed_mps = static_cast<float>(cruise * t / 3.0);
    } else if (t < 13.0) {
      c.speed_mps = cruise;
    } else if (t < 17.0) {
      // 선행차가 가까워진다. 절반까지 줄인다.
      c.speed_mps = static_cast<float>(cruise * (1.0 - 0.5 * (t - 13.0) / 4.0));
    } else if (t < 21.0) {
      // 적신호. 완전히 선다.
      c.speed_mps = static_cast<float>(cruise * 0.5 * (21.0 - t) / 4.0);
    } else if (t < 23.0) {
      c.speed_mps = 0.0f;
    } else {
      c.speed_mps = static_cast<float>(cruise * 0.5 * (t - 23.0) / 1.0);
    }

    // 차선 유지 조향. 진폭을 작게 둔다 — 크게 흔들면 차선을 벗어나 버려서
    // "조금 벗어났고 되돌린다"는 상황이 안 나온다.
    if (c.speed_mps > 0.5f) {
      // **개루프 사인으로는 차선 유지가 안 된다.** 진폭을 아무리 줄여도 반주기
      // 동안 한쪽으로 적분되어 결국 차선을 벗어난다 — 0.0022 rad 로도 79 m 를
      // 달리는 동안 2.5 m, 차선 하나를 통째로 나간다.
      //
      // 실제 운전이 그렇듯 **되돌리는 피드백**을 넣는다. 외란은 작게 얹어
      // 흔들림을 만들고, 이득은 50 km/h 에서 쓰는 크기(0.1~0.3도)에 맞춘다.
      // 그래야 코리도와 목표선이 "조금 벌어졌다 되돌아온다".
      // **목표 경로를 추종한다.** 외란을 작게 얹어 완벽히 붙지는 않게 한다 —
      // 코리도와 목표선이 늘 정확히 겹치면 그 둘을 비교하는 화면의 의미가 없다.
      const double disturb = 0.0007 * std::sin(t * 0.75) +
                             0.0004 * std::sin(t * 1.9);
      c.steering_rad = pursue(laneRibbon(), c.speed_mps) +
                       static_cast<float>(disturb);
    }
    return c;
  }

  // 도착 대본. **주행 → 감속 → 정지 → 후진 주차.**
  //
  // 이것이 있어야 표시 어법의 전환(13-48)을 눈으로 볼 수 있다. approach 는 계속
  // 주차이고 cruise 는 계속 주행이라, 판별기가 실제로 넘어가는 장면이 어느
  // 대본에도 없었다 — 단위 시험으로만 확인한 상태였다.
  Command arriveAt(double t) const {
    Command c;
    using VS = hero_hud::msg::VehicleState;
    c.gear = VS::GEAR_DRIVE;
    const float road = 11.0f;   // 40 km/h

    if (t < 2.0) {
      c.speed_mps = static_cast<float>(road * t / 2.0);
    } else if (t < 8.0) {
      c.speed_mps = road;
    } else if (t < 13.0) {
      c.speed_mps = static_cast<float>(road - (road - 1.2) * (t - 8.0) / 5.0);
    } else if (t < 15.0) {
      c.speed_mps = 1.2f;
    } else if (t < 16.0) {
      c.speed_mps = 0.0f;
      c.gear = VS::GEAR_PARK;
    } else if (t < 22.0) {
      c.gear = VS::GEAR_REVERSE;
      c.speed_mps = -1.0f;
      // 후진도 같은 추종을 쓴다. **부호를 뒤집지 않는다** — 자전거 모델에서
      // 경로 곡률은 tan(delta)/L 이라 진행 방향과 무관하고, 같은 조향이 같은
      // 원을 그린다. 앞서 부호를 뒤집었던 것이 조향이 튄 원인이었다.
      c.steering_rad = pursue(arriveIdeal(), c.speed_mps);
      return c;
    } else {
      c.gear = VS::GEAR_PARK;
      c.speed_mps = 0.0f;
      return c;
    }

    if (c.speed_mps > 0.5f) {
      // 목표가 차선에서 슬롯으로 넘어가면 차도 그리로 따라 들어간다 —
      // 그래야 "계획대로 가는 중"이 화면에 보인다.
      c.steering_rad = pursue(arriveIdeal(), c.speed_mps) +
                       static_cast<float>(0.0006 * std::sin(t * 0.8));
    }
    return c;
  }

  // 대본. 전진 가속 → 좌우 조향 → 정지 → 후진 조향 → 정지.
  Command commandAt(double t) const {
    Command c;
    using VS = hero_hud::msg::VehicleState;
    if (scenario_ == "cruise") {
      return cruiseAt(t);
    }
    if (scenario_ == "arrive") {
      return arriveAt(t);
    }
    if (t < 2.0) {
      c.gear = VS::GEAR_DRIVE;
      c.speed_mps = static_cast<float>(1.5 * t / 2.0);
    } else if (t < 10.0) {
      c.gear = VS::GEAR_DRIVE;
      c.speed_mps = 1.5f;
      // 진폭을 작게 둔다. 크게 흔들면 자차가 이상궤적에서 멀어져 두 리본을
      // 나란히 못 본다 — 이 화면이 실제로 보여주려는 것이 그 비교다.
      // **계획을 추종한다.** 전에는 사인으로 흔들기만 해서 차가 목표를 8 m
      // 지나쳐 갔고(드라이브바이), 코리도와 목표선이 평균 131 px 어긋난 채였다.
      // 이 화면이 보여주려는 것이 그 둘의 관계인데 매칭이 안 됐다.
      c.steering_rad = pursue(approachIdeal(), c.speed_mps) +
                       static_cast<float>(0.004 * std::sin((t - 2.0) * 1.20));
    } else if (t < 12.0) {
      c.gear = VS::GEAR_DRIVE;
      c.speed_mps = static_cast<float>(1.5 * (12.0 - t) / 2.0);
    } else if (t < 13.0) {
      c.gear = VS::GEAR_PARK;
    } else if (t < 20.0) {
      c.gear = VS::GEAR_REVERSE;
      c.speed_mps = -1.0f;
      c.steering_rad = 0.25f;
    } else {
      c.gear = VS::GEAR_PARK;
    }
    return c;
  }

  // ---------------------------------------------------------------- 자차 적분

  void tickVehicle() {
    const double t = phase();
    if (t < last_phase_) {
      // 주기가 넘어갔다 — 자차를 원점으로 되돌려 같은 장면을 반복한다.
      ego_x_ = 0.0f;
      ego_y_ = 0.0f;
      ego_theta_ = 0.0f;
      steer_ = 0.0f;
    }
    last_phase_ = t;

    Command c = commandAt(t);
    const double now = now_sec();
    const float dt = (last_tick_ > 0.0) ? static_cast<float>(now - last_tick_) : 0.0f;
    last_tick_ = now;

    // **대본이 내는 조향은 목표이고, 실제 조향은 고정 속도로 거기 수렴한다.**
    //
    // 목표를 그대로 실제 조향으로 쓰면 계획이 꺾이는 순간 핸들이 순간이동한다.
    // 사람은 그렇게 못 돌리고, 무엇보다 화면이 보여주려는 것이 **코리도가
    // 목표 호로 다가가는 과정**인데 순간이동하면 그 과정이 없다.
    //
    // 이 한 줄이 후진 제어기 문제도 없앤다 — 어떤 목표가 오든 조향은 한 번에
    // 한 걸음씩만 움직이므로 튀지 않는다. 가상 시연이므로 이것으로 충분하다.
    if (dt > 0.0f && dt < 0.5f) {
      const float step = steer_rate_ * dt;
      steer_ += std::clamp(c.steering_rad - steer_, -step, step);
    }
    c.steering_rad = steer_;

    if (dt > 0.0f && dt < 0.5f) {
      ego_theta_ += (c.speed_mps / wheelbase_) * std::tan(c.steering_rad) * dt;
      ego_x_ += c.speed_mps * std::cos(ego_theta_) * dt;
      ego_y_ += c.speed_mps * std::sin(ego_theta_) * dt;
    }

    hero_hud::msg::VehicleState out;
    out.header.stamp = get_clock()->now();
    out.header.frame_id = "base_link";
    out.gear = c.gear;
    out.speed_mps = c.speed_mps;
    out.steering_angle_rad = c.steering_rad;
    out.yaw_rate_rps = (c.speed_mps / wheelbase_) * std::tan(c.steering_rad);
    pub_vehicle_->publish(out);

    // 주변광은 천천히 바뀐다. 2 Hz 로 충분하다.
    if (++ambient_tick_ % 25 == 0) {
      hero_hud::msg::AmbientLight al;
      al.header.stamp = get_clock()->now();
      al.relative = ambient_;
      al.source = "manual";
      pub_ambient_->publish(al);
    }

    current_ = c;
  }

  // 월드 → base_link
  geometry_msgs::msg::Point32 toBase(float wx, float wy) const {
    const float dx = wx - ego_x_;
    const float dy = wy - ego_y_;
    const float ct = std::cos(ego_theta_);
    const float st = std::sin(ego_theta_);
    geometry_msgs::msg::Point32 p;
    p.x = dx * ct + dy * st;
    p.y = -dx * st + dy * ct;
    p.z = 0.0f;
    return p;
  }

  // 순수 추종(pure pursuit). **목 차량이 계획을 실제로 따라가게 한다.**
  //
  // 이것이 없으면 대본이 정한 대로만 달리므로 코리도와 목표선이 늘 크게 벌어진
  // 채로 남는다 — 화면이 보여주려는 것이 바로 그 둘의 관계인데, 추종하지 않는
  // 차를 띄우면 매 프레임이 "완전히 어긋난 상태"가 된다.
  //
  // 전방주시거리를 속도에 비례해 잡는다. 짧으면 흔들리고 길면 굽은 데서 안쪽으로
  // 잘라 들어간다.
  float pursue(const hero_hud::msg::TrajectoryRibbon& r, float speed) const {
    if (r.points.size() < 2) {
      return 0.0f;
    }
    // **HUD 가 목표 호를 뽑을 때와 같은 규칙으로 잡아야 한다.** 다르면 목
    // 차량은 다른 곡률을 추종하므로, 완벽히 따라가도 코리도와 목표 호가 영영
    // 겹치지 않는다. 실제로 11 m/s 에서 HUD 8.2 m 대 목 13.2 m 였다.
    //
    // HUD: range = clamp(|v| x 3, 6, 60), look = max(2, range x 0.25)
    const float rng = std::clamp(std::fabs(speed) * 3.0f, 6.0f, 60.0f);
    const float look = std::max(3.5f, rng * 0.25f);
    const auto* target = &r.points.back();
    for (const auto& p : r.points) {
      if (std::hypot(p.x, p.y) >= look) {
        target = &p;
        break;
      }
    }
    const float dsq = target->x * target->x + target->y * target->y;
    if (dsq < 1e-3f) {
      return 0.0f;
    }
    const float kappa = 2.0f * target->y / dsq;
    return std::clamp(std::atan(kappa * wheelbase_), -0.5f, 0.5f);
  }

  // arrive 의 목표 경로. **제어와 발행이 같은 것을 써야 한다** — 따로 만들면
  // 차가 따라가는 길과 화면에 그리는 길이 달라진다.
  hero_hud::msg::TrajectoryRibbon arriveIdeal() const {
    const auto goal = toBase(slot_x_, slot_y_);
    hero_hud::msg::TrajectoryRibbon plan;
    plan.width_m = 1.82f;
    plan.confidence = 0.85f;
    if (goal.x > 25.0f) {
      return laneRibbon();
    }
    // **후진 중에도 목표를 보내야 한다.** 슬롯이 뒤로 넘어갔다고 빈 리본을
    // 보내면 화면에서 목표선이 사라지고, 정작 조향이 가장 어려운 구간에서
    // 사용자가 기댈 것이 없어진다. 자차 뒤로 뻗는 계획을 낸다 — 접선을 뒤로
    // 잡으면 계약("첫 접선이 진행 방향")도 그대로 지켜진다. 후진의 진행
    // 방향은 뒤쪽이다.
    // 슬롯이 자차 옆이나 뒤로 오면 **후진 계획**으로 넘어간다. 기어가 아직 D
    // 여도 그렇게 한다 — 코치는 이미 후진을 권고하고 있고(advised_gear), 화면은
    // "곧 이렇게 들어간다"를 보여주어야 한다. 후진 계획을 안 내면 정작 조향이
    // 가장 어려운 구간에서 목표선이 사라진다.
    //
    // 접선을 뒤로 잡으면 계약("첫 접선이 진행 방향")도 지켜진다 — 후진의 진행
    // 방향은 뒤쪽이다.
    const bool backward = (goal.x < 8.0f);
    const float goal_m = std::hypot(goal.x, goal.y);
    if (goal_m < 0.5f) {
      return plan;
    }
    const float goal_yaw = slot_yaw_ - ego_theta_;
    const float reach = (backward ? -0.6f : 0.6f) * goal_m;
    constexpr int kSteps = 40;
    for (int i = 0; i <= kSteps; ++i) {
      const float u = static_cast<float>(i) / kSteps;
      const float uu = u * u;
      const float uuu = uu * u;
      const float h10 = uuu - 2.0f * uu + u;
      const float h01 = -2.0f * uuu + 3.0f * uu;
      const float h11 = uuu - uu;
      geometry_msgs::msg::Point32 p;
      p.x = h10 * reach + h01 * goal.x + h11 * reach * std::cos(goal_yaw);
      p.y = h01 * goal.y + h11 * reach * std::sin(goal_yaw);
      p.z = 0.0f;
      plan.points.push_back(p);
    }
    return plan;
  }

  // approach 의 목표 경로. **제어와 발행이 같은 것을 써야 한다** — 따로 만들면
  // 차가 따라가는 길과 화면에 그리는 길이 달라지고, 화면이 거짓말을 한다.
  hero_hud::msg::TrajectoryRibbon approachIdeal() const {
    hero_hud::msg::TrajectoryRibbon ribbon;
    ribbon.width_m = 1.82f;
    ribbon.confidence = 0.9f;
    const auto goal = toBase(target_x_, target_y_);
    const float goal_yaw = target_yaw_ - ego_theta_;
    if (goal.x <= 1.0f) {
      return ribbon;
    }
    const float reach = 0.6f * std::hypot(goal.x, goal.y);
    constexpr int kSteps = 40;
    for (int i = 0; i <= kSteps; ++i) {
      const float u = static_cast<float>(i) / kSteps;
      const float uu = u * u;
      const float uuu = uu * u;
      const float h10 = uuu - 2.0f * uu + u;
      const float h01 = -2.0f * uuu + 3.0f * uu;
      const float h11 = uuu - uu;
      geometry_msgs::msg::Point32 p;
      p.x = h10 * reach + h01 * goal.x + h11 * reach * std::cos(goal_yaw);
      p.y = h01 * goal.y + h11 * reach * std::sin(goal_yaw);
      p.z = 0.0f;
      ribbon.points.push_back(p);
    }
    return ribbon;
  }

  // 차선에서 만든 목표 경로. cruise 와 arrive 가 함께 쓴다.
  //
  // 차선을 **그대로** 보내면 안 된다 — 계약이 "자차에서 시작하고 첫 접선이
  // 진행 방향" 이라고 못박아 두었다. 원점이 다르면 코리도와 나란히 놓아도
  // 어긋남이 횡오차인지 헤딩오차인지 갈리지 않는다. smoothstep 으로 붙이면
  // b(0)=0, b'(0)=0 이라 둘 다 만족한다.
  //
  // **지평까지 뻗어야 한다.** 짧게 끊으면 HUD 가 주차 계획으로 오인한다(13-48).
  hero_hud::msg::TrajectoryRibbon laneRibbon() const {
    hero_hud::msg::TrajectoryRibbon lane;
    lane.width_m = 1.82f;
    lane.confidence = 0.9f;
    const float horizon = std::max(20.0f, std::fabs(current_.speed_mps) * 3.5f);
    const float conv = 14.0f;
    constexpr int kN = 48;
    for (int i = 0; i <= kN; ++i) {
      const float s_m = horizon * static_cast<float>(i) / kN;
      const auto p = toBase(ego_x_ + s_m, 0.0f);
      const float u = std::clamp(p.x / conv, 0.0f, 1.0f);
      const float b = u * u * (3.0f - 2.0f * u);
      geometry_msgs::msg::Point32 q;
      q.x = p.x;
      q.y = p.y * b;
      q.z = 0.0f;
      lane.points.push_back(q);
    }
    return lane;
  }

  // 도착 장면. 목표가 **차선에서 슬롯으로** 넘어간다.
  //
  // 이 전환이 표시 어법을 가른다 — 끝점이 12 m 안으로 들어오고 저속이면 HUD 가
  // 주차로 넘어간다(13-48). 상류가 실제로 하는 일도 이것이다: 멀리서는 차선을
  // 따라 리드하고, 슬롯이 잡히면 그리로 가는 계획을 낸다.
  void publishArriveScene(hero_hud::msg::SceneOverlay& out) {
    const auto goal = toBase(slot_x_, slot_y_);
    const auto ideal = arriveIdeal();
    if (ideal.points.size() >= 2) {
      out.ideal = ideal;
      out.has_ideal = true;
    }

    // 슬롯 양옆 주차차량. 코리도 안에 들어와야 정지선이 뜬다.
    for (const auto& o : arrive_obstacles_) {
      const auto p = toBase(o.x, o.y);
      if (p.x < -6.0f || p.x > 40.0f) {
        continue;
      }
      hero_hud::msg::ObstacleMarker m;
      m.position = p;
      m.extent_m = o.extent;
      m.height_m = o.height;
      m.confidence = 0.8f;
      m.source = (p.x >= 0.0f) ? hero_hud::msg::ObstacleMarker::SOURCE_FRONT
                               : hero_hud::msg::ObstacleMarker::SOURCE_REAR;
      const float v = std::max(std::fabs(current_.speed_mps), 0.5f);
      const float ttc = std::hypot(p.x, p.y) / v;
      if (ttc < 2.0f) {
        m.threat = hero_hud::msg::ObstacleMarker::THREAT_CRITICAL;
      } else if (ttc < 3.5f) {
        m.threat = hero_hud::msg::ObstacleMarker::THREAT_WARN;
      } else {
        m.threat = hero_hud::msg::ObstacleMarker::THREAT_CAUTION;
      }
      out.obstacles.push_back(m);
    }

    // 주차장이므로 신호등은 없다. 표지판은 슬롯이 보일 때만.
    out.has_signal = false;
    if (goal.x < 30.0f) {
      out.has_sign = true;
      out.sign.kind = 4;                 // 주차
      out.sign.confidence = 0.8f;
    }

    // 기어 권고. 슬롯 앞에 서면 후진하라고 말한다.
    const double t = phase();
    if (t >= 14.5 && t < 16.5) {
      out.advised_gear = hero_hud::msg::VehicleState::GEAR_REVERSE;
    } else if (t >= 21.0 && t < 23.0) {
      out.advised_gear = hero_hud::msg::VehicleState::GEAR_PARK;
    } else {
      out.advised_gear = hero_hud::msg::VehicleState::GEAR_UNKNOWN;
    }

    hero_hud::msg::SourceHealth h;
    h.name = "wide_camera";
    h.age_sec = 0.05f;
    h.ok = true;
    out.health.push_back(h);

    out.has_coach = true;
    out.coach.text = (goal.x > 25.0f) ? "차선을 유지하세요." : "주차 공간이 있습니다.";
    out.coach.severity = hero_hud::msg::CoachingMessage::SEVERITY_INFO;

    pub_scene_->publish(out);
  }

  // 주행 장면의 나머지 — 선행차, 신호, 표지판.
  //
  // 주차 대본의 장애물은 전부 코리도 밖에 있어서 조향에 따라 스쳤다 말았다
  // 했다(내 비판 3번). 여기서는 **선행차를 차선 안에** 둔다. 거리가 단조롭게
  // 줄어야 감속 그라데이션과 정지선이 설계대로 동작한다.
  void publishCruiseScene(hero_hud::msg::SceneOverlay& out) {
    const double t = phase();

    // 선행차. 월드 고정이 아니라 자차 앞 상대거리로 둔다 — 실제로는 앞차도
    // 달리고 있으므로 접근 속도가 자차 속도와 다르다.
    float gap = 60.0f;
    if (t >= 10.0 && t < 17.0) {
      gap = static_cast<float>(60.0 - (t - 10.0) * 6.43);   // 60 → 15 m
    } else if (t >= 17.0 && t < 23.0) {
      gap = 15.0f;
    }
    if (gap < 55.0f) {
      hero_hud::msg::ObstacleMarker m;
      m.position.x = gap;
      m.position.y = 0.0f;
      m.position.z = 0.0f;
      m.extent_m = 0.90f;
      m.height_m = 1.45f;
      m.confidence = 0.85f;
      m.source = hero_hud::msg::ObstacleMarker::SOURCE_FRONT;
      // 위협은 거리가 아니라 **남은 시간**으로 잡아야 한다. 50 km/h 에서 15 m 는
      // 1.1 초다 — 주차 기준(3 m)을 그대로 쓰면 충돌 0.2 초 전에야 급해진다.
      const float v = std::max(std::fabs(current_.speed_mps), 0.5f);
      const float ttc = gap / v;
      if (ttc < 2.0f) {
        m.threat = hero_hud::msg::ObstacleMarker::THREAT_CRITICAL;
      } else if (ttc < 3.5f) {
        m.threat = hero_hud::msg::ObstacleMarker::THREAT_WARN;
      } else {
        m.threat = hero_hud::msg::ObstacleMarker::THREAT_CAUTION;
      }
      out.obstacles.push_back(m);
    }

    // 신호. 주행에서는 비현실적 픽스처가 아니라 핵심 입력이다.
    out.has_signal = true;
    if (t >= 15.0 && t < 17.5) {
      out.signal.state = hero_hud::msg::SignalIndicator::STATE_YELLOW;
    } else if (t >= 17.5 && t < 23.0) {
      out.signal.state = hero_hud::msg::SignalIndicator::STATE_RED;
    } else {
      out.signal.state = hero_hud::msg::SignalIndicator::STATE_GREEN;
    }
    out.signal.arrow_left = false;
    out.signal.arrow_right = false;

    // 표지판. 속도 제한.
    out.has_sign = true;
    out.sign.kind = 2;
    out.sign.value = 50.0f;
    out.sign.source = hero_hud::msg::SignMarker::SOURCE_WIDE;
    out.sign.confidence = 0.8f;

    out.advised_gear = hero_hud::msg::VehicleState::GEAR_UNKNOWN;

    hero_hud::msg::SourceHealth h;
    h.name = "wide_camera";
    h.age_sec = 0.05f;
    h.ok = true;
    out.health.push_back(h);

    out.has_coach = true;
    if (gap < 55.0f && gap / std::max(std::fabs(current_.speed_mps), 0.5f) < 2.0f) {
      out.coach.text = "앞차와 간격이 좁습니다.";
      out.coach.severity = hero_hud::msg::CoachingMessage::SEVERITY_WARN;
    } else {
      out.coach.text = "차선을 유지하세요.";
      out.coach.severity = hero_hud::msg::CoachingMessage::SEVERITY_INFO;
    }

    pub_scene_->publish(out);
  }

  // ------------------------------------------------------------------ 장면

  void tickScene() {
    const double t = phase();

    // 저하 시연: 인지가 끊겼다가, 돌아왔지만 못 믿을 상태였다가, 회복한다.
    bool suppress = false;
    bool unhealthy = false;
    if (scenario_ == "degraded") {
      if (t >= 8.0 && t < 12.0) {
        suppress = true;    // 아무것도 안 온다 → HUD는 Basic 으로 내려간다
      } else if (t >= 12.0 && t < 16.0) {
        unhealthy = true;   // 오긴 오는데 상류가 못 믿겠다고 한다 → Untrusted
      }
    }
    if (suppress) {
      return;
    }

    hero_hud::msg::SceneOverlay out;
    out.header.stamp = get_clock()->now();
    out.header.frame_id = "base_link";

    // --- 이상궤적 ---
    //
    // 주행에서는 **차선이 목표를 만든다.** 차선 중심선(월드 y=0)을 base_link 로
    // 옮기고, 자차에서 부드럽게 붙인다.
    //
    // 차선을 **그대로** 보내면 안 된다 — 계약이 "자차에서 시작하고 첫 접선이
    // 진행 방향" 이라고 못박아 두었다. 원점이 다르면 코리도와 나란히 놓아도
    // 어긋남이 횡오차인지 헤딩오차인지 갈리지 않는다. 그래서 smoothstep 으로
    // 붙인다 — b(0)=0, b'(0)=0 이라 자차에서 시작하고 접선이 진행 방향이 된다.
    //
    // 그리고 **지평까지 뻗어야 한다.** 짧게 끊어 보내면 HUD 가 주차 계획으로
    // 오인한다(13-48). 차선 인식 거리가 짧아도 외삽해서 채우는 것이 상류의 몫이다.
    if (scenario_ == "arrive") {
      publishArriveScene(out);
      return;
    }
    if (scenario_ == "cruise") {
      const auto lane = laneRibbon();
      out.ideal = lane;
      out.has_ideal = true;
      publishCruiseScene(out);
      return;
    }

    //
    // **지금 자리에서 목표까지의 경로**다. 월드 고정 차선이 아니다.
    // 자차에서 시작하지 않으면 코리도와 비교가 성립하지 않는다 — 원점이 다르면
    // 어긋남이 횡오차인지 헤딩오차인지 갈리지 않는다.
    //
    // 3차 에르미트로 (0,0,헤딩 0) 에서 목표 자세까지 잇는다. 시작 접선이 차량
    // 진행 방향이므로 자차 바로 앞에서는 코리도와 겹치고, 목표로 갈수록 휜다.
    // 실제로는 Hybrid A* 가 이 자리에 들어간다.
    out.has_ideal = false;
    const hero_hud::msg::TrajectoryRibbon ribbon = approachIdeal();
    if (ribbon.points.size() >= 2) {
      out.ideal = ribbon;
      out.has_ideal = true;
    }

    // --- 장애물 ---
    float nearest = 1e9f;
    float nearest_y = 0.0f;
    for (const auto& o : obstacles_) {
      const auto p = toBase(o.x, o.y);
      // 뒤쪽이나 아주 먼 것은 보내지 않는다. 상류가 걸러 주는 것을 흉내낸다.
      if (p.x < -6.0f || p.x > 26.0f) {
        continue;
      }
      hero_hud::msg::ObstacleMarker m;
      m.position = p;
      m.extent_m = o.extent;
      m.height_m = o.height;
      m.confidence = 0.8f;
      m.source = (p.x >= 0.0f) ? hero_hud::msg::ObstacleMarker::SOURCE_FRONT
                               : hero_hud::msg::ObstacleMarker::SOURCE_REAR;

      // 위협 등급은 원래 hero_coach의 판단이다. 여기서는 거리로 흉내낸다.
      const float d = std::hypot(p.x, p.y);
      if (d < 3.0f) {
        m.threat = hero_hud::msg::ObstacleMarker::THREAT_CRITICAL;
      } else if (d < 5.0f) {
        m.threat = hero_hud::msg::ObstacleMarker::THREAT_WARN;
      } else if (d < 9.0f) {
        m.threat = hero_hud::msg::ObstacleMarker::THREAT_CAUTION;
      } else {
        m.threat = hero_hud::msg::ObstacleMarker::THREAT_INFO;
      }
      if (d < nearest) {
        nearest = d;
        nearest_y = p.y;
      }
      out.obstacles.push_back(m);
    }

    // --- 고정 자리 셋 ---
    //
    // 신호등·표지판은 지면에 심볼로 놓지 않는다. 74 x 46 셀에서 읽히지 않고,
    // 화면 가장자리 고정 자리에 두면 위치가 학습되어 흘끗 보기로 읽힌다.
    out.has_signal = true;
    const double cycle = std::fmod(t, 12.0);
    if (cycle < 5.0) {
      out.signal.state = hero_hud::msg::SignalIndicator::STATE_GREEN;
    } else if (cycle < 6.5) {
      out.signal.state = hero_hud::msg::SignalIndicator::STATE_YELLOW;
    } else {
      out.signal.state = hero_hud::msg::SignalIndicator::STATE_RED;
    }
    out.signal.arrow_left = (cycle >= 7.5 && cycle < 11.0);
    out.signal.arrow_right = false;

    out.has_sign = true;
    out.sign.kind = hero_hud::msg::SignMarker::KIND_PARKING;
    out.sign.value = 0.0f;
    out.sign.confidence = 0.8f;
    out.sign.source = hero_hud::msg::SignMarker::SOURCE_WIDE;

    // 좌측: 지금 넣어야 할 기어. 넣을 것이 없으면 표시하지 않는다.
    using VS = hero_hud::msg::VehicleState;
    if (t >= 11.5 && t < 14.0) {
      out.advised_gear = VS::GEAR_REVERSE;   // 곧 후진해야 한다
    } else if (t >= 19.0 && t < 21.0) {
      out.advised_gear = VS::GEAR_PARK;
    } else {
      out.advised_gear = VS::GEAR_UNKNOWN;
    }

    // --- 코칭 문구 ---
    // 거리를 숫자로 말하지 않는다. 상위 문서가 "거리 수치를 말하기엔 이르다"고
    // 적어놓았고, 차 재현율이 0.603이다. docs/HUD_전략.md 3절.
    out.has_coach = true;
    if (current_.gear == hero_hud::msg::VehicleState::GEAR_REVERSE) {
      if (nearest < 3.0f) {
        out.coach.text = "후방 장애물 매우 가까움. 정지하세요.";
        out.coach.severity = hero_hud::msg::CoachingMessage::SEVERITY_CRITICAL;
      } else {
        out.coach.text = "천천히 후진하세요.";
        out.coach.severity = hero_hud::msg::CoachingMessage::SEVERITY_INFO;
      }
    } else if (nearest < 5.0f) {
      out.coach.text = (nearest_y >= 0.0f) ? "좌측 장애물에 접근 중입니다."
                                           : "우측 장애물에 접근 중입니다.";
      out.coach.severity = hero_hud::msg::CoachingMessage::SEVERITY_WARN;
    } else if (nearest < 9.0f) {
      out.coach.text = "전방에 장애물이 있습니다.";
      out.coach.severity = hero_hud::msg::CoachingMessage::SEVERITY_CAUTION;
    } else {
      out.coach.text = "주행 경로가 비어 있습니다.";
      out.coach.severity = hero_hud::msg::CoachingMessage::SEVERITY_INFO;
    }
    out.coach.ttl_sec = 1.0f;

    // --- 소스 건강도 ---
    const char* names[] = {"front_camera", "rear_camera", "wide_camera",
                           "imu", "can", "odometry", "occupancy"};
    for (const char* n : names) {
      hero_hud::msg::SourceHealth h;
      h.name = n;
      h.age_sec = 0.02f;
      h.ok = !(unhealthy && (std::string(n) == "occupancy" || std::string(n) == "odometry"));
      out.health.push_back(h);
    }

    pub_scene_->publish(out);
  }

  // ------------------------------------------------------------------ 상태

  std::string scenario_;
  float wheelbase_{2.700f};

  // 실제 조향과 그 변화 속도. 대본은 목표만 내고 여기서 따라간다.
  float steer_{0.0f};
  float steer_rate_{0.55f};   // rad/s (조향륜). 사람이 핸들을 돌리는 속도쯤.
  double loop_sec_{24.0};
  float ambient_{0.85f};
  int ambient_tick_{0};

  std::vector<WorldObject> obstacles_;
  float target_x_{0.0f};
  float target_y_{0.0f};
  float target_yaw_{0.0f};

  // arrive 전용. 자차가 t=15 s 무렵 도달하는 자리에 슬롯을 둔다.
  float slot_x_{104.0f};
  float slot_y_{-3.2f};
  float slot_yaw_{-1.35f};
  std::vector<WorldObject> arrive_obstacles_;

  float ego_x_{0.0f};
  float ego_y_{0.0f};
  float ego_theta_{0.0f};
  Command current_;

  double start_{0.0};
  double last_tick_{0.0};
  double last_phase_{0.0};

  rclcpp::Publisher<hero_hud::msg::VehicleState>::SharedPtr pub_vehicle_;
  rclcpp::Publisher<hero_hud::msg::SceneOverlay>::SharedPtr pub_scene_;
  rclcpp::Publisher<hero_hud::msg::AmbientLight>::SharedPtr pub_ambient_;
  rclcpp::TimerBase::SharedPtr timer_vehicle_;
  rclcpp::TimerBase::SharedPtr timer_scene_;
};

}  // namespace

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ScenarioPublisher>());
  rclcpp::shutdown();
  return 0;
}
