// HERO HUD 렌더러 진입점.
//
// 프로세스 하나, 스레드 둘.
//   - ROS 실행기 스레드: 구독만 한다. 렌더링을 건드리지 않는다.
//   - Qt 렌더 스레드: vsync마다 HudState의 최신 스냅샷을 그린다.
// 경계는 HudState 하나뿐이다. docs/HUD_전략.md 7절.
//
// Orin에서는 데스크톱 환경 없이 DRM/KMS로 직행한다:
//   QT_QPA_PLATFORM=eglfs ros2 run hero_hud hud_node
#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <string>

#include <QColor>
#include <QElapsedTimer>
#include <QGuiApplication>
#include <QQuickWindow>
#include <QScreen>
#include <QSurfaceFormat>
#include <QTimer>

#include <rclcpp/rclcpp.hpp>

#include "hero_hud/chrome_item.hpp"
#include "hero_hud/ros_bridge.hpp"
#include "hero_hud/world_item.hpp"

namespace {

int envInt(const char* name, int fallback) {
  const char* raw = std::getenv(name);
  if (raw == nullptr) {
    return fallback;
  }
  try {
    return std::stoi(raw);
  } catch (...) {
    return fallback;
  }
}

}  // namespace

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  // 다중샘플링. 코리도는 채움과 테두리를 **따로** 그린 두 개의 삼각형 띠이고,
  // 둘이 맞닿는 경계가 곧 이 화면에서 가장 긴 선이다. 앨리어싱이 있으면 그
  // 경계가 표본마다 계단으로 튀어, 채움과 선이 어긋난 것처럼 보인다 —
  // 실제로 어긋난 것이 아니라 같은 곡선을 두 번 래스터화한 결과다.
  //
  // 창을 만들기 전에 기본 포맷으로 넣어야 씬그래프가 집어간다.
  QSurfaceFormat fmt = QSurfaceFormat::defaultFormat();
  fmt.setSamples(4);
  QSurfaceFormat::setDefaultFormat(fmt);

  QGuiApplication app(argc, argv);

  hero_hud::HudState state;
  hero_hud::RosBridge bridge(&state);
  const hero_hud::HudConfig config = bridge.config();
  bridge.start();

  QQuickWindow window;
  window.setTitle(QStringLiteral("HERO HUD"));
  // 정확히 검정. 컴바이너 가산광에서 이것이 투명이다. 짙은 회색으로 두면
  // 흐린 사각형이 전방 시야 위에 뜬다. docs/HUD_전략.md 4절.
  window.setColor(QColor(0, 0, 0));
  window.resize(config.display.width_px, config.display.height_px);

  auto* world = new hero_hud::WorldItem(window.contentItem());
  world->configure(&state, config);

  auto* chrome = new hero_hud::ChromeItem(window.contentItem());
  chrome->configure(&state, config);
  if (config.road_lane_only || config.parking_plan_view) chrome->setVisible(false);

  const auto fit = [&window, world, chrome, &config]() {
    const double w = window.width();
    const double h = window.height();
    // 월드는 화면 전체를 쓴다.
    //
    // 이전 판은 고정 자리만큼 인셋을 잡았는데, 코리도는 원래 가운데 절반만
    // 쓰므로 양옆 25%씩은 **자동으로 비어 있다.** 인셋은 공짜로 얻을 자리를
    // 돈 주고 산 셈이었고, 코리도가 64% 폭에 갇혀 주인공이 작아졌다.
    // 위쪽은 투영의 top_margin 이 이미 비워 둔다.
    world->setX(0);
    world->setY(0);
    world->setWidth(w);
    world->setHeight(h);
    chrome->setWidth(w);
    chrome->setHeight(h);
  };
  fit();
  QObject::connect(&window, &QWindow::widthChanged, &window, fit);
  QObject::connect(&window, &QWindow::heightChanged, &window, fit);

  // 렌더는 ROS를 기다리지 않는다. 데이터가 안 와도 계속 돌면서 저하 상태를
  // 그린다 — 화면이 멈추는 것 자체가 사용자에게는 오해를 부르는 신호다.
  QTimer render_timer;
  QObject::connect(&render_timer, &QTimer::timeout, &app, [&]() {
    if (!rclcpp::ok()) {
      QGuiApplication::quit();
      return;
    }
    world->update();
    chrome->update();
  });
  render_timer.start(16);

  // 켜진 면적을 주기적으로 남긴다. 가산광에서 이것은 그대로 전방 시야를 가리는
  // 양이다. 덩어리는 흐림에 강하지만 공짜가 아니고, 이 숫자가 그 대가를 잰다.
  QTimer lit_timer;
  QObject::connect(&lit_timer, &QTimer::timeout, &app, [world, &bridge]() {
    RCLCPP_INFO(bridge.node()->get_logger(), "켜진 면적 %.1f%%",
                100.0 * world->litFraction());
  });
  lit_timer.start(3000);

  // 화면 선택과 전체화면. Orin 은 eglfs 로 단일 화면에 직행하지만, 데스크톱
  // 에서 시연하거나 확장 모니터로 보낼 때가 있다.
  //
  //   HERO_HUD_SCREEN=HDMI-1-0  HERO_HUD_FULLSCREEN=1
  //
  // 크기가 바뀌어도 설계는 따라온다 — 화각은 패널의 물리 치수와 광로에서
  // 나오고 각도 예산은 그 화각에서 픽셀을 유도하므로, 해상도가 달라지면
  // 모든 표시 크기가 같은 비율로 다시 잡힌다.
  if (const char* want = std::getenv("HERO_HUD_SCREEN")) {
    const QString name = QString::fromUtf8(want);
    QScreen* chosen = nullptr;
    for (QScreen* s : QGuiApplication::screens()) {
      if (s->name() == name) {
        chosen = s;
        break;
      }
    }
    if (chosen == nullptr) {
      QStringList names;
      for (QScreen* s : QGuiApplication::screens()) {
        names << s->name();
      }
      qWarning("화면 '%s' 을 못 찾았다. 있는 것: %s", want,
               qPrintable(names.join(QStringLiteral(", "))));
    } else {
      window.setScreen(chosen);
      window.setGeometry(chosen->geometry());
      qInfo("화면 %s (%dx%d @ %d,%d)", qPrintable(chosen->name()),
            chosen->geometry().width(), chosen->geometry().height(),
            chosen->geometry().x(), chosen->geometry().y());
    }
  }
  if (std::getenv("HERO_HUD_FULLSCREEN") != nullptr) {
    window.showFullScreen();
  } else {
    window.show();
  }

  // 회귀 확인용 스크린샷. 표시 결과를 눈으로 보지 않고 확인할 수단이 없으면
  // 시각 변경이 조용히 깨진다.
  const char* shot_path = std::getenv("HERO_HUD_SCREENSHOT");
  if (shot_path != nullptr) {
    const int delay_ms = envInt("HERO_HUD_SCREENSHOT_DELAY_MS", 3000);
    const std::string path(shot_path);
    QTimer::singleShot(delay_ms, &app, [&window, path]() {
      const QImage shot = window.grabWindow();
      if (shot.isNull() || !shot.save(QString::fromStdString(path))) {
        qWarning("스크린샷 저장 실패: %s", path.c_str());
      } else {
        qInfo("스크린샷 저장: %s (%dx%d)", path.c_str(), shot.width(), shot.height());
      }
      QGuiApplication::quit();
    });
  }

  // 연속 프레임 기록. 정지 화면으로는 이 HUD의 시간 채널 셋이 통째로 안 보인다 —
  // 점멸(황색 신호·대기 기어), 감속 그라데이션이 코리도 끝에서 앞으로 번지는 진행,
  // 기어 전환의 순간. 각도 예산은 시간 채널이 "시력과 무관하게 흐림을 통과하는
  // 유일한 채널"이라고 적어놓았는데, 그 주장은 정지 화면으로는 검증할 수 없다.
  //
  // 프레임 시각을 벽시계로 재서 남긴다. grabWindow() 는 GPU 동기 읽기라
  // 요청한 fps 를 못 지킬 수 있고, 그때 요청 fps 로 인코딩하면 영상이 실제보다
  // 빨라진다 — 점멸 주기를 재는 용도에서 그것은 그냥 틀린 결과다.
  const char* rec_dir = std::getenv("HERO_HUD_RECORD");
  QTimer rec_timer;
  QElapsedTimer rec_clock;
  int rec_index = 0;
  if (rec_dir != nullptr) {
    const int fps = std::max(1, envInt("HERO_HUD_RECORD_FPS", 20));
    const int seconds = std::max(1, envInt("HERO_HUD_RECORD_SECONDS", 24));
    const int settle_ms = envInt("HERO_HUD_SCREENSHOT_DELAY_MS", 1500);
    const int total = fps * seconds;
    const std::string dir(rec_dir);

    QObject::connect(&rec_timer, &QTimer::timeout, &app, [&, dir, total]() {
      const QImage frame = window.grabWindow();
      char name[32];
      std::snprintf(name, sizeof(name), "/frame_%05d.png", rec_index);
      if (frame.isNull() || !frame.save(QString::fromStdString(dir + name))) {
        qWarning("프레임 저장 실패: %d", rec_index);
        QGuiApplication::quit();
        return;
      }
      if (++rec_index >= total) {
        const double elapsed = rec_clock.elapsed() / 1000.0;
        rec_timer.stop();
        // 이 줄을 record.sh 가 읽어 실제 fps 로 인코딩한다.
        qInfo("기록 완료: %d 프레임 %.3f 초 실측 %.3f fps", rec_index, elapsed,
              rec_index / std::max(elapsed, 1e-6));
        QGuiApplication::quit();
      }
    });
    QTimer::singleShot(settle_ms, &app, [&, fps]() {
      rec_clock.start();
      rec_timer.start(1000 / fps);
    });
  }

  const int rc = app.exec();

  bridge.stop();
  rclcpp::shutdown();
  return rc;
}
