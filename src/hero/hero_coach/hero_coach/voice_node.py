"""말로 하는 안내. "전진" "후진" "정지".

저시력 사용자 피드백으로 HUD 에서 글자를 걷어냈다. 기어와 정지는 화면의 도형과
이 노드가 함께 나른다. 화면이 못 하는 일이 하나 있다 — **보고 있지 않아도 닿는다.**

문구는 미리 WAV 로 굽는다. piper 합성은 한 마디에 수백 ms 가 걸리고, 그것을 말할
때마다 치르면 "정지"가 가장 급할 때 가장 늦는다. 구운 파일은 캐시에 남아 다음
실행에서는 굽지 않는다.

모델이나 piper 가 없으면 한 번 경고하고 조용히 지낸다. 코치의 다른 출력은 그대로
나가야 한다 — 음성은 거들 뿐이고, 없다고 화면을 멈출 이유가 없다.

    tools/fetch_voice.sh    # piper 와 한국어 모델
"""

import hashlib
import os
from pathlib import Path
import shutil
import subprocess
import threading
import time

from hero_hud.msg import SceneOverlay, VehicleState
import rclpy
from rclpy.node import Node

from hero_coach.core import VOICE_PHRASES, VOICE_STOP, VoicePolicy, travel_blocked
from hero_coach.ros_io import LATEST


class VoiceNode(Node):
    def __init__(self):
        super().__init__('voice_node')
        declare = self.declare_parameter
        declare('scene_topic', '/hero/scene_overlay')
        declare('vehicle_topic', '/hero/vehicle_state')
        declare('model', str(Path.home() / '.local/share/piper-voices/ko_KR-kss-medium.onnx'))
        declare('cache_dir', str(Path.home() / '.cache/hero_coach/voice'))
        declare('player', 'paplay')
        declare('min_gap_sec', 1.2)
        declare('repeat_sec', 3.0)
        declare('critical_threat', 3)
        self.p = lambda name: self.get_parameter(name).value  # noqa: E731

        self.policy = VoicePolicy(float(self.p('min_gap_sec')), float(self.p('repeat_sec')))
        self.gear = 0
        self.speaking = threading.Lock()
        self.clips = self.bake()

        self.create_subscription(
            VehicleState, str(self.p('vehicle_topic')), self.on_vehicle, LATEST)
        self.create_subscription(
            SceneOverlay, str(self.p('scene_topic')), self.on_scene, LATEST)

    # ------------------------------------------------------------ 준비
    def bake(self):
        """문구마다 WAV 하나. 이미 있으면 그대로 쓴다."""
        model = Path(str(self.p('model')))
        cache = Path(str(self.p('cache_dir')))
        phrases = sorted(set(VOICE_PHRASES.values()) | {VOICE_STOP})
        clips = {}
        if shutil.which(str(self.p('player'))) is None:
            self.get_logger().warn(
                f"재생기 '{self.p('player')}' 가 없다. 음성 없이 돈다.")
            return clips
        cache.mkdir(parents=True, exist_ok=True)
        for phrase in phrases:
            # 모델이 바뀌면 다시 굽는다 — 이름에 모델을 넣어 구분한다.
            key = hashlib.sha1(f'{model.name}|{phrase}'.encode()).hexdigest()[:12]
            wav = cache / f'{key}.wav'
            if wav.exists() and wav.stat().st_size > 0:
                clips[phrase] = wav
                continue
            if not model.exists():
                self.get_logger().warn(
                    f'piper 모델이 없다: {model} — tools/fetch_voice.sh. 음성 없이 돈다.')
                return clips
            try:
                subprocess.run(
                    ['python3', '-m', 'piper', '-m', str(model), '-f', str(wav)],
                    input=phrase.encode(), check=True, capture_output=True, timeout=120)
            except (OSError, subprocess.SubprocessError) as error:
                self.get_logger().warn(f'piper 합성 실패({phrase}): {error}. 음성 없이 돈다.')
                return {}
            clips[phrase] = wav
        self.get_logger().info(f'문구 {len(clips)}개 준비: {", ".join(phrases)}')
        return clips

    # ------------------------------------------------------------ 입력
    def on_vehicle(self, msg: VehicleState):
        self.gear = msg.gear

    def on_scene(self, msg: SceneOverlay):
        blocked = travel_blocked(
            [(o.position.x, o.threat) for o in msg.obstacles], self.gear,
            int(self.p('critical_threat')))
        phrase = self.policy.decide(time.monotonic(), msg.advised_gear, blocked)
        if phrase is not None:
            self.say(phrase)

    # ------------------------------------------------------------ 출력
    def say(self, phrase: str):
        clip = self.clips.get(phrase)
        if clip is None:
            return
        # 겹쳐 말하지 않는다. 말이 겹치면 둘 다 안 들린다.
        if not self.speaking.acquire(blocking=False):
            return
        self.get_logger().info(f'"{phrase}"')

        def play():
            try:
                subprocess.run([str(self.p('player')), str(clip)],
                               check=False, capture_output=True, timeout=10)
            except (OSError, subprocess.SubprocessError) as error:
                self.get_logger().warn(f'재생 실패: {error}')
            finally:
                self.speaking.release()

        threading.Thread(target=play, daemon=True).start()


def main():
    # 스크린리더 등이 쓰는 환경변수와 섞이지 않게, 재생은 파라미터의 재생기로만 한다.
    os.environ.setdefault('PULSE_PROP', 'media.role=event')
    rclpy.init()
    node = VoiceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
