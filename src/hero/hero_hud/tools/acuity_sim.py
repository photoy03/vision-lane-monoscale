#!/usr/bin/env python3
"""HUD 화면이 **실제 조건에서** 읽히는지 본다.

    tools/acuity_sim.py <입력.png> <출력.png> --hfov 12.3 --acuity 0.1 \
        --bg daylight --csf 0.5 --cutoff 3.0 --pair

렌더러는 `include/hero_hud/angular.hpp` 의 예산대로 그린다. 이 도구는 그 예산이
실제로 맞았는지를 눈으로 확인한다 — 계산이 "읽힌다"고 해도 구도가 과밀하거나
배경이 밝으면 안 읽히고, 그것은 숫자로 안 나온다.

모형은 세 단계다. **순서가 물리 순서와 같아야 한다.**

  1) 합성   가산광 컴바이너는 실세계 위에 빛을 더한다.  out = clip(배경 + HUD)
  2) 흐림   눈의 저역통과. 차단주파수에서 sigma 를 얻는다
  3) 대비   대비감도 저하. 평균 주위로 진폭을 줄인다

이전 판은 1번과 3번이 통째로 빠져 있었고, 2번의 sigma 도 낙관적이었다:

  MAR 10 arcmin 을 FWHM 으로 → sigma 4.25 arcmin ≈ 차단 4.8 cyc/deg
  저시력 대비감도 차단 3 cyc/deg → sigma 6.83 arcmin

즉 **검은 배경에 60% 덜 흐리게** 보고 있었다. 가장 쉬운 조건만 재고 있었던 것이다.

배경은 합성이다. 실제 CARLA 프레임이나 실차 영상이 생기면 `--bg <파일>` 로 넣는
편이 낫다. 여기 값들은 "밝은 낮 노면"의 대략적인 휘도 분포일 뿐이다.

여전히 안 들어간 것: 중심암점·시야결손(병인 미확인), 눈부심 산란, 컴바이너의
실제 최대 휘도와 투과율. 셋 다 상황을 나쁘게만 만든다.
"""

import argparse
import math
import os

import numpy as np
from PIL import Image, ImageFilter

ARCMIN_PER_RADIAN = 3437.7468


def arcmin_per_pixel(hfov_deg: float, width_px: int) -> float:
    """중심 화소의 각크기. `include/hero_hud/angular.hpp` 와 같은 식이어야 한다."""
    fx = (width_px * 0.5) / math.tan(math.radians(hfov_deg) * 0.5)
    return ARCMIN_PER_RADIAN / fx


def sigma_arcmin(cutoff_cyc_per_deg: float) -> float:
    """대비감도 차단주파수에서 가우시안 sigma(arcmin).

    가우시안 MTF = exp(-2 pi^2 sigma^2 f^2). MTF 가 0.1 로 떨어지는 f 를
    차단으로 본다.
    """
    sigma_deg = math.sqrt(2.303 / (2.0 * math.pi ** 2)) / max(cutoff_cyc_per_deg, 0.1)
    return sigma_deg * 60.0


def make_background(kind: str, w: int, h: int) -> np.ndarray:
    """가산광 뒤로 비치는 실세계. 합성이다."""
    if kind == "none":
        return np.zeros((h, w, 3), dtype=np.float32)
    if os.path.exists(kind):
        img = Image.open(kind).convert("RGB").resize((w, h))
        return np.asarray(img, dtype=np.float32)

    # 위쪽은 하늘/먼 배경으로 밝고, 아래로 갈수록 노면이다.
    top, bottom, grain = {
        "night":    (28.0, 12.0, 4.0),
        "overcast": (170.0, 95.0, 10.0),
        "daylight": (245.0, 165.0, 16.0),
    }.get(kind, (170.0, 95.0, 10.0))

    ramp = np.linspace(top, bottom, h, dtype=np.float32)[:, None]
    base = np.repeat(ramp, w, axis=1)
    rng = np.random.default_rng(7)   # 결정적 — 같은 입력이면 같은 그림
    base = base + rng.normal(0.0, grain, size=(h, w)).astype(np.float32)
    return np.clip(np.stack([base, base, base * 0.97], axis=2), 0.0, 255.0)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("src")
    ap.add_argument("dst")
    ap.add_argument("--hfov", type=float, required=True, help="가상상의 수평 화각(도)")
    ap.add_argument("--acuity", type=float, default=0.1, help="교정시력. 조아영님 0.1")
    ap.add_argument("--cutoff", type=float, default=0.0,
                    help="대비감도 차단(cyc/deg). 0이면 시력의 MAR 에서 유도한다 "
                         "(낙관적). 저시력이면 3.0 을 권한다")
    ap.add_argument("--bg", default="none",
                    help="none | night | overcast | daylight | <이미지 파일>")
    ap.add_argument("--csf", type=float, default=1.0,
                    help="대비감도 계수 0..1. 1이면 손실 없음, 0.5면 진폭 절반")
    ap.add_argument("--pair", action="store_true",
                    help="원본과 결과를 나란히. 결과만 보면 무엇이 사라졌는지 "
                         "알 수 없고, 원본만 보면 무엇이 안 보이는지 알 수 없다")
    args = ap.parse_args()

    hud = Image.open(args.src).convert("RGB")
    w, h = hud.size
    ampx = arcmin_per_pixel(args.hfov, w)

    detail_arcmin = 1.0 / max(args.acuity, 0.001)
    if args.cutoff > 0.0:
        sig_arcmin = sigma_arcmin(args.cutoff)
        basis = f"차단 {args.cutoff:.1f} cyc/deg"
    else:
        sig_arcmin = detail_arcmin / 2.355   # MAR 을 FWHM 으로 — 낙관적
        basis = "MAR 을 FWHM 으로 (낙관적)"
    sigma_px = sig_arcmin / ampx

    # 1) 가산 합성 — 컴바이너는 실세계 위에 빛을 더한다.
    bg = make_background(args.bg, w, h)
    seen = np.clip(bg + np.asarray(hud, dtype=np.float32), 0.0, 255.0)

    # 2) 눈의 저역통과.
    img = Image.fromarray(seen.astype(np.uint8))
    if sigma_px > 0.3:
        img = img.filter(ImageFilter.GaussianBlur(radius=sigma_px))

    # 3) 대비감도 저하 — 평균 주위로 진폭을 줄인다.
    if args.csf < 0.999:
        arr = np.asarray(img, dtype=np.float32)
        mean = float(arr.mean())
        arr = mean + (arr - mean) * args.csf
        img = Image.fromarray(np.clip(arr, 0, 255).astype(np.uint8))

    detail_px = detail_arcmin / ampx
    print(f"화각 {args.hfov:.1f}도 · {w}x{h} · 시력 {args.acuity:.2f}")
    print(f"  픽셀당        {ampx:.3f} arcmin")
    print(f"  분해 한계     {detail_arcmin:.0f} arcmin = {detail_px:.1f} px")
    print(f"  흐림 sigma    {sig_arcmin:.2f} arcmin = {sigma_px:.1f} px   [{basis}]")
    print(f"  배경          {args.bg}   평균 휘도 {bg.mean():.0f}")
    print(f"  대비감도      {args.csf:.2f}")

    if args.pair:
        gap = 8
        sheet = Image.new("RGB", (w * 2 + gap, h), (40, 40, 40))
        sheet.paste(hud, (0, 0))
        sheet.paste(img, (w + gap, 0))
        sheet.save(args.dst)
    else:
        img.save(args.dst)
    print(f"저장: {args.dst}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
