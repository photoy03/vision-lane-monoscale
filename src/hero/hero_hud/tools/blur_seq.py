#!/usr/bin/env python3
"""프레임 묶음을 한 번에 저시력 시야로 바꾼다.

    tools/blur_seq.py <입력디렉터리> <출력디렉터리> --hfov 12.3 --bg overcast

`acuity_sim.py` 의 함수를 그대로 import 한다. 흐림 모형을 여기 다시 쓰지 않는
것이 이 파일의 요점이다 — 이 프로젝트는 그 모형이 한 번 틀렸던 이력이 있고
(합성과 대비감도가 통째로 빠져 있었다), 구현이 두 벌이면 한쪽만 고쳐진다.
ffmpeg 의 gblur 로 대신하지 않는 이유도 같다.

배경은 프레임마다 같으므로 한 번만 만든다. 480장을 도는 데 이것이 절반이다.
"""

import argparse
import os
import sys

import numpy as np
from PIL import Image, ImageFilter

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from acuity_sim import arcmin_per_pixel, make_background, sigma_arcmin  # noqa: E402


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("src_dir")
    ap.add_argument("dst_dir")
    ap.add_argument("--hfov", type=float, required=True)
    ap.add_argument("--acuity", type=float, default=0.1)
    ap.add_argument("--cutoff", type=float, default=3.0)
    ap.add_argument("--bg", default="overcast")
    ap.add_argument("--csf", type=float, default=0.6)
    args = ap.parse_args()

    frames = sorted(f for f in os.listdir(args.src_dir) if f.endswith(".png"))
    if not frames:
        print(f"프레임이 없다: {args.src_dir}", file=sys.stderr)
        return 1
    os.makedirs(args.dst_dir, exist_ok=True)

    first = Image.open(os.path.join(args.src_dir, frames[0]))
    w, h = first.size
    ampx = arcmin_per_pixel(args.hfov, w)

    detail_arcmin = 1.0 / max(args.acuity, 0.001)
    if args.cutoff > 0.0:
        sig_arcmin = sigma_arcmin(args.cutoff)
    else:
        sig_arcmin = detail_arcmin / 2.355
    sigma_px = sig_arcmin / ampx

    bg = make_background(args.bg, w, h)   # 프레임마다 같다. 한 번만.

    print(f"{len(frames)}장 · {w}x{h} · 화각 {args.hfov:.1f}도 · 시력 {args.acuity:.2f}")
    print(f"  흐림 sigma {sig_arcmin:.2f} arcmin = {sigma_px:.1f} px")
    print(f"  배경 {args.bg} 평균 휘도 {bg.mean():.0f} · 대비감도 {args.csf:.2f}")

    for i, name in enumerate(frames):
        hud = Image.open(os.path.join(args.src_dir, name)).convert("RGB")

        # 물리 순서 — 합성(가산) → 흐림 → 대비감도. acuity_sim.py 와 같아야 한다.
        seen = np.clip(bg + np.asarray(hud, dtype=np.float32), 0.0, 255.0)
        img = Image.fromarray(seen.astype(np.uint8))
        if sigma_px > 0.3:
            img = img.filter(ImageFilter.GaussianBlur(radius=sigma_px))
        if args.csf < 0.999:
            arr = np.asarray(img, dtype=np.float32)
            mean = float(arr.mean())
            img = Image.fromarray(
                np.clip(mean + (arr - mean) * args.csf, 0, 255).astype(np.uint8))

        img.save(os.path.join(args.dst_dir, name))
        if (i + 1) % 60 == 0:
            print(f"  {i + 1}/{len(frames)}")

    print(f"저장: {args.dst_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
