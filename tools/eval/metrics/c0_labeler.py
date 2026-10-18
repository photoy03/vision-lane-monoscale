#!/usr/bin/env python3
"""Minimal labeller for the C-0 frames (labelme-compatible JSON beside the images).

Usage: python3 tools/eval/metrics/c0_labeler.py <label folder> [frame_id,frame_id,...]
       (folder made by c0_samples.py; the optional list limits the session to those frames)
Guide: docs in the label folder, LABEL_GUIDE_KO.md.

Keys
  1 lane_line (line)   2 obstacle   3 dynamic   4 unjudgeable   5 marking_other
  left click    add a point            right click / Enter   finish the shape
  u             undo (point, else last shape)
  x             delete the shape nearest the mouse (within 30 px)
  z             zoom x2 around the mouse (again: back)
  d             mark this frame done (only done frames are evaluated)
  n / p         next / previous frame   (also saves)      q   quit
Every change is saved at once. Only the raw frame is shown: no map output.
"""
import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

import cv2
import numpy as np

KINDS = {ord("1"): ("lane_line", "linestrip", (0, 0, 255)),
         ord("2"): ("obstacle", "polygon", (0, 160, 255)),
         ord("3"): ("dynamic", "polygon", (255, 0, 255)),
         ord("4"): ("unjudgeable", "polygon", (160, 160, 160)),
         ord("5"): ("marking_other", "polygon", (255, 200, 0))}
COLORS = {name: color for name, _, color in KINDS.values()}
WINDOW = "c0 labeler"


class Labeler:
    def __init__(self, folder: Path, only=None):
        self.folder = folder
        self.samples = json.loads((folder / "samples.json").read_text(encoding="utf-8"))
        self.frames = [f for f in self.samples["frames"] if not only or f["id"] in only]
        if not self.frames:
            raise SystemExit("no matching frames")
        self.index = next((i for i, f in enumerate(self.frames) if not self.load(f).get("done")), 0)
        self.kind = KINDS[ord("1")]
        self.points = []
        self.zoom = None          # (x0, y0) of the x2 view, image pixels
        self.mouse = (0, 0)
        self.guides = self.ground_guides()
        self.open_frame()

    def ground_guides(self):
        """Pixel polylines of the ground lines 1, 4 and 8 m ahead (geometry only, no map)."""
        import map_accuracy
        camera = map_accuracy.load_camera(self.samples["camera"])
        guides = []
        for distance in (1.0, 4.0, 8.0):
            points = [camera.base_to_pixel(distance, y / 10.0) for y in range(-60, 61)]
            guides.append((distance, np.array([p for p in points if p is not None], np.int32)))
        return guides

    def path(self, frame):
        return self.folder / f"{frame['id']}.json"

    def load(self, frame):
        p = self.path(frame)
        if p.exists():
            return json.loads(p.read_text(encoding="utf-8"))
        return {"version": "c0-1", "shapes": [], "imagePath": frame["image"], "done": False,
                "imageWidth": self.samples["camera"]["width"], "imageHeight": self.samples["camera"]["height"]}

    def save(self):
        tmp = self.path(self.frame).with_suffix(".tmp")
        tmp.write_text(json.dumps(self.doc, indent=1), encoding="utf-8")
        tmp.replace(self.path(self.frame))

    def open_frame(self):
        self.frame = self.frames[self.index]
        self.image = cv2.imread(str(self.folder / self.frame["image"]))
        self.doc = self.load(self.frame)
        self.points = []
        self.zoom = None

    def to_image(self, x, y):
        if self.zoom is None:
            return float(x), float(y)
        return self.zoom[0] + x / 2.0, self.zoom[1] + y / 2.0

    def on_mouse(self, event, x, y, *_):
        self.mouse = (x, y)
        if event == cv2.EVENT_LBUTTONDOWN:
            self.points.append(list(self.to_image(x, y)))
        elif event == cv2.EVENT_RBUTTONDOWN:
            self.finish()

    def finish(self):
        name, shape_type, _ = self.kind
        need = 2 if shape_type == "linestrip" else 3
        if len(self.points) >= need:
            self.doc["shapes"].append({"label": name, "shape_type": shape_type, "points": self.points})
            self.save()
        self.points = []

    def delete_nearest(self):
        mx, my = self.to_image(*self.mouse)
        best, best_d = None, 30.0 if self.zoom is None else 15.0
        for k, shape in enumerate(self.doc["shapes"]):
            pts = shape["points"] + ([shape["points"][0]] if shape["shape_type"] == "polygon" else [])
            for (x0, y0), (x1, y1) in zip(pts, pts[1:]):
                dx, dy = x1 - x0, y1 - y0
                n = dx * dx + dy * dy
                t = 0.0 if n < 1e-9 else max(0.0, min(1.0, ((mx - x0) * dx + (my - y0) * dy) / n))
                d = ((mx - x0 - t * dx) ** 2 + (my - y0 - t * dy) ** 2) ** 0.5
                if d < best_d:
                    best, best_d = k, d
        if best is not None:
            self.doc["shapes"].pop(best)
            self.save()

    def draw(self):
        canvas = self.image.copy()
        for distance, pts in self.guides:
            cv2.polylines(canvas, [pts], False, (0, 220, 255), 1)
            if len(pts):
                cv2.putText(canvas, f"{distance:.0f} m", (int(pts[len(pts) // 2][0]) + 6, int(pts[len(pts) // 2][1]) - 4),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 220, 255), 1)
        for shape in self.doc["shapes"]:
            pts = np.array(shape["points"], np.int32)
            color = COLORS.get(shape["label"], (255, 255, 255))
            if shape["shape_type"] == "polygon":
                overlay = canvas.copy()
                cv2.fillPoly(overlay, [pts], color)
                canvas = cv2.addWeighted(overlay, 0.25, canvas, 0.75, 0)
                cv2.polylines(canvas, [pts], True, color, 2)
            else:
                cv2.polylines(canvas, [pts], False, color, 3)
        if self.points:
            cv2.polylines(canvas, [np.array(self.points, np.int32)], False, self.kind[2], 2)
            for p in self.points:
                cv2.circle(canvas, (int(p[0]), int(p[1])), 4, self.kind[2], -1)
        if self.zoom is not None:
            h, w = canvas.shape[:2]
            x0, y0 = int(self.zoom[0]), int(self.zoom[1])
            canvas = cv2.resize(canvas[y0:y0 + h // 2, x0:x0 + w // 2], (w, h), interpolation=cv2.INTER_NEAREST)
        done = "DONE" if self.doc.get("done") else "-"
        text = (f"{self.index + 1}/{len(self.frames)} {self.frame['id']} [{done}]  mode: {self.kind[0]}"
                f"  shapes: {len(self.doc['shapes'])}  (1-5 mode, u undo, x delete near mouse, z zoom, d done, n/p, q)")
        cv2.rectangle(canvas, (0, 0), (canvas.shape[1], 28), (0, 0, 0), -1)
        cv2.putText(canvas, text, (8, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1)
        cv2.imshow(WINDOW, canvas)

    def run(self):
        cv2.namedWindow(WINDOW, cv2.WINDOW_AUTOSIZE)
        cv2.setMouseCallback(WINDOW, self.on_mouse)
        while True:
            self.draw()
            key = cv2.waitKey(30) & 0xFF
            if key == 255:
                if cv2.getWindowProperty(WINDOW, cv2.WND_PROP_VISIBLE) < 1:
                    break
                continue
            if key in KINDS:
                self.finish()
                self.kind = KINDS[key]
            elif key in (13, 10):
                self.finish()
            elif key == ord("u"):
                if self.points:
                    self.points.pop()
                elif self.doc["shapes"]:
                    self.doc["shapes"].pop()
                    self.save()
            elif key == ord("x") and not self.points:
                self.delete_nearest()
            elif key == ord("z"):
                if self.zoom is None:
                    h, w = self.image.shape[:2]
                    x, y = self.mouse
                    self.zoom = (min(max(x - w // 4, 0), w // 2), min(max(y - h // 4, 0), h // 2))
                else:
                    self.zoom = None
            elif key == ord("d"):
                self.finish()
                self.doc["done"] = not self.doc.get("done")
                self.save()
            elif key in (ord("n"), ord("p")):
                self.finish()
                self.save()
                self.index = (self.index + (1 if key == ord("n") else -1)) % len(self.frames)
                self.open_frame()
            elif key == ord("q"):
                self.finish()
                self.save()
                break
        cv2.destroyAllWindows()
        done = sum(bool(self.load(f).get("done")) for f in self.frames)
        print(f"done {done}/{len(self.frames)}")


if __name__ == "__main__":
    if len(sys.argv) not in (2, 3):
        raise SystemExit(__doc__)
    Labeler(Path(sys.argv[1]), set(sys.argv[2].split(",")) if len(sys.argv) == 3 else None).run()
