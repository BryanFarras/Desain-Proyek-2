#!/usr/bin/env python3
"""map_eval.py - Pengukuran akurasi peta (OFFLINE, tanpa ROS).

Menghitung dari artefak peta SLAM (`.pgm` + `.yaml`) dibandingkan
ground-truth geometri dari world Gazebo (`.world`):

  * Akurasi dimensi peta  : rentang peta SLAM vs GT gedung (%), rotasi-invarian
  * Akurasi okupansi (IoU): overlat peta SLAM vs peta GT setelah penyelarasan
  * Error posisi dinding  : jarak garis dinding GT ke sel terdekat peta SLAM

Metode penyelarasan: ketebakan (rotasi kelipatan 90 + translasi) menyamakan
bbox peta SLAM dengan bbox GT gedung. Read-only, tidak menyentuh ROS.

Pemakaian:
  python3 tools/eval/map_eval.py
  python3 tools/eval/map_eval.py --map-yaml /path/building_map_save.yaml \
      --world /path/building_floor.world
"""
import argparse
import json
import math
import os
import re
from datetime import datetime

import numpy as np
from PIL import Image


def nearest_dist(samples, pts):
    out = np.empty(len(samples))
    for i, sp in enumerate(samples):
        if len(pts) == 0:
            out[i] = np.inf
            continue
        out[i] = np.hypot(pts[:, 0] - sp[0], pts[:, 1] - sp[1]).min()
    return out


def read_yaml(path):
    d = {}
    with open(path) as fh:
        for line in fh:
            line = line.split("#")[0].strip()
            if not line or ":" not in line:
                continue
            k, v = line.split(":", 1)
            v = v.strip()
            if v.startswith("[") and v.endswith("]"):
                v = [float(x) for x in v.strip("[]").split(",")]
            else:
                try:
                    v = float(v)
                except ValueError:
                    pass
            d[k.strip()] = v
    return d


def load_slam_points(yaml_path):
    meta = read_yaml(yaml_path)
    res = float(meta["resolution"])
    ox, oy = float(meta["origin"][0]), float(meta["origin"][1])
    occ_t = float(meta.get("occupied_thresh", 0.65))
    img_path = os.path.join(os.path.dirname(yaml_path), str(meta["image"]))
    img = np.asarray(Image.open(img_path).convert("L"))
    h, w = img.shape
    occ = img < (255.0 * (1.0 - occ_t))
    rows, cols = np.nonzero(occ)
    xs = ox + cols * res
    ys = oy + (h - 1 - rows) * res
    return np.column_stack([xs, ys]), res, (float(h), float(w))


def load_gt_rects(world_path):
    rects = []
    with open(world_path) as fh:
        for line in fh:
            if "<size>" not in line or "<pose>" not in line:
                continue
            msz = re.search(r"<size>\s*([-0-9.eE]+)\s+([-0-9.eE]+)\s+([-0-9.eE]+)", line)
            mps = re.search(r"<pose>\s*([-0-9.eE]+)\s+([-0-9.eE]+)\s+([-0-9.eE]+)", line)
            if not (msz and mps):
                continue
            sx, sy = float(msz.group(1)), float(msz.group(2))
            cx, cy = float(mps.group(1)), float(mps.group(2))
            if 0.3 <= max(sx, sy) <= 12 and min(sx, sy) <= 0.3:
                rects.append((cx, cy, sx, sy))  # only thin walls
    return rects


def rect_points(rect):
    cx, cy, sx, sy = rect
    hx, hy = sx / 2.0, sy / 2.0
    return (cx - hx, cy - hy, cx + hx, cy + hy)


def bbox_of(points):
    return (points[:, 0].min(), points[:, 1].min(),
            points[:, 0].max(), points[:, 1].max())


def grid_of(pts, bbox, res, margin=0.1):
    x0, y0, x1, y1 = bbox
    x0 -= margin; y0 -= margin; x1 += margin; y1 += margin
    nx = int(math.ceil((x1 - x0) / res))
    ny = int(math.ceil((y1 - y0) / res))
    g = np.zeros((ny, nx), dtype=bool)
    ix = np.clip(((pts[:, 0] - x0) / res).astype(int), 0, nx - 1)
    iy = np.clip(((pts[:, 1] - y0) / res).astype(int), 0, ny - 1)
    g[iy, ix] = True
    return g, (x0, y0, res)


def dilate(grid, k):
    if k <= 0:
        return grid
    out = grid.copy()
    for dy in range(-k, k + 1):
        for dx in range(-k, k + 1):
            out |= np.roll(np.roll(grid, dy, 0), dx, 1)
    return out


def main():
    here = os.path.dirname(os.path.abspath(__file__))
    ap = argparse.ArgumentParser(description="Akurasi peta (offline)")
    ap.add_argument("--map-yaml", default="/home/edar/desproRover_ws/building_map_save.yaml")
    ap.add_argument("--world", default="/home/edar/desproRover_ws/src/AREbot_description/worlds/building_floor.world")
    ap.add_argument("--out", default=os.path.join(here, "out"))
    args = ap.parse_args()

    slam_pts, res, _ = load_slam_points(args.map_yaml)
    rects = load_gt_rects(args.world)
    if len(rects) == 0:
        print("[!] Tidak ada dinding terdeteksi di world. Cek --world.")
        return

    gt_bbox = (min(r[0] - r[2] / 2 for r in rects), min(r[1] - r[3] / 2 for r in rects),
               max(r[0] + r[2] / 2 for r in rects), max(r[1] + r[3] / 2 for r in rects))
    gt_cx, gt_cy = (gt_bbox[0] + gt_bbox[2]) / 2, (gt_bbox[1] + gt_bbox[3]) / 2
    gt_span = sorted([gt_bbox[2] - gt_bbox[0], gt_bbox[3] - gt_bbox[1]])

    # --- penyelarasan: coba 4 rotasi kelipatan 90, pilih bbox paling cocok ---
    best = None
    for rot in (0, 90, 180, 270):
        th = math.radians(rot)
        c, s = math.cos(th), math.sin(th)
        rpts = np.column_stack([c * slam_pts[:, 0] - s * slam_pts[:, 1],
                                s * slam_pts[:, 0] + c * slam_pts[:, 1]])
        bb = bbox_of(rpts)
        span = sorted([bb[2] - bb[0], bb[3] - bb[1]])
        err = abs(span[0] - gt_span[0]) + abs(span[1] - gt_span[1])
        if best is None or err < best[0]:
            best = (err, rpts, bb, span)
    _, slam_rot, slam_bb, slam_span = best
    # translasi agar pusat bbox berimpit
    tx = gt_cx - (slam_bb[0] + slam_bb[2]) / 2
    ty = gt_cy - (slam_bb[1] + slam_bb[3]) / 2
    slam_al = slam_rot + np.array([tx, ty])

    # --- metrik dimensi ---
    dim_err = [abs(slam_span[i] - gt_span[i]) / gt_span[i] * 100 for i in range(2)]

    # --- grid & IoU ---
    bbox = (min(gt_bbox[0], bbox_of(slam_al)[0]), min(gt_bbox[1], bbox_of(slam_al)[1]),
            max(gt_bbox[2], bbox_of(slam_al)[2]), max(gt_bbox[3], bbox_of(slam_al)[3]))
    g_slam, _ = grid_of(slam_al, bbox, res)
    g_gt = np.zeros_like(g_slam)
    x0, y0, r = grid_of(np.zeros((1, 2)), bbox, res)[1]
    for rect in rects:
        rx0, ry0, rx1, ry1 = rect_points(rect)
        i0 = int((rx0 - x0) / r); i1 = int((rx1 - x0) / r)
        j0 = int((ry0 - y0) / r); j1 = int((ry1 - y0) / r)
        g_gt[max(j0, 0):j1 + 1, max(i0, 0):i1 + 1] = True
    inter = np.logical_and(g_slam, g_gt).sum()
    union = np.logical_or(g_slam, g_gt).sum()
    iou = inter / union * 100 if union else 0.0
    inter_d = np.logical_and(dilate(g_slam, 1), g_gt).sum()
    union_d = np.logical_or(dilate(g_slam, 1), g_gt).sum()
    iou_tol = inter_d / union_d * 100 if union_d else 0.0

    # --- error posisi dinding ---
    samples = []
    for rect in rects:
        cx, cy, sx, sy = rect
        n = max(2, int(max(sx, sy) / 0.05))
        for t in np.linspace(-0.5, 0.5, n):
            px = cx + (t * sx if sx >= sy else 0.0)
            py = cy + (t * sy if sy > sx else 0.0)
            samples.append((px, py))
    samples = np.array(samples)
    dist = nearest_dist(samples, slam_al)
    miss = dist > 0.30
    good = dist[~miss]
    wall_mean = good.mean() * 100 if good.size else float("nan")
    wall_med = np.median(good) * 100 if good.size else float("nan")
    wall_p90 = np.percentile(good, 90) * 100 if good.size else float("nan")
    coverage = (1 - miss.mean()) * 100

    result = {
        "timestamp": datetime.now().isoformat(timespec="seconds"),
        "map_yaml": args.map_yaml,
        "world": args.world,
        "resolution_m": res,
        "dimensi": {
            "gt_span_m": gt_span,
            "slam_span_m": slam_span,
            "error_pct": dim_err,
        },
        "iou_pct": round(iou, 2),
        "iou_pct_tol1": round(iou_tol, 2),
        "dinding": {
            "mean_cm": round(wall_mean, 2),
            "median_cm": round(wall_med, 2),
            "p90_cm": round(wall_p90, 2),
            "coverage_pct": round(coverage, 1),
        },
    }
    os.makedirs(args.out, exist_ok=True)
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    with open(os.path.join(args.out, f"map_eval_{stamp}.json"), "w") as fh:
        json.dump(result, fh, indent=2)

    print("\n================ RINGKASAN AKURASI PETA ================")
    print(f"  dimensi GT   (m)   : {gt_span[0]:.2f} x {gt_span[1]:.2f}")
    print(f"  dimensi SLAM (m)   : {slam_span[0]:.2f} x {slam_span[1]:.2f}")
    print(f"  error dimensi (%)  : {dim_err[0]:.2f} , {dim_err[1]:.2f}")
    print(f"  IoU peta (%)       : {iou:.1f}  (toleransi 1 sel: {iou_tol:.1f})")
    print(f"  error dinding      : mean {wall_mean:.1f} cm | median {wall_med:.1f} cm"
          f" | p90 {wall_p90:.1f} cm")
    print(f"  cakupan dinding (%) : {coverage:.1f}")
    print("=======================================================")
    print(f"\nJSON: {os.path.join(args.out, f'map_eval_{stamp}.json')}")


if __name__ == "__main__":
    main()
