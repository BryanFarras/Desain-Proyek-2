#!/usr/bin/env python3
"""live_eval.py - Pengukuran drift pose (READ-ONLY).

Node HANYA membaca (subscriber + subprocess `ign/gz topic -e`). Tidak
mempublikasikan apa pun dan tidak mengubah node/parameter/config lain.

Metrik:
  * Drift vs ground-truth Gazebo : pose odom vs pose GT (RMSE, maks, akhir,
    selisih jarak tempuh %)
  * Fallback (tanpa GT)          : divergensi odom vs /amcl_pose

Ground-truth Fortress dibaca dari `/world/<world>/dynamic_pose/info`
(atau `/world/<world>/pose/info`) via CLI `ign`/`gz` (proses terpisah).

Pemakaian:
  python3 tools/eval/live_eval.py --durasi 60
  python3 tools/eval/live_eval.py --durasi 0 --debug
"""
import argparse
import csv
import math
import os
import re
import shutil
import subprocess
import threading
import time
from datetime import datetime

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.parameter import Parameter
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped


def yaw_from_quat(x, y, z, w):
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def ang_diff(a, b):
    d = a - b
    while d > math.pi:
        d -= 2 * math.pi
    while d < -math.pi:
        d += 2 * math.pi
    return d


_NUM = r"(-?[0-9]+(?:\.[0-9]*)?(?:[eE][-+]?[0-9]+)?)"


def pick_cli(preferred):
    if preferred:
        return preferred
    for c in ("ign", "gz"):
        p = shutil.which(c)
        if p:
            return p
    return "ign"


def topic_list(cli):
    try:
        out = subprocess.run([cli, "topic", "-l"], capture_output=True,
                             text=True, timeout=8).stdout
    except Exception:
        return []
    return [t.strip() for t in out.splitlines() if t.strip().startswith("/")]


def pick_gt_topic(cli, world, override):
    if override:
        return override
    topics = topic_list(cli)
    candidates = [
        f"/world/{world}/dynamic_pose/info",
        f"/world/{world}/pose/info",
        "/model/arebot/pose",
    ]
    for c in candidates:
        if c in topics:
            return c
    # fallback: apapun yang mengandung pose/info
    hit = [t for t in topics if "pose" in t and "info" in t]
    return hit[0] if hit else candidates[0]


class GzPoseReader(threading.Thread):
    """Baca pose ground-truth Gazebo via subprocess (read-only)."""

    def __init__(self, cli, topic, robot):
        super().__init__(daemon=True)
        self.cli, self.topic, self.robot = cli, topic, robot
        self.pose = None
        self.count = 0
        self.names_seen = set()

    def run(self):
        cmd = [self.cli, "topic", "-e", "-t", self.topic]
        try:
            proc = subprocess.Popen(cmd, stdout=subprocess.PIPE,
                                    stderr=subprocess.DEVNULL, text=True)
        except Exception as exc:
            print(f"[GT] gagal menjalankan '{' '.join(cmd)}': {exc}")
            return
        name, section, pos, ori = None, None, {}, {}
        for line in proc.stdout:
            s = line.strip()
            m = re.search(r'name:\s*"([^"]+)"', s)
            if m:
                name = m.group(1)
                self.names_seen.add(name)
            if s.startswith("position"):
                section, pos = "pos", {}
            elif s.startswith("orientation"):
                section, ori = "ori", {}
            for k, v in re.findall(rf'([xyzw]):\s*{_NUM}', s):
                if name == self.robot:
                    if section == "pos" and k in "xyz":
                        pos[k] = float(v)
                    elif section == "ori" and k in "xyzw":
                        ori[k] = float(v)
            if name == self.robot and len(pos) == 3 and len(ori) == 4:
                yaw = yaw_from_quat(ori["x"], ori["y"], ori["z"], ori["w"])
                self.pose = (pos["x"], pos["y"], yaw, time.time())
                self.count += 1
                name, section, pos, ori = None, None, {}, {}
        try:
            proc.terminate()
        except Exception:
            pass


class LiveEval(Node):
    def __init__(self, args, cli, gt_topic):
        super().__init__("live_eval_readonly",
                         parameter_overrides=[Parameter("use_sim_time", value=True)])
        qos = QoSProfile(depth=50, reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST)
        self.odom_count = 0
        self.amcl_count = 0
        self.last_odom = None
        self.last_amcl = None
        self.samples = []
        self.ref_gt = None    # (odom0, gt0) untuk meniadakan offset frame
        self.ref_amcl = None  # (odom0, amcl0)
        for t in args.odom_topic:
            self.create_subscription(Odometry, t, self.on_odom, qos)
        self.create_subscription(PoseWithCovarianceStamped, args.amcl_topic,
                                 self.on_amcl, qos)
        self.gt = GzPoseReader(cli, gt_topic, args.robot)
        self.gt.start()
        self.get_logger().info(
            f"live_eval (read-only) | cli={cli} | gt={gt_topic} | odom={args.odom_topic}")

    def on_odom(self, msg: Odometry):
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        self.last_odom = (p.x, p.y, yaw_from_quat(q.x, q.y, q.z, q.w), time.time())
        self.odom_count += 1

    def on_amcl(self, msg: PoseWithCovarianceStamped):
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        self.last_amcl = (p.x, p.y, yaw_from_quat(q.x, q.y, q.z, q.w))
        self.amcl_count += 1

    def snapshot(self):
        if self.last_odom is None:
            return None
        ox, oy, oyaw, _ = self.last_odom
        row = {"t": round(time.time(), 3),
               "odom_x": round(ox, 4), "odom_y": round(oy, 4), "odom_yaw": round(oyaw, 4),
               "err_odom_m": "", "err_yaw_odom_deg": ""}
        if self.gt.pose is not None:
            gx, gy, gyaw, _ = self.gt.pose
            row["gt_x"], row["gt_y"], row["gt_yaw"] = round(gx, 4), round(gy, 4), round(gyaw, 4)
            if self.ref_gt is None:
                self.ref_gt = (ox, oy, gx, gy, ang_diff(oyaw, gyaw))
            ox0, oy0, gx0, gy0, yaw0 = self.ref_gt
            # drift = deviasi perpindahan relatif (offset frame ditiadakan)
            row["err_odom_m"] = round(
                math.hypot((ox - ox0) - (gx - gx0), (oy - oy0) - (gy - gy0)), 5)
            row["err_yaw_odom_deg"] = round(
                abs(ang_diff(ang_diff(oyaw, gyaw), yaw0)) * 57.2958, 3)
        else:
            row["gt_x"] = row["gt_y"] = row["gt_yaw"] = ""
        if self.last_amcl is not None:
            ax, ay, _ = self.last_amcl
            row["amcl_x"], row["amcl_y"] = round(ax, 4), round(ay, 4)
            if self.ref_amcl is None:
                self.ref_amcl = (ox, oy, ax, ay)
            ox0, oy0, ax0, ay0 = self.ref_amcl
            row["err_odom_amcl_m"] = round(
                math.hypot((ox - ox0) - (ax - ax0), (oy - oy0) - (ay - ay0)), 5)
        else:
            row["amcl_x"] = row["amcl_y"] = row["err_odom_amcl_m"] = ""
        return row


def summarize(node):
    s = node.samples
    print("\n================ DIAGNOSTIK ================")
    print(f"  odom diterima      : {node.odom_count} pesan")
    print(f"  amcl diterima      : {node.amcl_count} pesan")
    print(f"  GT terbaca         : {node.gt.count} pose")
    if node.gt.count == 0 and node.gt.names_seen:
        print(f"  nama entitas di GT : {sorted(node.gt.names_seen)[:12]}")
    print("===========================================")

    gt_rows = [r for r in s if r["err_odom_m"] != ""]
    amcl_rows = [r for r in s if r["err_odom_amcl_m"] != ""]

    print("\n============ RINGKASAN (odom vs GT Gazebo) ============")
    if gt_rows:
        e = [r["err_odom_m"] for r in gt_rows]
        rmse = math.sqrt(sum(v * v for v in e) / len(e))

        def traveled(k):
            return sum(math.hypot(b[f"{k}_x"] - a[f"{k}_x"], b[f"{k}_y"] - a[f"{k}_y"])
                       for a, b in zip(gt_rows, gt_rows[1:]))

        d_gt = traveled("gt"); d_odom = traveled("odom")
        print(f"  sampel            : {len(gt_rows)}")
        print(f"  RMSE posisi       : {rmse*100:.2f} cm")
        print(f"  error maks        : {max(e)*100:.2f} cm")
        print(f"  error akhir       : {e[-1]*100:.2f} cm")
        print(f"  jarak tempuh GT   : {d_gt:.3f} m | odom: {d_odom:.3f} m")
        if d_gt > 1e-6:
            print(f"  error jarak       : {abs(d_odom-d_gt)/d_gt*100:.2f} %")
    else:
        print("  (GT tidak terpakai — lihat DIAGNOSTIK di atas)")
    print("======================================================")

    if amcl_rows:
        e = [r["err_odom_amcl_m"] for r in amcl_rows]
        print("\n======== RINGKASAN (odom vs amcl; tanpa GT) ========")
        print(f"  sampel            : {len(amcl_rows)}")
        print(f"  divergensi mean   : {sum(e)/len(e)*100:.2f} cm")
        print(f"  divergensi maks   : {max(e)*100:.2f} cm")
        print("====================================================")


def self_test(cli, gt_topic):
    print(f"CLI terpakai   : {cli}")
    print(f"GT topic       : {gt_topic}")
    print("Topik pose/odom yang terlihat:")
    for t in topic_list(cli):
        if "pose" in t or "odom" in t:
            print("   ", t)


def main():
    here = os.path.dirname(os.path.abspath(__file__))
    ap = argparse.ArgumentParser(description="Pengukuran drift pose (read-only)")
    ap.add_argument("--odom-topic", action="append",
                    default=None,
                    help="bisa diulang; default keduanya")
    ap.add_argument("--amcl-topic", default="/amcl_pose")
    ap.add_argument("--gt-topic", default=None)
    ap.add_argument("--world", default="building_floor_world")
    ap.add_argument("--robot", default="arebot")
    ap.add_argument("--cli", default=None, help="ign / gz (auto bila kosong)")
    ap.add_argument("--durasi", type=float, default=60.0, help="detik; 0 = Ctrl-C")
    ap.add_argument("--hz", type=float, default=10.0)
    ap.add_argument("--debug", action="store_true")
    ap.add_argument("--out", default=os.path.join(here, "out"))
    args = ap.parse_args()
    if not args.odom_topic:
        args.odom_topic = ["/diff_drive_controller/odom", "/odom"]

    cli = pick_cli(args.cli)
    gt_topic = pick_gt_topic(cli, args.world, args.gt_topic)
    self_test(cli, gt_topic)

    os.makedirs(args.out, exist_ok=True)
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    csv_path = os.path.join(args.out, f"live_eval_{stamp}.csv")

    rclpy.init()
    node = LiveEval(args, cli, gt_topic)
    period = 1.0 / max(1.0, args.hz)
    t0 = time.time()
    t_dbg = 0.0
    try:
        with open(csv_path, "w", newline="") as fh:
            writer = None
            while rclpy.ok():
                rclpy.spin_once(node, timeout_sec=period)
                s = node.snapshot()
                if s is not None:
                    node.samples.append(s)
                    if writer is None:
                        writer = csv.DictWriter(fh, fieldnames=list(s.keys()))
                        writer.writeheader()
                    writer.writerow(s)
                    fh.flush()
                if args.debug and time.time() - t_dbg > 2.0:
                    t_dbg = time.time()
                    print(f"[debug] odom={node.odom_count} amcl={node.amcl_count} "
                          f"gt={node.gt.count} samples={len(node.samples)}", flush=True)
                if args.durasi > 0 and (time.time() - t0) > args.durasi:
                    break
    except KeyboardInterrupt:
        pass
    finally:
        summarize(node)
        print(f"\nCSV : {csv_path}")
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
