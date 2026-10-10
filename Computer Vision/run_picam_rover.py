#!/usr/bin/env python3
"""
Script Inferensi Real-Time Pi Cam untuk Rover pada Raspberry Pi
---------------------------------------------------------------
Fungsi:
1. Membaca video stream langsung dari Pi Camera (CSI Camera via Picamera2 atau V4L2 OpenCV).
2. Menjalankan inferensi model YOLO (format .pt, .onnx, atau .tflite).
3. Mendeteksi rintangan (human, rock, vehicle, dll.) secara real-time.
4. Menghitung FPS, latensi, dan orientasi rintangan (Kiri, Tengah, Kanan) untuk navigasi rover.
5. Mendukung mode Headless (SSH tanpa monitor) dan mode GUI (cv2.imshow).
"""

import sys
import time
import argparse
from pathlib import Path
import cv2
import numpy as np

def init_camera(camera_type="opencv", camera_id=0, width=640, height=480, fps=30):
    """
    Inisialisasi kamera (mendukung libcamera/picamera2 dan OpenCV V4L2)
    """
    if camera_type.lower() == "picamera2":
        try:
            from picamera2 import Picamera2
            picam2 = Picamera2()
            config = picam2.create_preview_configuration(main={"size": (width, height), "format": "RGB888"})
            picam2.configure(config)
            picam2.start()
            print("[INFO] Berhasil menginisialisasi Pi Cam via Picamera2.")
            return ("picam2", picam2)
        except Exception as e:
            print(f"[WARNING] Picamera2 gagal ({e}). Fallback ke OpenCV VideoCapture...")

    # OpenCV VideoCapture (V4L2)
    cap = cv2.VideoCapture(camera_id, cv2.CAP_V4L2)
    if not cap.isOpened():
        # Fallback tanpa CAP_V4L2
        cap = cv2.VideoCapture(camera_id)
        
    if not cap.isOpened():
        print(f"[ERROR] Tidak dapat membuka kamera ID {camera_id}.")
        return (None, None)

    cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    cap.set(cv2.CAP_PROP_FPS, fps)
    print(f"[INFO] Kamera terbuka via OpenCV (ID: {camera_id}, {width}x{height} @ {fps}fps).")
    return ("opencv", cap)

def get_frame(cam_handle):
    cam_type, cam_obj = cam_handle
    if cam_type == "picamera2":
        frame = cam_obj.capture_array()
        # Picamera2 mengembalikan RGB, konversi ke BGR untuk OpenCV
        return True, cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
    elif cam_type == "opencv":
        return cam_obj.read()
    return False, None

def release_camera(cam_handle):
    cam_type, cam_obj = cam_handle
    if cam_type == "picamera2":
        try:
            cam_obj.stop()
        except Exception:
            pass
    elif cam_type == "opencv":
        cam_obj.release()

def main():
    parser = argparse.ArgumentParser(description="Rover Pi Cam Real-time YOLO Obstacle Detection")
    parser.add_argument("--model", type=str, default="best.pt",
                        help="Path ke file model (.pt, .onnx, .tflite, atau _ncnn_model)")
    parser.add_argument("--cam", type=str, default="opencv", choices=["opencv", "picamera2"],
                        help="Tipe interface kamera ('picamera2' atau 'opencv')")
    parser.add_argument("--cam-id", type=int, default=0,
                        help="Index video device V4L2 (default 0 untuk /dev/video0)")
    parser.add_argument("--imgsz", type=int, default=416,
                        help="Resolusi inferensi model (default 416)")
    parser.add_argument("--conf", type=float, default=0.45,
                        help="Confidence threshold deteksi (default 0.45)")
    parser.add_argument("--width", type=int, default=640,
                        help="Lebar resolusi kamera (default 640)")
    parser.add_argument("--height", type=int, default=480,
                        help="Tinggi resolusi kamera (default 480)")
    parser.add_argument("--headless", action="store_true",
                        help="Aktifkan jika menjalankan via SSH tanpa display GUI")

    args = parser.parse_args()

    # 1. Load YOLO Model
    print("=" * 60)
    print("🤖 ROVER PI CAM OBSTACLE DETECTOR")
    print("=" * 60)
    print(f"• Model Path     : {args.model}")
    print(f"• Inference Size : {args.imgsz}x{args.imgsz}")
    print(f"• Conf Threshold : {args.conf}")
    print(f"• Camera Type    : {args.cam}")
    print(f"• Headless Mode  : {args.headless}")
    print("=" * 60)

    try:
        from ultralytics import YOLO
    except ImportError:
        print("[ERROR] Modul 'ultralytics' belum terinstall pada Raspberry Pi!")
        print("Jalankan: pip install ultralytics")
        sys.exit(1)

    model_path = Path(args.model)
    if not model_path.exists():
        print(f"[ERROR] File model '{args.model}' tidak ditemukan!")
        print("Pastikan Anda sudah menyalin model hasil training (contoh: best.pt atau best.onnx) ke folder ini.")
        sys.exit(1)

    print(f"[INFO] Memuat model {args.model}...")
    model = YOLO(str(model_path), task="detect")
    print("[INFO] Model berhasil dimuat.")

    # 2. Inisialisasi Kamera
    cam_handle = init_camera(
        camera_type=args.cam,
        camera_id=args.cam_id,
        width=args.width,
        height=args.height
    )
    if cam_handle[0] is None:
        sys.exit(1)

    print("\n[INFO] Memulai streaming deteksi... (Tekan Ctrl+C atau 'q' untuk keluar)")

    fps_count = 0
    fps_start_time = time.time()
    current_fps = 0.0

    try:
        while True:
            loop_start = time.time()
            ret, frame = get_frame(cam_handle)
            if not ret or frame is None:
                print("[WARNING] Gagal membaca frame dari Pi Cam.")
                time.sleep(0.05)
                continue

            frame_h, frame_w = frame.shape[:2]
            center_x_zone = frame_w // 3

            # 3. Jalankan Inferensi YOLO
            results = model.predict(
                source=frame,
                imgsz=args.imgsz,
                conf=args.conf,
                verbose=False
            )

            detections_info = []

            for r in results:
                boxes = r.boxes
                for box in boxes:
                    x1, y1, x2, y2 = map(int, box.xyxy[0])
                    conf = float(box.conf[0])
                    cls_id = int(box.cls[0])
                    label = model.names[cls_id] if hasattr(model, 'names') else str(cls_id)

                    # Hitung titik tengah objek untuk panduan manuver rover
                    obj_cx = (x1 + x2) // 2
                    obj_cy = (y1 + y2) // 2
                    obj_width = x2 - x1

                    if obj_cx < center_x_zone:
                        position_sector = "KIRI"
                    elif obj_cx > (2 * center_x_zone):
                        position_sector = "KANAN"
                    else:
                        position_sector = "TENGAH"

                    detections_info.append({
                        "label": label,
                        "conf": conf,
                        "sector": position_sector,
                        "bbox": [x1, y1, x2, y2]
                    })

                    # Gambar Bounding Box
                    color = (0, 0, 255) if position_sector == "TENGAH" else (0, 255, 0)
                    cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
                    caption = f"{label} {conf:.2f} [{position_sector}]"
                    cv2.putText(frame, caption, (x1, max(y1 - 8, 15)),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 2)

            # Hitung FPS
            fps_count += 1
            elapsed = time.time() - fps_start_time
            if elapsed >= 1.0:
                current_fps = fps_count / elapsed
                fps_count = 0
                fps_start_time = time.time()

            latency_ms = (time.time() - loop_start) * 1000

            # Log ringkasan deteksi ke terminal
            if detections_info:
                det_summary = ", ".join([f"{d['label']}({d['sector']}, {d['conf']:.2f})" for d in detections_info])
                print(f"[DETEKSI] FPS: {current_fps:4.1f} | Latensi: {latency_ms:5.1f}ms | Objek: {det_summary}")
            else:
                # Cetak heartbeat setiap detik jika tidak ada objek
                if fps_count == 0:
                    print(f"[STATUS] Kamera Aktif | FPS: {current_fps:4.1f} | Bebas Rintangan")

            # Overlay info FPS pada gambar
            cv2.putText(frame, f"FPS: {current_fps:.1f} ({latency_ms:.0f}ms)", (15, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)

            # 4. Tampilkan GUI jika bukan headless
            if not args.headless:
                cv2.imshow("Rover Pi Cam - Object Detection", frame)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break

    except KeyboardInterrupt:
        print("\n[INFO] Menghentikan program...")
    finally:
        release_camera(cam_handle)
        if not args.headless:
            cv2.destroyAllWindows()
        print("[INFO] Kamera dilepas. Program selesai.")

if __name__ == "__main__":
    main()
