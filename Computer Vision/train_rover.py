#!/usr/bin/env python3
"""
Script Training YOLO untuk Rover Obstacle Detection (Target: Raspberry Pi + Pi Cam)
-----------------------------------------------------------------------------------
Fungsi:
1. Melatih model YOLOv8n (Nano) pada dataset Obstacle_detection.
2. Menggunakan resolusi optimal untuk Raspberry Pi (misal 416x416 atau 640x640).
3. Otomatis mengekspor model ke format ringan (ONNX, TFLite, atau NCNN)
   yang siap dijalankan pada Raspberry Pi dengan Pi Camera.
"""

import os
import sys
import argparse
from pathlib import Path

def train_rover_model(
    data_yaml_path: str,
    base_model: str = "yolov8n.pt",
    epochs: int = 50,
    imgsz: int = 416,
    batch: int = 16,
    project_dir: str = "runs/rover_train",
    name: str = "rover_obstacle_model",
    export_formats: list = None
):
    try:
        from ultralytics import YOLO
        import torch
    except ImportError:
        print("\n[ERROR] Modul 'ultralytics' atau 'torch' belum terinstall!")
        print("Silakan jalankan perintah berikut terlebih dahulu:")
        print("  pip install ultralytics torch torchvision\n")
        sys.exit(1)

    data_path = Path(data_yaml_path).resolve()
    if not data_path.exists():
        print(f"\n[ERROR] File dataset config '{data_path}' tidak ditemukan!")
        sys.exit(1)

    device = "0" if torch.cuda.is_available() else "cpu"
    print("=" * 60)
    print("🚀 MEMULAI TRAINING MODEL ROVER YOLO")
    print("=" * 60)
    print(f"• Dataset Config : {data_path}")
    print(f"• Base Model     : {base_model} (direkomendasikan Nano untuk Raspi)")
    print(f"• Epochs         : {epochs}")
    print(f"• Image Size     : {imgsz}x{imgsz} (optimal untuk Pi Cam)")
    print(f"• Batch Size     : {batch}")
    print(f"• Device         : {'GPU (CUDA)' if device != 'cpu' else 'CPU'}")
    print("=" * 60)

    # 1. Load Pretrained YOLO Nano Model
    print(f"\n[1/3] Mengunduh/Memuat base model '{base_model}'...")
    model = YOLO(base_model)

    # 2. Training
    print("\n[2/3] Memulai proses training...")
    results = model.train(
        data=str(data_path),
        epochs=epochs,
        imgsz=imgsz,
        batch=batch,
        device=device,
        project=project_dir,
        name=name,
        workers=4,
        save=True,
        plots=True,
        verbose=True
    )

    best_weight_path = Path(project_dir) / name / "weights" / "best.pt"
    print("\n" + "=" * 60)
    print("✅ TRAINING SELESAI!")
    print(f"Model bobot terbaik tersimpan di: {best_weight_path}")
    print("=" * 60)

    # 3. Export ke format edge-computing untuk Raspberry Pi
    if export_formats is None:
        export_formats = ["onnx", "tflite", "ncnn"]

    best_model = YOLO(str(best_weight_path))
    print("\n[3/3] Mengekspor model untuk akselerasi Raspberry Pi...")
    
    for fmt in export_formats:
        try:
            print(f"\n--> Mengonversi ke format '{fmt}' (imgsz={imgsz})...")
            exported_file = best_model.export(
                format=fmt,
                imgsz=imgsz,
                half=False # set True jika hardware mendukung FP16
            )
            print(f"    Berhasil diekspor: {exported_file}")
        except Exception as e:
            print(f"    [Peringatan] Gagal mengekspor ke format {fmt}: {e}")
            print("    (Catatan: Anda dapat menginstal dependensi terkait seperti `pip install onnx` atau `pip install tensorflow`)")

    print("\n" + "=" * 60)
    print("🎉 SIAP DEPLOY KE RASPBERRY PI + PI CAM!")
    print("Langkah selanjutnya:")
    print("1. Salin file model (.pt / .onnx / .tflite) ke Raspberry Pi via SCP:")
    print(f"   scp {best_weight_path} pi@<IP_RASPBERRY_PI>:~/rover_models/")
    print("2. Jalankan script inferensi 'run_picam_rover.py' pada Raspberry Pi.")
    print("=" * 60)

def main():
    script_dir = Path(__file__).parent.resolve()
    default_data_yaml = (script_dir / ".." / "Dataset CV" / "Obstacle_detection" / "data.yaml").resolve()

    parser = argparse.ArgumentParser(description="Training YOLO Model untuk Rover & Pi Cam")
    parser.add_argument("--data", type=str, default=str(default_data_yaml),
                        help="Path ke data.yaml")
    parser.add_argument("--model", type=str, default="yolov8n.pt",
                        help="Base YOLO model (contoh: yolov8n.pt, yolo11n.pt)")
    parser.add_argument("--epochs", type=int, default=50,
                        help="Jumlah epoch training (default: 50)")
    parser.add_argument("--imgsz", type=int, default=416,
                        help="Ukuran gambar input (default: 416 untuk keseimbangan speed/akurasi di Raspi)")
    parser.add_argument("--batch", type=int, default=16,
                        help="Batch size (default: 16)")
    parser.add_argument("--project", type=str, default=str(script_dir / "runs"),
                        help="Folder output hasil training")
    parser.add_argument("--name", type=str, default="rover_obstacle_model",
                        help="Nama eksperimen training")

    args = parser.parse_args()

    train_rover_model(
        data_yaml_path=args.data,
        base_model=args.model,
        epochs=args.epochs,
        imgsz=args.imgsz,
        batch=args.batch,
        project_dir=args.project,
        name=args.name
    )

if __name__ == "__main__":
    main()
