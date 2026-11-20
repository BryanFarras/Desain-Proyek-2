# Panduan Instalasi Dependensi AER Rover (Sekali Run)

Dokumen ini berisi panduan dan perintah sekali eksekusi (*one-liner / single-run script*) untuk menginstall seluruh dependensi yang dibutuhkan oleh **Autonomous Evacuation Rover (AER)**, mencakup:
- **Web Monitoring System** (Flask, psutil, websockets, transforms3d)
- **Komunikasi Hardware ESP32 TB6612FNG** (PySerial)
- **Kamera & Computer Vision** (OpenCV, NumPy)
- **Model Deteksi Rintangan AI/YOLO** (Ultralytics, PyTorch, Torchvision)

---

## ⚡ Eksekusi Sekali Run di Terminal

Pilih sesuai dengan sistem operasi yang Anda gunakan:

### 1. 🐧 Linux / Ubuntu (22.04 / 24.04) / Raspberry Pi OS

Cukup copy dan paste satu baris perintah berikut pada terminal:

```bash
chmod +x install_all_dependencies.sh && ./install_all_dependencies.sh
```

> **Alternatif One-Liner (Langsung Bash):**
> ```bash
> bash install_all_dependencies.sh
> ```

Script di atas akan secara otomatis:
1. Memperbarui indeks paket `apt` sistem dan menginstall dependensi pustaka OpenGL/GStreamer/V4L2 (`libgl1-mesa-glx`, `libglib2.0-0`, `v4l-utils`).
2. Membuat Python Virtual Environment (`.venv`) dengan opsi `--system-site-packages` agar dapat mengakses modul ROS 2 (`rclpy`) dan `python3-picamera2`.
3. Mengaktifkan environment dan meng-upgrade `pip`.
4. Menginstall seluruh pustaka Python dari [requirements.txt](file:///c:/Users/asus/Documents/Despro/requirements.txt).

---

### 2. 🪟 Windows (PowerShell)

Buka PowerShell pada direktori proyek (`c:\Users\asus\Documents\Despro`) dan jalankan:

```powershell
.\install_all_dependencies.ps1
```

> **Alternatif One-Liner (Tanpa file script terpisah):**
> ```powershell
> python -m pip install --upgrade pip && python -m pip install -r requirements.txt
> ```

---

## 📋 Daftar Paket & Dependensi yang Diinstall

Berikut adalah rincian seluruh dependensi yang tertera di [requirements.txt](file:///c:/Users/asus/Documents/Despro/requirements.txt):

| Kategori | Paket | Fungsi & Komponen |
| :--- | :--- | :--- |
| **Web Monitoring** | `flask >= 3.0.0` | Backend server dashboard monitoring real-time |
| | `psutil >= 5.9.0` | Membaca metrik performa CPU, RAM, Disk, dan network sistem |
| **Hardware Link** | `pyserial >= 3.5` | Komunikasi UART/Serial USB dengan ESP32 motor driver (TB6612FNG) |
| **Vision & Kamera** | `opencv-python >= 4.8.0` | Capture feed video webcam/CSI, drawing bounding box, encoding MJPEG |
| | `numpy >= 1.24.0` | Operasi matriks frame gambar dan kalkulasi telemetri SLAM |
| **AI / Object Detection** | `ultralytics >= 8.0.0` | Engine inferensi YOLOv8 real-time deteksi rintangan |
| | `torch`, `torchvision` | Backend neural network PyTorch untuk model inferensi |
| **Robotics Utilities** | `transforms3d >= 0.4.0` | Konversi orientasi kuaternion SLAM ke sudut Yaw |
| | `websockets >= 12.0` | Bridge protokol komunikasi WebSocket / ROSBridge |

---

## 🚀 Menjalankan Sistem Setelah Instalasi

Setelah proses instalasi dependensi selesai, Anda dapat langsung menjalankan komponen:

### 1. Menjalankan Web Monitoring Dashboard
```bash
# Di Linux / Raspberry Pi:
source .venv/bin/activate
cd "New Monitoring System"
python main.py
```

```powershell
# Di Windows:
cd "New Monitoring System"
python main.py
```

Buka browser di: **`http://localhost:5000`**

### 2. Menjalankan Computer Vision YOLO Real-Time
```bash
cd "Computer Vision"
python run_picam_rover.py --cam opencv --cam-id 0
```

---

## 🔧 Pemecahan Masalah (Troubleshooting)

1. **Permission Denied pada Port Serial di Linux:**
   Jika ESP32 terhubung ke `/dev/ttyUSB0` atau `/dev/ttyACM0` namun tidak dapat diakses, tambahkan user ke grup `dialout`:
   ```bash
   sudo usermod -aG dialout $USER
   ```
   *(Logout dan login kembali untuk menerapkan perubahan)*

2. **Error `libGL.so.1: cannot open shared object file` di Ubuntu:**
   Jalankan:
   ```bash
   sudo apt-get install -y libgl1-mesa-glx libglib2.0-0
   ```

3. **Script Execution Policy di Windows PowerShell:**
   Jika muncul peringatan script execution policy pada PowerShell, jalankan:
   ```powershell
   Set-ExecutionPolicy -Scope Process -ExecutionPolicy Bypass
   .\install_all_dependencies.ps1
   ```
