# Autonomous Evacuation Rover (AER) — DESPRO Rover

Sistem robot mobile *differential-drive* berbasis **ROS 2 Humble**: Raspberry Pi 4 + ESP32 Closed-Loop Motor Controller + LD06 LiDAR + Web Operations & RViz2 Dashboard.

Repositori ini mendukung seluruh alur kerja robotika:
1. **Setup Lingkungan Windows WSL 2 / Native Linux Ubuntu 22.04 LTS**
2. **Diagnostik Perangkat Keras & Deteksi Otomatis Port USB Serial**
3. **Pemetaan Awal (SLAM Toolbox 2D Mapping)**
4. **Navigasi Otonom (Nav2 & AMCL Particle Filter)**
5. **Monitoring & Kendali Jarak Jauh Berbasis Web (Operations & RViz2 Dashboard)**

---

## 1. Arsitektur Sistem & Aliran Data

```text
 ┌──────────────────────┐
 │   LiDAR LD06         │──/scan───────────┐
 └──────────────────────┘                  ├──> SLAM Toolbox ──/map──┬──> Nav2 Stack
 ┌──────────────────────┐                  │                         └──> AMCL ──/amcl_pose
 │   ESP32 Controller   │──/wheel_telem──> wheel_odometry ──/odom────┤             ▲
 └──────────────────────┘                  │    (+ TF odom->base)    │             │
            ▲                              │                         v             │
   /cmd_vel │                     ┌────────┴────────┐         ┌────────────────────┴┐
            └─────────────────────┤  esp32_bridge   │<────────┤ Nav2 / Web Dashboard│
                                  └─────────────────┘         └─────────────────────┘
```

### Spesifikasi Perangkat Keras
| Komponen | Spesifikasi & Koneksi | Fungsi |
|---|---|---|
| **SBC Utama** | Raspberry Pi 4 Model B (aarch64), Ubuntu 22.04 LTS | Menjalankan ROS 2 Humble & Web Dashboard |
| **Microcontroller** | ESP32-WROOM-32 (CP2102 USB Serial, 115200 baud) | PID closed-loop motor, pembacaan encoder roda |
| **Sensor Jarak** | LD06 LiDAR 360° (CP2102 USB Serial, 230400 baud) | Pemetaan SLAM, lokalisasi AMCL, *obstacle avoidance* |
| **Aktuator** | 2x DC Geared Motor (34:1, 11 PPR encoder) | Penggerak diferensial (radius roda 33.5 mm, track 200 mm) |
| **Driver Motor** | TB6612FNG H-Bridge Driver | Pengendali arah dan PWM motor DC |

---

## 2. Bagian I: Panduan Instalasi WSL 2 di Windows

Jika Anda menjalankan sistem monitoring, visualisasi RViz2 desktop, atau pengujian simulasi dari komputer Windows (Windows 10/11), gunakan langkah-langkah berikut:

### 2.1 Install WSL 2 & Ubuntu 22.04 LTS
Buka **PowerShell sebagai Administrator** di Windows, lalu jalankan:

```powershell
# 1. Pastikan WSL versi 2 menjadi default
wsl --set-default-version 2

# 2. Install Ubuntu 22.04 LTS
wsl --install -d Ubuntu-22.04

# 3. Setelah instalasi selesai, reboot komputer jika diminta oleh Windows
```

Buka Ubuntu 22.04 dari Start Menu Windows dan buat username & password akun Linux Anda.

### 2.2 Setup USB Passthrough ke WSL 2 (Menggunakan `usbipd-win`)
Agar port serial ESP32 dan LiDAR yang tercolok ke Windows dapat diakses langsung oleh WSL 2:

1. Di Windows PowerShell (Administrator), install `usbipd-win`:
   ```powershell
   winget install --interactive --exact dorssel.usbipd-win
   ```
2. Cek daftar perangkat USB yang terhubung di Windows:
   ```powershell
   usbipd list
   ```
   *Cari perangkat dengan keterangan `Silicon Labs CP210x USB to UART Bridge` (ESP32 dan LiDAR).*
3. Hubungkan perangkat USB ke WSL 2 (ganti `<BUSID>` sesuai output list di atas, misal `2-4`):
   ```powershell
   usbipd bind --busid <BUSID>
   usbipd attach --wsl --busid <BUSID>
   ```

### 2.3 Setup Dependensi ROS 2 Humble di WSL 2 / Ubuntu 22.04
Buka terminal **Ubuntu 22.04 (WSL 2)**, jalankan:

```bash
# 1. Update paket sistem
sudo apt update && sudo apt upgrade -y

# 2. Setup repository ROS 2 Humble
sudo apt install -y software-properties-common curl git build-essential
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# 3. Install ROS 2 Humble Desktop & tools navigasi
sudo apt update
sudo apt install -y \
    ros-humble-desktop \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-slam-toolbox \
    ros-humble-robot-state-publisher \
    ros-humble-xacro \
    ros-humble-tf2-tools \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-pip

# 4. Berikan izin akses port serial USB tanpa perlu sudo
sudo usermod -a -G dialout $USER
```

---

## 3. Bagian II: Build Workspace & Diagnostik USB

Clone repositori ini atau arahkan ke workspace rover Anda:

```bash
cd ~/desproRover_ws

# 1. Inisialisasi rosdep
sudo rosdep init 2>/dev/null || true
rosdep update
rosdep install --from-paths src --ignore-src -r -y

# 2. Build seluruh paket ROS 2
colcon build --symlink-install
source install/setup.bash
```

> [!TIP]
> Tambahkan perintah source ke `~/.bashrc` agar otomatis aktif setiap membuka terminal baru:
> ```bash
> echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
> echo "source ~/desproRover_ws/install/setup.bash" >> ~/.bashrc
> source ~/.bashrc
> ```

### 3.1 Verifikasi Port USB Otomatis (Pre-flight Diagnostic)
Kedua modul USB (ESP32 dan LD06) memiliki chip CP2102 dengan VID:PID identik (`10c4:ea60`). Sistem ini dilengkapi **deteksi dinamis 100%** berdasarkan aliran data asli (*probe*), sehingga urutan colokan `/dev/ttyUSB0` atau `/dev/ttyUSB1` tidak akan pernah tertukar lagi:

```bash
ros2 run AREbot_description usb_preflight
```
*Output yang diharapkan:*
```text
[OK] USB Terverifikasi: ESP32=/dev/ttyUSB1, LiDAR=/dev/ttyUSB0
```

---

## 4. Bagian III: Pemetaan Awal (SLAM Mapping)

Pemetaan dilakukan untuk membuat peta 2D (Occupancy Grid) lingkungan tempat rover akan beroperasi.

### Langkah 1: Jalankan Stack SLAM Real Robot (Terminal 1)
```bash
cd ~/desproRover_ws
source install/setup.bash
ros2 launch AREbot_description real_robot_slam.launch.py gui:=false
```
*Stack ini secara otomatis menjalankan:*
- Deteksi port USB dinamis (`usb_preflight`)
- Driver LiDAR LD06 (`ldlidar_stl_ros2_node`)
- Jembatan komunikasi ESP32 (`esp32_bridge`)
- Estimasi odometri roda (`wheel_odometry`)
- SLAM Toolbox 2D mapping online async
- Node agregator telemetri dashboard

### Langkah 2: Jalankan Monitoring & Kendalikan Robot (Terminal 2)

**Opsi A: Menggunakan Web Operations Dashboard (Rekomendasi)**
Buka terminal baru dan jalankan dashboard:
```bash
cd ~/desproRover_ws/dashboard
./start_dashboard.sh
```
Buka browser (di laptop/PC atau HP) pada alamat:
`http://<IP_RASPBERRY_PI>:5000` (atau `http://localhost:5000`)
1. Pilih mode **MANUAL (HITL)** pada navbar atas.
2. Buka tab **🗺️ RViz2 Map** untuk melihat peta yang sedang digambar secara real-time.
3. Buka tab **🎮 HITL Manual Drive** untuk mengemudikan rover menggunakan **Virtual Joystick**, **D-Pad**, atau tombol keyboard **WASD**.
4. Kelilingi seluruh ruangan hingga perimeter ruangan tertutup rapi pada tampilan peta.

**Opsi B: Menggunakan Teleop Terminal Standar**
```bash
ros2 launch AREbot_description teleop.launch.py
```
*(Gunakan tombol `i` maju, `,` mundur, `j` belok kiri, `l` belok kanan, `k` berhenti).*

### Langkah 3: Simpan Peta ke Disk (Terminal 3)
Setelah seluruh ruangan terpetakan dengan baik, simpan peta menggunakan:
```bash
cd ~/desproRover_ws
ros2 run nav2_map_server map_saver_cli -f ~/desproRover_ws/map_saved
```
*(Perintah ini akan menghasilkan dua file: `map_saved.yaml` dan `map_saved.pgm`).*

---

## 5. Bagian IV: Navigasi Otonom (Autonomous Navigation)

Setelah peta tersimpan, Anda dapat menjalankan Nav2 untuk navigasi otonom berbasis *waypoints* dan *costmaps*.

### Langkah 1: Jalankan Bringup Navigasi (Terminal 1)
```bash
cd ~/desproRover_ws
source install/setup.bash
ros2 launch AREbot_description main_realbot_nav_setup.launch.py map:=/home/dar/desproRover_ws/map_saved.yaml
```

> [!NOTE]
> **Perbaikan Integrasi Dashboard (Pencegahan Motor Berputar Tidak Terkontrol):**
> Pada versi sebelumnya, bringup navigasi secara bawaan mengeksekusi `auto_localize` yang langsung memutar robot secara liar sebanyak 2 putaran penuh (720°) dengan kecepatan 0.5 rad/s.
> Sekarang, argumen `auto_localize` telah diatur ke `false` secara default. Rover akan **diam dengan aman di posisi awalnya** dan menunggu perintah lokalisasi atau tujuan dari Anda.

### Langkah 2: Berikan Estimasi Posisi Awal (2D Pose Estimate)
Agar AMCL mengetahui letak rover di dalam peta:

- **Via Web Dashboard**:
  1. Buka dashboard di browser (`http://<IP_ROVER>:5000`).
  2. Di toolbar tab **RViz2 Map**, klik tombol **`📍 2D Pose Estimate`**.
  3. Klik titik pada peta tempat rover berada, lalu drag panah hijau sesuai arah hadap moncong rover saat ini.
- **Via RViz2 Desktop di Komputer Remote**:
  1. Klik tool **2D Pose Estimate** di toolbar atas RViz2.
  2. Klik dan tarik panah orientasi pada peta.

*(Partikel AMCL akan langsung terkonsentrasi di sekitar rover).*

### Langkah 3: Kirim Titik Tujuan Navigasi (2D Nav Goal)
1. Pada Web Dashboard, klik tombol **`🎯 2D Nav Goal`**.
2. Klik lokasi tujuan pada peta dan drag panah arah hadap akhir yang diinginkan.
3. Ubah mode rover ke **AUTONOMOUS** pada navbar atas.
4. Nav2 global planner (NavFn/Smac) dan controller (DWB/RPP) akan menghitung jalur bebas hambatan dan menggerakkan rover secara mandiri hingga mencapai target.

---

## 6. Bagian V: Panduan & Fitur Web Operations Dashboard

Dashboard web menyediakan antarmuka terpadu untuk monitoring, visualisasi RViz2 tanpa aplikasi desktop, dan kendali manual manusia (*Human-in-the-loop*).

```bash
cd ~/desproRover_ws/dashboard
./start_dashboard.sh
```

### Arsitektur Keamanan yang Diperbaiki:
1. **In-Flight Request Throttler**:
   - Menghilangkan *network latency backlog*. Perintah kendali dibatasi pada 10 Hz dengan antrean *non-blocking*. Rover berhenti seketika saat joystick atau tombol dilepas.
2. **Backend Safety Watchdog (400 ms)**:
   - Jika koneksi Wi-Fi atau browser terputus saat rover sedang melaju di mode MANUAL, backend otomatis memotong sinyal `/cmd_vel` ke `(0, 0)` dalam waktu 400 milidetik.
3. **Firmware Watchdog ESP32 (600 ms)**:
   - Jika kabel data Raspberry Pi terlepas atau program crash, ESP32 mematikan semua pin motor PWM secara hardware.
4. **Mode Isolation**:
   - **STANDBY**: Perintah motor manual dikunci rapat untuk mencegah sentuhan tidak sengaja.
   - **MANUAL**: Kendali penuh keyboard WASD, Joystick, dan D-Pad aktif dengan pembatas kecepatan aman.
   - **AUTONOMOUS**: Kendali manual dinonaktifkan agar tidak bertabrakan dengan sinyal navigasi Nav2.
   - **EMERGENCY STOP (Tombol Merah / Shortcut `ESC`)**: Menghentikan seluruh pergerakan seketika.

---

## 7. Referensi Perintah Cepat (Cheatsheet)

| Operasi | Perintah Terminal |
|---|---|
| **Cek Koneksi USB** | `ros2 run AREbot_description usb_preflight` |
| **Mulai SLAM Mapping** | `ros2 launch AREbot_description real_robot_slam.launch.py gui:=false` |
| **Simpan Peta** | `ros2 run nav2_map_server map_saver_cli -f ~/desproRover_ws/nama_peta` |
| **Mulai Nav2 Otonom** | `ros2 launch AREbot_description main_realbot_nav_setup.launch.py map:=/home/dar/desproRover_ws/nama_peta.yaml` |
| **Buka Web Dashboard** | `cd ~/desproRover_ws/dashboard && ./start_dashboard.sh` |
| **Teleop Terminal Manual** | `ros2 launch AREbot_description teleop.launch.py` |
| **Uji Komprehensif Nav2** | `python3 tools/nav_acceptance.py --goals "1.0,0.0,0; 0.0,0.0,0"` |
| **Kalibrasi Odometri Roda** | `python3 tools/calibrate_odometry.py --mode rotation --turns 5` |
| **Unit Test Kode** | `colcon test --packages-select AREbot_description && colcon test-result --verbose` |
