# AREbot Rover — Panduan Proyek (ROS 2)

Panduan ini ditulis untuk orang yang **baru mulai di ROS 2 / Gazebo**. Tujuannya sesederhana mungkin:
apa yang harus di-install, dari mana mulai, dan perintah apa yang dipakai.

Proyek ini adalah **rover otonom** yang bisa:
- **Mapping (SLAM)** memakai LiDAR,
- **Lokalisasi** di peta (AMCL),
- **Navigasi otonom** memakai Nav2.

Ada dua mode:
- **Simulasi** (Gazebo) — dicoba di laptop.
- **Robot nyata** — Raspberry Pi + ESP32 + LiDAR LD06.

---

## 0. Mulai dari sini

Pilih salah satu:

| Mau apa? | Langsung ke |
|---|---|
| Coba-coba di simulasi (tanpa robot) | [Bagian 6 — Simulasi](#6-simulasi-gazebo) |
| Bikin peta dengan robot asli | [Bagian 7.2 — Mapping](#72-mapping-robot-nyata) |
| Navigasi robot asli di peta | [Bagian 7.4 — Navigasi](#74-navigasi--lokalisasi-robot-nyata) |
| Bingung istilah ROS | [Bagian 1 — Istilah](#1-istilah-singkat) |

### 3 aturan emas (penting!)
1. **Setiap terminal baru**, selalu jalankan:
   ```bash
   source /opt/ros/humble/setup.bash
   source ~/desproRover_ws/install/setup.bash
   ```
   Kalau tidak, perintah `ros2 ...` tidak menemukan paketnya.
2. **Build selalu dari root workspace** (`~/desproRover_ws`), **jangan** dari dalam `src/...`.
3. Di robot nyata: **ESP32 = `/dev/ttyUSB0`**, **LiDAR = `/dev/ttyUSB1`**. Jangan tukar.

---

## 1. Istilah singkat

| Istilah | Arti singkat |
|---|---|
| **ROS 2 / node** | Program kecil yang saling bicara. |
| **Topic** | "Kanal" data (contoh `/scan`, `/cmd_vel`). |
| **TF** | Hubungan posisi antar bagian robot (contoh `odom → base_footprint → laser`). |
| **Launch file** | File `.launch.py` untuk menjalankan banyak node sekaligus. |
| **odom** | Perkiraan posisi robot dari putaran roda (encoder). |
| **SLAM** | Membuat peta + posisi robot sekaligus (di sini: `slam_toolbox`). |
| **AMCL** | Menentukan posisi robot di peta yang sudah ada (particle filter). |
| **map_server** | Memuat file peta (`.yaml` + `.pgm`) agar dipakai AMCL/Nav2. |
| **Nav2** | Tumpukan navigasi: perencanaan jalur + kontrol gerak + obstacle avoidance. |
| **RViz2** | Alat visualisasi (lihat peta, scan, TF). Boleh di laptop, jangan di Pi yang lemah. |

---

## 2. Isi folder workspace

```
~/desproRover_ws/
├── src/
│   ├── AREbot_description/          # PAKET UTAMA proyek ini
│   │   ├── AREbot_description/      # kode node Python (executable)
│   │   ├── launch/                  # file launch
│   │   ├── config/                  # parameter YAML + config RViz
│   │   ├── urdf/                    # model robot (xacro)
│   │   └── worlds/                  # dunia Gazebo
│   ├── ldlidar_stl_ros2/            # driver LiDAR LD06/LD19/STL27L
│   └── beluga*/                     # TIDAK DIPAKAI (abaikan)
├── build/ install/ log/             # hasil build colcon (jangan diedit)
├── kamar_save_map.pgm/.yaml         # peta kamar (robot nyata)
├── building_map_save.pgm/.yaml      # peta gedung (simulasi)
├── maze_map_save.pgm/.yaml          # peta maze (simulasi)
├── *_serialize.data/.posegraph      # posegraph SLAM (untuk lanjut mapping/lokalisasi)
└── tools/eval/                      # skrip evaluasi peta (opsional)
```

> **beluga / beluga_amcl / beluga_ros**: paket percobaan yang **tidak dipakai** proyek ini.
> Sudah dimasukkan `.gitignore`. Jangan dibuild/dipakai.

---

## 3. Paket yang harus di-install

### 3.1 Laptop (untuk simulasi Gazebo)

```bash
sudo apt update
sudo apt install ros-humble-desktop \
  ros-humble-ros-gz ros-humble-ros-gz-sim ros-humble-ros-gz-bridge \
  ros-humble-ros2-control ros-humble-ros2-controllers \
  ros-humble-ign-ros2-control ros-humble-gz-ros2-control \
  ros-humble-diff-drive-controller ros-humble-controller-manager ros-humble-joint-state-broadcaster \
  ros-humble-navigation2 ros-humble-nav2-bringup ros-humble-nav2-map-server ros-humble-nav2-amcl \
  ros-humble-slam-toolbox ros-humble-robot-state-publisher ros-humble-xacro \
  ros-humble-rviz2 ros-humble-tf2-tools ros-humble-topic-tools \
  python3-colcon-common-extensions python3-rosdep

# pyserial (untuk komunikasi ESP32 di robot nyata)
pip3 install pyserial        # atau: sudo apt install python3-serial
```

### 3.2 Raspberry Pi (untuk robot nyata)

```bash
sudo apt update
sudo apt install ros-humble-ros-base \
  ros-humble-robot-state-publisher ros-humble-xacro \
  ros-humble-navigation2 ros-humble-nav2-bringup ros-humble-nav2-map-server ros-humble-nav2-amcl \
  ros-humble-slam-toolbox ros-humble-tf2-tools \
  python3-serial python3-colcon-common-extensions python3-rosdep
```

> RViz **tidak perlu** di Pi. Jalankan RViz di laptop, di jaringan/ROS graph yang sama
> (`ROS_DOMAIN_ID` sama, DDS bisa saling lihat).

### 3.3 Dependensi paket workspace

```bash
cd ~/desproRover_ws
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
```

---

## 4. Build & source

```bash
cd ~/desproRover_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

- Jika ada perubahan kode/config: build ulang, lalu `source install/setup.bash` lagi.
- **Jangan** jalankan `colcon build` dari dalam `src/AREbot_description`.
  Kalau itu terjadi, akan muncul folder `build/ install/ log/` di dalam paket → bikin bingung.
  Hapus saja folder bersarang itu.

---

## 5. Simulasi Gazebo

### 5.1 Jalankan Gazebo + robot
```bash
ros2 launch AREbot_description gazebo.launch.py
```
- World default: `worlds/simple_maze.world`.
- World lain: `worlds/empty.world`, `worlds/building_floor.world`
  (ubah argumen world di `launch/gazebo.launch.py` kalau mau ganti).
- Robot spawn di `(2.0, 2.0, yaw -90°)`.

### 5.2 Mapping di simulasi (bangun peta)
1. Pastikan `config/mapper_params_online_async.yaml` diset untuk **mapping**:
   ```yaml
   mode: mapping
   # map_file_name: ...        <- comment / hapus
   # map_start_at_dock: true   <- comment / hapus
   ```
2. Jalankan:
   ```bash
   ros2 launch AREbot_description online_async_launch.py use_sim_time:=true
   ```
3. Gerakkan robot (lihat 5.3). Peta akan terbentuk di RViz.

### 5.3 Teleop (gerakkan robot dengan keyboard)
```bash
ros2 run AREbot_description teleop_twist_keyboard
```
- Default publish ke **`/cmd_vel`**.
- Di simulasi, `/cmd_vel` di-relay oleh `cmd_vel_relay` ke controller roda.

### 5.4 Navigasi di simulasi
Jalankan (setelah Gazebo + lokalisasi/map aktif):
```bash
ros2 run AREbot_description nav2_navigation
```
- Memakai `config/nav2_navigation_params.yaml` (khusus simulasi, odom dari
  `/diff_drive_controller/odom`).
- Untuk lokalisasi + peta di simulasi, jalankan `map_server` (edit path peta di
  `AREbot_description/map_server.py`) + `run_amcl`, atau `slam_toolbox` mode localization.

### 5.5 Simpan peta
```bash
ros2 run nav2_map_server map_saver_cli -f ~/desproRover_ws/peta_baru
```
Akan menghasilkan `peta_baru.pgm` + `peta_baru.yaml`.

---

## 6. Robot nyata (Raspberry Pi)

Alur: **colok USB → cek port → mapping → simpan peta → navigasi**.

### 6.1 Cek USB (safe boot)
`usb_preflight` mengecek apakah ESP32 di `ttyUSB0` dan LiDAR di `ttyUSB1`.
Kalau tertukar, ia akan **menunggu** sampai kabel ditukar.

### 6.2 Mapping robot nyata
```bash
ros2 launch AREbot_description real_robot_slam.launch.py
```
Yang dijalankan: `usb_preflight`, `robot_state_publisher`, LiDAR LD06 (`/scan`, frame `laser`),
`esp32_bridge`, `wheel_odometry` (`/odom`), `slam_toolbox` (mode mapping), dan RViz.
- Gerakkan robot pelan pelan (teleop) sampai peta terbentuk.
- RViz berat untuk Pi; kalau perlu, jalankan RViz di laptop dan matikan RViz di Pi.

### 6.3 Simpan peta
```bash
ros2 run nav2_map_server map_saver_cli -f ~/desproRover_ws/peta_baru
```

### 6.4 Navigasi + lokalisasi robot nyata
1. Edit `config/realbot_nav_setup.yaml`:
   ```yaml
   map_yaml: ~/desproRover_ws/peta_baru.yaml   # peta yang mau dipakai
   initial_pose:
     x: 0.0
     y: 0.0
     yaw: 0.0        # pose robot saat peta dibuat
   nomotion_update_rate: 2.0
   ```
2. Jalankan:
   ```bash
   ros2 launch AREbot_description main_realbot_nav_setup.launch.py
   ```
   atau pilih peta langsung saat run:
   ```bash
   ros2 launch AREbot_description main_realbot_nav_setup.launch.py map:=~/desproRover_ws/peta_baru.yaml
   ```
Yang dijalankan: `usb_preflight`, hardware (LiDAR, ESP32, odometri, robot_state_publisher),
**map_server** (memuat `.yaml`/`.pgm`), **AMCL**, `auto_localize` (set pose awal + spin +
publish `/initialpose` + nomotion update), dan **Nav2 navigation**. Tanpa RViz.

### 6.5 Catatan penting robot nyata
- `config/realbot_nav_setup.yaml` → ganti `map_yaml` untuk berganti peta.
- `initial_pose` harus cocok dengan posisi & arah robot saat peta dibuat.
- `auto_localize` akan **memutar robot ±2 putaran** saat start (pastikan ruang cukup).
- Jangan jalankan Gazebo/`diff_drive_controller` bersamaan dengan robot nyata
  (nanti TF `odom → base_footprint` dobel).

---

## 7. Daftar executable (`ros2 run AREbot_description <nama>`)

| Executable | Fungsi | Dipakai di |
|---|---|---|
| `usb_preflight` | Cek ESP32=`ttyUSB0`, LiDAR=`ttyUSB1`; menunggu kalau tertukar | Robot nyata |
| `esp32_bridge` | Serial ESP32: baca `DATA:...` → `/wheel_telemetry`; `/cmd_vel` → RPM → `speed(...)`; watchdog `berhenti` | Robot nyata |
| `wheel_odometry` | `/wheel_telemetry` (tick encoder) → `/odom` + TF `odom→base_footprint` | Robot nyata |
| `auto_localize` | Set pose awal (prior) + spin + publish `/initialpose`; bisa lanjut nomotion update | Robot nyata |
| `map_server` | Jalankan `nav2_map_server` + lifecycle configure/activate (peta dari `.yaml`) | Sim/real |
| `run_amcl` | Jalankan AMCL + lifecycle + auto_localize + nomotion update | Sim/real |
| `nav2_navigation` | Jalankan Nav2 navigation (`navigation_launch.py`), `use_sim_time:=true` | Simulasi |
| `scan_relay` | Relay `/scan` → `/amcl_scan` (dengan stamp baru) | Opsional |
| `cmd_vel_relay` | Relay `/cmd_vel` → `/diff_drive_controller/cmd_vel_unstamped` (QoS kompatibel) | Simulasi |
| `teleop_twist_keyboard` | Teleop keyboard; param `speed`, `turn`, `topic` (default `/cmd_vel`) | Sim/real |
| `imu_reader` | Konversi quaternion IMU → Euler (derajat) di `/imu` (non-standar) | Opsional |

---

## 8. Daftar launch file

| Launch file | Fungsi |
|---|---|
| `gazebo.launch.py` | Gazebo + robot + bridge + controller + RViz (simulasi) |
| `online_async_launch.py` | SLAM `slam_toolbox` async (mapping/lokalisasi) di simulasi; default `use_sim_time:=true` |
| `nav2_script.launch.py` | Teleop + `twist_mux` (simulasi) |
| `real_robot_slam.launch.py` | **Mapping robot nyata**: preflight + hardware + slam_toolbox + RViz |
| `main_realbot_nav_setup.launch.py` | **Navigasi+lokalisasi robot nyata**: preflight + hardware + map_server + AMCL + auto_localize + Nav2 (tanpa RViz) |
| `beluga_localizaton_amcl.launch.py` | Legacy berbasis beluga — **tidak dipakai** |

---

## 9. Config penting (`src/AREbot_description/config/`)

| File | Isi |
|---|---|
| `arebot_controllers.yaml` | Parameter `diff_drive_controller` (roda, limit kecepatan/akselerasi) untuk simulasi |
| `nav2_navigation_params.yaml` | Parameter Nav2 **simulasi** (odom `/diff_drive_controller/odom`) |
| `nav2_navigation_params_real.yaml` | Parameter Nav2 **robot nyata** (odom `/odom`) |
| `nav2_amcl_params.yaml` | Parameter AMCL |
| `mapper_params_online_async.yaml` | Parameter SLAM (simulasi); `mode: mapping`/`localization` |
| `mapper_params_real_mapping.yaml` | Parameter SLAM robot nyata (mode mapping) |
| `slam_toolbox_localization.yaml` | Parameter SLAM mode localization |
| `esp32_bridge.yaml` | Port/baud ESP32, geometri roda, invert, timeout, `motor_a_is_right` |
| `wheel_odometry.yaml` | `ticks_per_rev`, radius, separation, sign, `arm_on_cmd_vel`, `publish_tf` |
| `realbot_nav_setup.yaml` | Peta (`map_yaml`), `initial_pose`, `nomotion_update_rate` |
| `default.rviz`, `real_slam.rviz` | Konfigurasi RViz |
| `twist_mux_params.yaml` | Prioritas `cmd_vel` (simulasi) |
| `beluga_amcl_params.yaml` | Legacy beluga — tidak dipakai |

**Nilai robot nyata yang sudah dipakai:** `wheel_radius=0.0335` (diameter 67 mm),
`wheel_separation=0.20`, `ticks_per_rev≈232`, `motor_a_is_right=true`
(Motor A = roda kanan).

---

## 10. Topik & TF

**Topik utama:**

| Topik | Tipe | Keterangan |
|---|---|---|
| `/scan` | `sensor_msgs/LaserScan` | Data LiDAR (frame `laser`) |
| `/wheel_telemetry` | `std_msgs/Float64MultiArray` | `[tickA, tickB, rpmA, rpmB]` dari ESP32 |
| `/odom` | `nav_msgs/Odometry` | Odometri roda (robot nyata) |
| `/cmd_vel` | `geometry_msgs/Twist` | Perintah gerak (teleop/Nav2) |
| `/map` | `nav_msgs/OccupancyGrid` | Peta (latched / Transient Local) |
| `/amcl_pose` | `geometry_msgs/PoseWithCovarianceStamped` | Pose hasil AMCL |
| `/initialpose` | `geometry_msgs/PoseWithCovarianceStamped` | Pose awal untuk AMCL |

**Rantai TF:**
```
map → odom → base_footprint → base_link → laser
```
- `map→odom`: dari AMCL / SLAM.
- `odom→base_footprint`: dari `wheel_odometry` (robot nyata) atau `diff_drive_controller` (sim).
- `base_footprint→base_link→laser`: dari `robot_state_publisher` (URDF).

---

## 11. Kendala yang pernah ditemui & solusinya

| Gejala | Penyebab | Solusi yang dipakai |
|---|---|---|
| Peta SLAM tidak akurat / drift | `minimum_travel=0`, `transform_timeout` kecil, TF ganda dari bridge `/tf` Gazebo | travel threshold wajar, timeout 0.5, **hapus bridge `/tf`** |
| LiDAR simulasi beda dengan LD06 asli | FOV 270°, 40 Hz | ubah jadi 360°, 10 Hz, 450 sample, 0.05–12 m, noise ±2 cm |
| Robot diam padahal Nav2 mengirim path | QoS `/cmd_vel` tidak cocok (relay `TRANSIENT_LOCAL` vs publisher `VOLATILE`) | ganti relay dengan `cmd_vel_relay` QoS kompatibel |
| Map tidak muncul di RViz | durability Map display `Volatile` vs publisher latched | set display Map → **Transient Local**, Fixed Frame `map` |
| Map baru muncul setelah AMCL | `map→odom` belum ada | jalankan AMCL/lokalisasi + set pose awal |
| Harus klik 2D Pose Estimate tiap start | AMCL `set_initial_pose=false` | `auto_localize` (prior + spin + publish `/initialpose`) |
| ESP32 & LiDAR tidak bisa dibedakan | dua-duanya CP2102, serial sama `0001` | pakai `ttyUSB0`/`ttyUSB1` + `usb_preflight` |
| Roda twitch saat start → origin odom geser | ESP32 reset/glitch saat port dibuka | `dtr/rts=False`, kirim `berhenti` saat connect, `arm_on_cmd_vel` |
| "Tidak ada data dari ESP32" | `delay(1000)` + `readStringUntil` blocking di firmware | `data_timeout` dinaikkan, kirim `berhenti` saat connect |
| Odom jarak meleset (1 m → 0.75 m) | `ticks_per_rev` belum benar | kalibrasi → `ticks_per_rev≈232`, `radius=0.0335`, `sep=0.20` |
| Belok kiri/kanan terbalik | Motor A = roda kanan | `motor_a_is_right: true` + urutan `speed(rpm_kanan,rpm_kiri)` |
| Robot putar ~180° lalu mundur saat goal (Pi) | `default_server_timeout=20 ms` + `bt_loop_duration=10 ms` + CPU Pi jenuh → BT timeout → recovery Spin/BackUp | naikkan timeout, turunkan frequency, ringankan costmap, hentikan proses berat |
| Robot "terkurung" costmap | `obstacle_min_range=0.0` + LiDAR melihat badan sendiri | set `obstacle/raytrace_min_range ≈0.18`, ringankan voxel |
| `map_server` stuck / lifecycle menunggu | path peta salah / file tidak ada | perbaiki `map_yaml`, pastikan `.yaml` + `.pgm` ada |
| RViz berat di Pi | RViz + Nav2 + SLAM di satu Pi | jalankan RViz di laptop |
| Build aneh / versi lama | `colcon build` dari dalam paket → folder bersarang | selalu build dari root workspace |

---

## 12. Troubleshooting cepat

```bash
ros2 node list                       # node apa saja yang hidup?
ros2 topic list                      # topik apa saja yang ada?
ros2 topic echo /scan --once         # LiDAR mengalir?
ros2 topic echo /odom --once         # odometri mengalir?
ros2 run tf2_tools view_frames       # struktur TF (cek map→odom→base_footprint)
ros2 run tf2_ros tf2_echo map base_footprint
```

- **USB tidak terdeteksi / tertukar** → `ros2 run AREbot_description usb_preflight`
  (atau cek `ls -l /dev/ttyUSB*`).
- **Map_server stuck** → cek `ros2 param get /map_server yaml_filename` dan pastikan file
  `.yaml` + `.pgm` ada di path itu.
- **Nav2 langsung spin/backup** → lihat log `Timed out while waiting for action server...`;
  naikkan `default_server_timeout`, kurangi beban Pi.
- **CPU Pi tinggi** → hentikan proses berat (`lcd-mirror.py`, `unattended-upgrade`), cek
  `vcgencmd get_throttled` (harus `0x0`).

---

## 13. Catatan beluga & file yang diabaikan

- `src/beluga`, `src/beluga_amcl`, `src/beluga_ros` → **tidak dipakai** (sudah di `.gitignore`).
- `build/`, `install/`, `log/` → hasil build (diabaikan git).
- `frames_*.gv/pdf` → output debug TF (diabaikan git).
- `.agents/`, `.claude/`, `skills-lock.json` → tooling developer (diabaikan git).

---

## 14. Daftar peta di root workspace

| File | Keterangan |
|---|---|
| `kamar_save_map.yaml` + `.pgm` | Peta **kamar** dari robot nyata |
| `building_map_save.yaml` + `.pgm` | Peta gedung (simulasi) |
| `maze_map_save.yaml` + `.pgm` | Peta maze (simulasi) |
| `building_map_serialize.data` / `.posegraph` | Posegraph SLAM gedung (untuk lanjut mapping/lokalisasi) |
| `maze_map_serialize.data` / `.posegraph` | Posegraph SLAM maze |

Untuk memakai peta lain, cukup ganti `map_yaml` di `config/realbot_nav_setup.yaml`
(atau `map:=...` saat launch). Pastikan file `.pgm` yang dirujuk ada di folder yang sama.

---

## 15. Ringkasan perintah yang paling sering dipakai

```bash
# Simulasi
ros2 launch AREbot_description gazebo.launch.py
ros2 launch AREbot_description online_async_launch.py use_sim_time:=true
ros2 run AREbot_description teleop_twist_keyboard
ros2 run AREbot_description nav2_navigation

# Robot nyata — mapping
ros2 launch AREbot_description real_robot_slam.launch.py
ros2 run nav2_map_server map_saver_cli -f ~/desproRover_ws/peta_baru

# Robot nyata — navigasi
ros2 launch AREbot_description main_realbot_nav_setup.launch.py
# atau pilih peta lain:
ros2 launch AREbot_description main_realbot_nav_setup.launch.py map:=~/desproRover_ws/peta_baru.yaml

# Cek koneksi USB
ros2 run AREbot_description usb_preflight
```
