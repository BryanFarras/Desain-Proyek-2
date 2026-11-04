# tools/eval — Toolkit Pengukuran Hasil Pengujian Awal

Folder ini **terisolasi**: tidak mengubah paket, launch, parameter, atau config
ROS apa pun. `live_eval.py` hanya **membaca** (subscriber + subprocess
`gz topic -e`), `map_eval.py` murni offline (tanpa ROS). Aman dijalankan
bersamaan dengan launch yang sudah ada.

## 1. `map_eval.py` (offline — bisa dijalankan kapan saja)
Menghitung akurasi peta dari artefak peta SLAM vs geometri world Gazebo.

```bash
python3 tools/eval/map_eval.py
# opsional:
python3 tools/eval/map_eval.py --map-yaml building_map_save.yaml \
    --world src/AREbot_description/worlds/building_floor.world
```

Output:
- **Akurasi dimensi peta** — rentang peta SLAM vs GT gedung (%), rotasi-invarian.
- **IoU okupansi** — overlay peta SLAM vs peta GT (raw + toleransi 1 sel).
- **Error posisi dinding** — jarak garis dinding GT ke sel terdekat peta SLAM
  (mean / median / p90, cm) + cakupan (%).

Hasil JSON ditulis ke `tools/eval/out/map_eval_<timestamp>.json`.

## 2. `live_eval.py` (live — saat Gazebo + robot jalan)
Mengukur drift pose: pose odometri vs pose **ground-truth Gazebo**.

```bash
# terminal lain, saat sim sudah jalan:
python3 tools/eval/live_eval.py --durasi 60
python3 tools/eval/live_eval.py --durasi 0 --debug   # rekam sampai Ctrl-C + log tiap 2s
```

- **Auto-pilih CLI** `ign`/`gz` dan **auto-deteksi topik GT** Fortress
  (`/world/<world>/dynamic_pose/info`).
- **Alignment otomatis**: offset frame odom-vs-world ditiadakan (drift = deviasi
  perpindahan relatif terhadap sampel awal), jadi hasilnya drift murni.
- Ground-truth dibaca read-only via `ign/gz topic -e` (proses terpisah).

Output:
- CSV per-sampel: `tools/eval/out/live_eval_<timestamp>.csv`
- Ringkasan: RMSE posisi (cm), error maks/akhir, jarak tempuh GT vs odom,
  error jarak (%) — plus divergensi odom vs `/amcl_pose` bila ada.

### Prosedur uji (untuk metrik drift) — WAJIB gerakkan robot
1. Jalankan sim: `ros2 launch AREbot_description gazebo.launch.py`
2. Terminal baru: `python3 tools/eval/live_eval.py --durasi 60 --debug`
3. **Gerakkan robot**: teleop (`ros2 run teleop_twist_keyboard teleop_twist_keyboard`)
   atau Nav2 → **jalan lurus ~2 m** (untuk drift odometri) dan/atau **loop
   kembali ke titik awal** (untuk drift lokalisasi).
4. Stop, salin angka RMSE / error jarak dari terminal.

> Robot diam → RMSE ≈ 0 (benar, tidak ada perpindahan). Angka baru bermakna
> saat robot bergerak.

## Argumen penting
| Argumen | Default | Keterangan |
|---|---|---|
| `--odom-topic` | `/diff_drive_controller/odom` + `/odom` | bisa diulang |
| `--amcl-topic` | `/amcl_pose` | untuk mode tanpa GT |
| `--gt-topic` | auto (dynamic_pose/info) | pose GT Gazebo |
| `--cli` | auto (`ign`→`gz`) | CLI transport |
| `--robot` | `arebot` | nama model di Gazebo |
| `--durasi` | `60` | detik (0 = sampai Ctrl-C) |
| `--debug` | off | log odom/amcl/gt tiap 2 s |
