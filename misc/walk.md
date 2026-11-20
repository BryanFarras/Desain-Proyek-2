# Hasil Implementasi Fondasi Script

Berdasarkan rancangan `plan1.txt`, saya telah membuatkan semua fondasi perangkat lunak untuk antarmuka pemantauan _Autonomous Evacuation Rover_.

## Apa yang telah dibuat?

1. **Frontend (Vite + React)**
   - Mengubah file HTML statis menjadi komponen React interaktif.
   - Menggunakan `roslibjs` untuk terhubung ke WebSocket `rosbridge`.
   - File utama berada di folder [frontend](file:///c:/Users/asus/Documents/Despro/frontend).
   - [App.jsx](file:///c:/Users/asus/Documents/Despro/frontend/src/App.jsx) (UI Dashboard)
   - [RosConnection.js](file:///c:/Users/asus/Documents/Despro/frontend/src/components/RosConnection.js) (Logika ROS WebSocket)
   - [index.css](file:///c:/Users/asus/Documents/Despro/frontend/src/index.css) (CSS Desain yang diadaptasi dari HTML mock Anda).

2. **Backend / Mock ROS (Python)**
   - Saya juga membuatkan mock server _websocket_ ringan menggunakan Python agar Anda dapat mengetes dashboard _tanpa_ perlu menyalakan ROS 2 asli di Raspberry Pi.
   - [ros2_telemetry_mock.py](file:///c:/Users/asus/Documents/Despro/backend/ros2_telemetry_mock.py)
   - Skrip ini akan menerbitkan data dummy (`/odom`, `/battery_status`, `/cmd_vel`) secara periodik yang akan ditangkap langsung oleh React.

## Cara Menjalankan untuk Uji Coba

Anda perlu membuka dua terminal (Command Prompt/PowerShell) baru:

### 1. Jalankan Backend (Mock ROS Server)

Buka terminal baru, masuk ke direktori `backend` dan jalankan skrip Python:

```bash
cd C:\Users\asus\Documents\Despro\backend
pip install -r requirements.txt
python ros2_telemetry_mock.py
```

> Server akan berjalan di `ws://localhost:9090` layaknya `rosbridge_server` yang asli.

### 2. Jalankan Frontend (React Dashboard)

Buka terminal lainnya, masuk ke direktori `frontend` dan jalankan Vite Server:

```bash
cd C:\Users\asus\Documents\Despro\frontend
npm run dev
```

> [!TIP]
> Klik URL yang muncul di terminal (biasanya `http://localhost:5173/`). Anda akan melihat UI Dashboard, dan indikator status di pojok kanan atas akan berubah menjadi **Connected**. Metrik posisi dan baterai akan bergerak secara otomatis merespons data dari Python.

## Langkah Selanjutnya

Jika Anda ingin mengintegrasikannya dengan hardware ROS 2 fisik (seperti Raspberry Pi 4B), Anda cukup:

1. Menjalankan package `rosbridge_suite` di Pi.
2. Mengubah IP Address WebSocket di file [RosConnection.js:8](file:///c:/Users/asus/Documents/Despro/frontend/src/components/RosConnection.js#L8) dari `localhost` menjadi IP dari Raspberry Pi Anda (misal: `ws://192.168.1.100:9090`).
