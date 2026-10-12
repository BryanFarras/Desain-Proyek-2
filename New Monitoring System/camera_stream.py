"""
Camera Stream Manager for Autonomous Evacuation Rover
------------------------------------------------------
Supports:
1. Raspberry Pi PiCamera2 (if hardware present)
2. OpenCV VideoCapture (USB/V4L2/DShow Webcams)
3. Tactical Synthetic HUD Feed (automatic fallback if no camera hardware is attached)
"""

import time
import math
import threading
import numpy as np
import cv2

try:
    from picamera2 import Picamera2
    PICAMERA2_AVAILABLE = True
except ImportError:
    PICAMERA2_AVAILABLE = False


class CameraStreamer:
    def __init__(self, width: int = 640, height: int = 480, fps: int = 25):
        self.width = width
        self.height = height
        self.target_fps = fps
        self.frame_interval = 1.0 / fps

        self.hardware_active = False
        self.camera_type = "none"  # "picamera2", "opencv", "synthetic"
        self.cam_obj = None
        self.lock = threading.Lock()

        self.current_frame = None
        self.running = True
        self.frame_count = 0
        self.start_time = time.time()

        # Shared telemetry references to render HUD
        self.telemetry = {
            "mode": "STANDBY",
            "battery": 100.0,
            "x": 0.0,
            "y": 0.0,
            "speed": 0.0,
            "detections": []
        }

        # Initialize background capture
        self.thread = threading.Thread(target=self._worker_loop, daemon=True)
        self.thread.start()

    def update_telemetry(self, telemetry_dict: dict):
        """Update telemetry overlay values."""
        with self.lock:
            self.telemetry.update(telemetry_dict)

    def _init_hardware_camera(self) -> bool:
        """Attempt to open hardware camera safely."""
        # 1. Try PiCamera2
        if PICAMERA2_AVAILABLE:
            try:
                cam = Picamera2()
                config = cam.create_video_configuration(
                    main={"size": (self.width, self.height)}
                )
                cam.configure(config)
                cam.start()
                self.cam_obj = cam
                self.camera_type = "picamera2"
                self.hardware_active = True
                return True
            except Exception as e:
                print(f"[CAM] Picamera2 init failed: {e}")

        # 2. Try OpenCV (Non-blocking check on index 0)
        # On Windows, DirectShow backend is fast; on Linux, V4L2
        backends = [cv2.CAP_DSHOW] if hasattr(cv2, 'CAP_DSHOW') else [cv2.CAP_ANY]
        for backend in backends:
            try:
                cap = cv2.VideoCapture(0, backend)
                if cap.isOpened():
                    cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
                    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
                    ret, test_frame = cap.read()
                    if ret and test_frame is not None:
                        self.cam_obj = cap
                        self.camera_type = "opencv"
                        self.hardware_active = True
                        print("[CAM] Hardware camera opened via OpenCV.")
                        return True
                    cap.release()
            except Exception as e:
                print(f"[CAM] OpenCV test exception: {e}")

        self.camera_type = "synthetic"
        self.hardware_active = False
        print("[CAM] Using Tactical Synthetic Rover HUD (fallback).")
        return False

    def _draw_synthetic_hud(self, t: float) -> np.ndarray:
        """Generate high-tech tactical Rover HUD frame."""
        frame = np.zeros((self.height, self.width, 3), dtype=np.uint8)

        # Background gradient / subtle grid
        for y in range(0, self.height, 40):
            cv2.line(frame, (0, y), (self.width, y), (18, 20, 26), 1)
        for x in range(0, self.width, 40):
            cv2.line(frame, (x, 0), (x, self.height), (18, 20, 26), 1)

        cx, cy = self.width // 2, self.height // 2

        # Artificial horizon line
        pitch_offset = int(math.sin(t * 0.8) * 12)
        roll_angle = math.sin(t * 0.5) * 4  # degrees

        # Rotating horizon bar
        rad = math.radians(roll_angle)
        cos_r = math.cos(rad)
        sin_r = math.sin(rad)
        h_len = 120
        p1 = (int(cx - h_len * cos_r), int(cy + pitch_offset - h_len * sin_r))
        p2 = (int(cx + h_len * cos_r), int(cy + pitch_offset + h_len * sin_r))
        cv2.line(frame, p1, p2, (40, 160, 240), 2)

        # Center Reticle Crosshair
        cv2.circle(frame, (cx, cy), 28, (0, 200, 255), 1)
        cv2.circle(frame, (cx, cy), 4, (0, 220, 255), -1)
        cv2.line(frame, (cx - 40, cy), (cx - 10, cy), (0, 200, 255), 1)
        cv2.line(frame, (cx + 10, cy), (cx + 40, cy), (0, 200, 255), 1)
        cv2.line(frame, (cx, cy - 40), (cx, cy - 10), (0, 200, 255), 1)
        cv2.line(frame, (cx, cy + 10), (cx, cy + 40), (0, 200, 255), 1)

        # Sweeping radar circle in corner
        rx, ry, r_rad = 70, 70, 45
        cv2.circle(frame, (rx, ry), r_rad, (30, 80, 50), 1)
        cv2.circle(frame, (rx, ry), r_rad // 2, (30, 80, 50), 1)
        sweep_angle = (t * 2.5) % (2 * math.pi)
        sx = int(rx + r_rad * math.cos(sweep_angle))
        sy = int(ry + r_rad * math.sin(sweep_angle))
        cv2.line(frame, (rx, ry), (sx, sy), (0, 255, 120), 2)
        cv2.putText(frame, "RADAR 360", (rx - 32, ry + r_rad + 14),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.35, (0, 255, 120), 1)

        # Corner Tactical Brackets
        c_len = 20
        # Top-left
        cv2.line(frame, (10, 10), (10 + c_len, 10), (100, 150, 255), 2)
        cv2.line(frame, (10, 10), (10, 10 + c_len), (100, 150, 255), 2)
        # Top-right
        cv2.line(frame, (self.width - 10, 10), (self.width - 10 - c_len, 10), (100, 150, 255), 2)
        cv2.line(frame, (self.width - 10, 10), (self.width - 10, 10 + c_len), (100, 150, 255), 2)
        # Bottom-left
        cv2.line(frame, (10, self.height - 10), (10 + c_len, self.height - 10), (100, 150, 255), 2)
        cv2.line(frame, (10, self.height - 10), (10, self.height - 10 - c_len), (100, 150, 255), 2)
        # Bottom-right
        cv2.line(frame, (self.width - 10, self.height - 10), (self.width - 10 - c_len, self.height - 10), (100, 150, 255), 2)
        cv2.line(frame, (self.width - 10, self.height - 10), (self.width - 10, self.height - 10 - c_len), (100, 150, 255), 2)

        # Dynamic simulated obstacle bounding boxes
        mode = self.telemetry.get("mode", "STANDBY")
        detections = self.telemetry.get("detections", [])

        # If detections exist in telemetry, render them
        if detections:
            for det in detections:
                bbox = det.get("bbox", [180, 140, 280, 320])
                label = det.get("label", "Target")
                conf = det.get("confidence", 0.88)
                color = (0, 60, 255) if "orang" in label.lower() or "person" in label.lower() else (0, 220, 255)
                cv2.rectangle(frame, (bbox[0], bbox[1]), (bbox[2], bbox[3]), color, 2)
                cv2.putText(frame, f"{label} {int(conf*100)}%", (bbox[0], max(bbox[1] - 6, 15)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.45, color, 1)
        else:
            # Default tactical tracking box (simulated human target in patrol/evacuation)
            if mode in ["PATROL", "EVACUATION"]:
                bx = int(360 + math.sin(t * 1.2) * 40)
                by = int(180 + math.cos(t * 0.9) * 20)
                cv2.rectangle(frame, (bx, by), (bx + 90, by + 160), (0, 255, 180), 1)
                cv2.putText(frame, f"ORANG (HITL) 91%", (bx, by - 6),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 180), 1)

        # Status HUD Text
        cv2.putText(frame, f"AER VISION FEED [SIMULATED HUD]", (self.width - 240, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (200, 200, 220), 1)
        cv2.putText(frame, f"MODE: {mode}", (self.width - 240, 48),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (50, 200, 255), 1)
        cv2.putText(frame, f"POS: ({self.telemetry.get('x', 0):.1f}, {self.telemetry.get('y', 0):.1f}) m",
                    (self.width - 240, 66), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (180, 180, 180), 1)
        cv2.putText(frame, f"SPD: {self.telemetry.get('speed', 0):.2f} m/s", (self.width - 240, 84),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (180, 180, 180), 1)

        # Timestamp
        now_str = time.strftime("%Y-%m-%d %H:%M:%S")
        cv2.putText(frame, now_str, (16, self.height - 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (120, 140, 160), 1)

        return frame

    def _worker_loop(self):
        """Frame acquisition loop."""
        self._init_hardware_camera()

        while self.running:
            start_t = time.time()
            frame = None

            if self.hardware_active and self.cam_obj:
                try:
                    if self.camera_type == "picamera2":
                        arr = self.cam_obj.capture_array()
                        frame = cv2.cvtColor(arr, cv2.COLOR_RGB2BGR)
                    elif self.camera_type == "opencv":
                        ret, raw_frame = self.cam_obj.read()
                        if ret and raw_frame is not None:
                            frame = raw_frame
                except Exception as e:
                    print(f"[CAM] Read exception: {e}")
                    self.hardware_active = False

            if frame is None:
                # Render synthetic HUD
                frame = self._draw_synthetic_hud(time.time() - self.start_time)

            # Store current frame
            with self.lock:
                self.current_frame = frame
                self.frame_count += 1

            # Sleep to maintain target FPS
            elapsed = time.time() - start_t
            sleep_time = max(0.01, self.frame_interval - elapsed)
            time.sleep(sleep_time)

    def generate_mjpeg(self):
        """MJPEG generator for Flask streaming route."""
        while self.running:
            frame_to_encode = None
            with self.lock:
                if self.current_frame is not None:
                    frame_to_encode = self.current_frame.copy()

            if frame_to_encode is not None:
                try:
                    ret, jpeg = cv2.imencode('.jpg', frame_to_encode, [cv2.IMWRITE_JPEG_QUALITY, 75])
                    if ret:
                        yield (b'--frame\r\n'
                               b'Content-Type: image/jpeg\r\n\r\n' +
                               jpeg.tobytes() + b'\r\n')
                except Exception:
                    pass

            time.sleep(self.frame_interval)

    def capture_snapshot(self) -> bytes:
        """Capture single snapshot as JPEG."""
        with self.lock:
            if self.current_frame is not None:
                ret, jpeg = cv2.imencode('.jpg', self.current_frame, [cv2.IMWRITE_JPEG_QUALITY, 90])
                if ret:
                    return jpeg.tobytes()
        return b''

    def close(self):
        """Release camera resources."""
        self.running = False
        if self.cam_obj:
            try:
                if self.camera_type == "picamera2":
                    self.cam_obj.stop()
                    self.cam_obj.close()
                elif self.camera_type == "opencv":
                    self.cam_obj.release()
            except Exception:
                pass
