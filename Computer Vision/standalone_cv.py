import cv2
# Uncomment below when ultralytics is installed
# from ultralytics import YOLO

def main():
    print("Memulai Standalone CV (Tanpa ROS 2)...")
    
    # Inisialisasi model YOLO (Placeholder)
    # model = YOLO('yolov8n.pt') 
    print("Model YOLO siap (placeholder).")

    # Buka webcam lokal (0 adalah default camera)
    cap = cv2.VideoCapture(0)

    if not cap.isOpened():
        print("Error: Tidak dapat membuka webcam.")
        return

    print("Tekan 'q' pada jendela video untuk keluar.")

    while True:
        ret, frame = cap.read()
        if not ret:
            print("Error: Tidak dapat membaca frame.")
            break

        # --- YOLO Inference Placeholder ---
        # results = model(frame)
        # 
        # for r in results:
        #     boxes = r.boxes
        #     for box in boxes:
        #         # Koordinat Bounding box
        #         x1, y1, x2, y2 = box.xyxy[0]
        #         x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)
        #         
        #         # Gambar bounding box
        #         cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 3)
        #         
        #         # Gambar label (Placeholder)
        #         cv2.putText(frame, "Object", (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)
        
        # Tampilkan video
        cv2.imshow("Standalone CV Test - YOLO", frame)

        # Tekan 'q' untuk keluar
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # Bersihkan resources
    cap.release()
    cv2.destroyAllWindows()
    print("Standalone CV selesai.")

if __name__ == '__main__':
    main()
