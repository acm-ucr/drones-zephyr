import cv2
import numpy as np
import queue
import threading
from ultralytics import YOLO
import paho.mqtt.client as mqtt

MQTT_BROKER = "broker.emqx.io"
MQTT_PORT = 1883
MQTT_TOPIC = "drone/zephyrcommands"
client = mqtt.Client()
client.connect(MQTT_BROKER, MQTT_PORT, 60)
client.loop_start()
model = YOLO("model/best.pt")

frame_queue = queue.Queue(maxsize=10)

running = True

# Function to capture frames from the webcam
def get_frames():
    global running
    cap = cv2.VideoCapture(0)
    while running:
        ret, frame = cap.read()
        if not ret:
            continue
        if not frame_queue.full():
            frame_queue.put(frame)
    cap.release()

# Function to process frames from the queue
def process_frames():
    global running
    while running:
        if not frame_queue.empty():
            frame = frame_queue.get()

            if frame is None or frame.size == 0:
                continue

            # YOLOv8 inference
            results = model(frame, conf=0.4, iou=0.5)

            for result in results:
                boxes = result.boxes
                largest_box = None
                largest_area = 0

                box_data = []  # Store all box info for drawing later

                for box in boxes:
                    x1, y1, x2, y2 = map(int, box.xyxy[0].tolist())
                    conf = float(box.conf[0])
                    cls_id = int(box.cls[0])
                    area = (x2 - x1) * (y2 - y1)

                    box_data.append({
                        "coords": (x1, y1, x2, y2),
                        "conf": conf,
                        "cls_id": cls_id,
                        "area": area
                    })

                    if area > largest_area:
                        largest_area = area
                        largest_box = (x1, y1, x2, y2)
                        
                if largest_box != None:
                    box_center = (largest_box[0] + largest_box[2])/2
                    if box_center < frame.shape[1] / 2:
                        cv2.putText(frame, "Turn Right", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                        client.publish(MQTT_TOPIC, "Right")
                    else:
                        cv2.putText(frame, "Turn Left", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                        client.publish(MQTT_TOPIC, "Left")
                    
                # Draw bounding boxes
                for data in box_data:
                    x1, y1, x2, y2 = data["coords"]
                    conf = data["conf"]
                    cls_id = data["cls_id"]
                    class_name = model.names[cls_id]
                    label = f"{class_name} ({conf:.2f})"

                    # Largest box is red, others are green
                    color = (0, 0, 255)
                    if (x1, y1, x2, y2) != largest_box:
                        color = (0, 255, 0)
                    thickness = 3 
                    if (x1, y1, x2, y2) == largest_box:
                        thickness = 2

                    cv2.rectangle(frame, (x1, y1), (x2, y2), color, thickness)
                    cv2.putText(frame, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

            cv2.imshow("YOLOv8 - Cone Detection", frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):  
                break

    cv2.destroyAllWindows()


thread1 = threading.Thread(target=get_frames)
thread2 = threading.Thread(target=process_frames)

thread1.start()
thread2.start()

thread1.join()
thread2.join()
