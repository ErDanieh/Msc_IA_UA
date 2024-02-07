import cv2
import urllib.request
import numpy as np
from ultralytics import YOLO
import time
import csv
from datetime import datetime

csv_filename = "database.csv"

yolo_net = YOLO("yolov8n.pt")  # load a pretrained model (recommended for training)

# URL of the MJPEG video stream
url = "http://83.56.31.69/mjpg/video.mjpg"

# Open the video stream
stream = urllib.request.urlopen(url)

# Create a VideoCapture object to read frames from the stream
cap = cv2.VideoCapture()

# Set the VideoCapture object to use the MJPEG stream
cap.open(url)

# Check if the stream is opened successfully
if not cap.isOpened():
    print("Error: Could not open the stream.")
    exit(1)
    


while True:
    # Read a frame from the stream
    ret, frame = cap.read()

    # Check if the frame was read successfully
    if not ret:
        print("Error: Could not read frame.")
        break

    # Perform person detection using YOLOv8
    #blob = cv2.dnn.blobFromImage(frame, 1/255.0, (416, 416), swapRB=True, crop=False)
    detections = yolo_net.predict(source=frame)

    person_count = 0
    for detection in detections[0]:
        class_id = detection.boxes.cls.item()
        confidence = detection.boxes.conf.item()

        # Check if the detected object is a person (class_id 0)
        if class_id == 0 and confidence > 0.5:
            # Extract bounding box coordinates
            center_x, center_y, width, height = detection.boxes.xywh[0].tolist()
            x, y = (int(center_x - width / 2), int(center_y - height / 2))
            p1 = (x, y)
            p2=(int(x + width), int(y + height))

            # Draw a bounding box around the person
            cv2.rectangle(img=frame, pt1=p1, pt2=p2, color=(0, 255, 0), thickness=2)
            person_count = person_count + 1
            
    # Get the current date and time with seconds precision
    timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    
    # Create or open the CSV file for writing
    with open(csv_filename, mode='a', newline='') as csv_file:
        fieldnames = ["Timestamp", "Person_Count"]
        writer = csv.DictWriter(csv_file, fieldnames=fieldnames)

        # Write the timestamp and person count to the CSV file
        writer.writerow({"Timestamp": timestamp, "Person_Count": person_count})

    # Display the frame with detected persons
    #cv2.imshow("Person Detection", frame)
    cv2.imwrite('processed_frame.jpg', frame)
    
    # Press 'q' to exit the loop
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
    
    time.sleep(0.5)


# Release the VideoCapture and close the window
cap.release()
cv2.destroyAllWindows()
