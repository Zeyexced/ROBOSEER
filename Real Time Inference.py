import cv2
import pandas as pd
import xgboost as xgb
from ultralytics import YOLO

# Load models
pose_model = YOLO("yolov8s-pose.pt")
classifier = xgb.XGBClassifier()
classifier.load_model("behavior_model.ubj")

cap = cv2.VideoCapture("normal_videos1.mp4")  # Webcam

while True:
    ret, frame = cap.read()
    if not ret: break
    
    results = pose_model(frame, verbose=False)
    for r in results:
        keypoints = r.keypoints.xyn.cpu().tolist()
        boxes = r.boxes.xyxy.cpu().tolist()
        
        for box, person in zip(boxes, keypoints):
            # Prepare features
            features = {}
            for i, (x, y) in enumerate(person):
                features.update({f"x{i}": x, f"y{i}": y})
            
            # Predict
            pred = classifier.predict(pd.DataFrame([features]))[0]
            label = "Normal" if pred == 0 else "SUSPICIOUS"
            color = (0, 255, 0) if pred == 0 else (0, 0, 255)
            
            # Draw
            x1, y1, x2, y2 = map(int, box)
            cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
            cv2.putText(frame, label, (x1, y1-10), 
                       cv2.FONT_HERSHEY_SIMPLEX, 1, color, 2)
    
    cv2.imshow("Behavior Monitor", frame)
    if cv2.waitKey(1) == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()