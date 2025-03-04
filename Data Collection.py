import cv2
import pandas as pd
from ultralytics import YOLO

# Initialize
model = YOLO("yolov8s-pose.pt")  # Download from Ultralytics assets
ACTIVITY_LABELS = {"Normal": 0, "Suspicious": 1}

def extract_keypoints(video_path, label):
    cap = cv2.VideoCapture(video_path)
    data = []
    
    while cap.isOpened():
        ret, frame = cap.read()
        if not ret: break
        
        results = model(frame, verbose=False)
        for r in results:
            keypoints = r.keypoints.xyn.cpu().tolist()
            for person in keypoints:
                entry = {"label": ACTIVITY_LABELS[label]}
                for i, (x, y) in enumerate(person):
                    entry.update({f"x{i}": x, f"y{i}": y})
                data.append(entry)
    
    cap.release()
    return pd.DataFrame(data)

# Process your labeled videos
normal_df = extract_keypoints("normal_videos1.mp4", "Normal")
normal_df = extract_keypoints("normal_videos2.mp4", "Normal")
normal_df = extract_keypoints("normal_videos3.mp4", "Normal")
normal_df = extract_keypoints("normal_videos4.mp4", "Normal")
suspicious_df = extract_keypoints("suspicious_videos.mp4", "Suspicious")

# Combine & save
combined = pd.concat([normal_df, suspicious_df])
combined.to_csv("pose_dataset.csv", index=False)
