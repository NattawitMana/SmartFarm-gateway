import warnings
import os
import platform
import pathlib
import torch
import cv2
import requests
from flask import Flask, jsonify, Response

# Line Notify token
LINE_NOTIFY_TOKEN = "5csNQjR898Iev5G5ENGXP3q7QmNr6l5ZDZBvjHhwLhf"

app = Flask(__name__)

# Load custom weights
weights_path = os.path.join(os.path.dirname(__file__), 'best_10.pt')
if not os.path.exists(weights_path):
    print(f"Error: Weights file not found at {weights_path}")
    exit()

# Load the YOLOv5 model
model = torch.hub.load('./yolov5', "custom", path=weights_path, source="local")

# Open webcam
cap = cv2.VideoCapture(0)
if not cap.isOpened():
    print("Error: Could not open webcam.")
    exit()

# Confidence threshold
CONFIDENCE_THRESHOLD = 0.60  # Adjust as needed

@app.route('/api/detect', methods=['GET'])
def detect_objects():
    ret, frame = cap.read()
    if not ret:
        return jsonify({"error": "Failed to grab frame."}), 400

    # Perform inference
    results = model(frame)

    # Filter results based on confidence threshold
    detections = results.pandas().xyxy[0]  # Get detections as a DataFrame
    detections = detections[detections['confidence'] > CONFIDENCE_THRESHOLD]

    objects = []
    for _, row in detections.iterrows():
        label = row['name']
        confidence = row['confidence']
        objects.append({
            "label": label,
            "confidence": confidence
        })

        # Send Line Notify message
        send_line_notify(f"Detected: {label} with confidence {confidence:.2f}")

    return jsonify({"objects": objects})

def send_line_notify(message):
    """Send message via Line Notify"""
    url = "https://notify-api.line.me/api/notify"
    headers = {
        "Authorization": f"Bearer {LINE_NOTIFY_TOKEN}"
    }
    data = {
        "message": message
    }
    response = requests.post(url, headers=headers, data=data)
    if response.status_code != 200:
        print("Failed to send Line Notify:", response.text)

if __name__ == "__main__":
    app.run(debug=True)