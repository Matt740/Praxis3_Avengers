from ultralytics import YOLO

# This is the code that trains the model

# Choose the model you want to train (yolov8n.pt, yolov8s.pt, yolov8m.pt, yolov8l.pt, yolov8x.pt) for nano, small, medium, large, and extra large respectively
model = YOLO("yolov8m.pt")

# This is the code that trains the model with specific parameters
results = model.train(data = "datasets/data.yaml", plots = True, epochs = 100, device = 0, imgsz = 640)

