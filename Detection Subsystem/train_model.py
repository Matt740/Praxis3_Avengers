from ultralytics import YOLO

model = YOLO("yolov8s.pt")

results = model.train(data = "E_Waste_Detection-2/data.yaml", plots = True, epochs = 1)

