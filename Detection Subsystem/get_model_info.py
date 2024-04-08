from ultralytics import YOLO

# This is the code showing how to get the class names of a trained model
# Note, train2 means that this is the second training of the model
model = YOLO("runs/detect/train2/weights/best.pt")

print(model.names)