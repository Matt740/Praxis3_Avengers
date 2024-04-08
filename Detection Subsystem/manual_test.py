from ultralytics import YOLO
import os

# This is the directory where the test images are stored
directory = "manual_test_images/phone"

# This is a list of the images in the directory
print(os.listdir(directory))

# This lets the program access the model that you desire to test
model = YOLO("runs/detect/train16/weights/best.pt")

# Example code of testing the model on a single image
# results = model.predict("manual_test_images/general_test3.jpeg", save = True, imgsz = 640, conf = 0.5)

# This is the loop that goes through all the images in the directory
for img in os.listdir(directory):

    # This is the code that runs the prediction on the image, the resulting image is saved and can be used to manually assess the model
    results = model.predict(f"{directory}/{img}", save = True, imgsz = 640, conf = 0.5, show = True)
    print(f"The results are {len(results[0].boxes.xyxy)}") # number of objects detected, useful because our object detection model is binary
    print(results.json())
    print(f"{img} done")
    print("\n\n\n\n\n")