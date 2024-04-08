from ultralytics import YOLO
import cv2
import math 

'''
    This is the code that runs the object detection model on the webcam
'''

# start webcam
cap = cv2.VideoCapture(0)
cap.set(3, 640)
cap.set(4, 480)

# model
model = YOLO("runs/detect/train6/weights/best.pt")

# this gets the classes the model was trained on
classNames = model.names


while True:
    success, img = cap.read()
    results = model(img, imgsz = 640, stream=True)

    # coordinates of the bounding box
    for r in results:
        boxes = r.boxes

        for box in boxes:
            # bounding box coordinates
            x1, y1, x2, y2 = box.xyxy[0]
            x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2) # convert to int values

            # put box in cam 
            cv2.rectangle(img, (x1, y1), (x2, y2), (255, 0, 255), 3)

            # confidence of the object
            confidence = math.ceil((box.conf[0]*100))/100
            print("Confidence --->",confidence)

            # class name
            cls = int(box.cls[0])
            print("Class name -->", classNames[cls])

            # object details
            org = [x1, y1]
            font = cv2.FONT_HERSHEY_SIMPLEX
            fontScale = 1
            color = (255, 0, 0)
            thickness = 2

            cv2.putText(img, classNames[cls], org, font, fontScale, color, thickness)

    cv2.imshow('Webcam', img)
    # press q to quit
    if cv2.waitKey(1) == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()


'''
    An Alternative way to run the model on the webcam is:

    results = model.predict(source = 0, show = True, imgsz = 640, conf = 0.5)

    This code will run the model on the webcam and display the results on the screen, note that pressing q will not quit for this code
'''