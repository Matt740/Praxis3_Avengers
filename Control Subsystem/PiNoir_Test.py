from PI4_sensor_library import PiNoir

camera = PiNoir("test.jpg")

if camera.capture_image():
    print("Success")
else:
    print("Uh Oh")