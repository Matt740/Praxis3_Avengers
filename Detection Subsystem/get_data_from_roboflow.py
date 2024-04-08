from roboflow import Roboflow

# The code below is used to download the dataset from Roboflow
# Replace YOUR_API_KEY with your actual API key
rf = Roboflow(api_key="YOUR_API_KEY")
project = rf.workspace("computervisionprojects-qk1pn").project("praxis-iii-e-waste-detection")
version = project.version(1)
dataset = version.download("yolov8")





