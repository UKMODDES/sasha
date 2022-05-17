from tensorflow_object_detection import DetectorAPI
from PIL import Image
import time

print("Starting")
odapi = DetectorAPI(path_to_ckpt="/app/models/rcnn.pb")

image = Image.open("/app/person.jpeg")
print(odapi.process_frame(image))
print("Finished")
