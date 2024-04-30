import cv2
from picamera2 import Picamera2
import pandas as pd
from ultralytics import YOLO
import cvzone
import numpy as np

#config camera
picam2 = Picamera2()
picam2.preview_configuration.main.size = (640,480)
picam2.preview_configuration.main.format = "RGB888"
picam2.preview_configuration.align()
picam2.configure("preview")
picam2.start()

#utilisation du model Yolov8
model=YOLO('bestTravaux.pt')
my_file = open("Travaux.txt", "r")
data = my_file.read()
class_list = data.split("\n")
count=0

while True:
    im= picam2.capture_array()
    
    count += 1
    if count % 20 != 0:
        continue

    results=model.predict(im)
    a=results[0].boxes.data
    px=pd.DataFrame(a).astype("float")
    
    for index,row in px.iterrows():
 
        x1=int(row[0])
        y1=int(row[1])
        x2=int(row[2])
        y2=int(row[3])
        d=int(row[5])
        c=class_list[d]

        cv2.rectangle(im,(x1,y1),(x2,y2),(0,0,255),2)
        cvzone.putTextRect(im,f'{c}',(x1,y1),1,1)
        
    cv2.imshow("Camera", im)
    if cv2.waitKey(1)==ord('q'):
        break
cv2.destroyAllWindows()
