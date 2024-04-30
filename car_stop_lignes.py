from threading import Timer
import logging
import cv2
import datetime
import serial
import struct
from time import sleep
from picamera2 import Picamera2
from ultralytics import YOLO
import pandas as pd 
import cvzone
import numpy as np
from lane_follower_test import HandCodedLaneFollower


_SHOW_IMAGE = True  # Variable pour afficher les images

#liaison en série avec arduino 
ser = serial.Serial('/dev/ttyACM0', 9600, timeout=1) 

class LittleBerryCar(object):
    """Classe pour contrôler LittleBerryCar"""

    __INITIAL_SPEED = 0  # Vitesse initiale du véhicule
    __SCREEN_WIDTH = 640  # Largeur de l'écran de la caméra
    __SCREEN_HEIGHT = 480  # Hauteur de l'écran de la caméra
    __SPEED_TURNING = 100  # Vitesse de virage

    def __init__(self):
        """Initialisation de LittleBerryCar"""
        logging.info('Starting LittleBerryCar')

        logging.debug('Set up camera') 

        #config caméra        
        self.picam2 = Picamera2()
        self.picam2.preview_configuration.main.size = (self.__SCREEN_WIDTH, self.__SCREEN_HEIGHT)
        self.picam2.preview_configuration.main.format = "RGB888"
        self.picam2.preview_configuration.align()
        self.picam2.configure("preview")
        self.picam2.start()

        self.model = YOLO('best.pt')

        #fonction de détection des lignes
        self.lane_follower_test = HandCodedLaneFollower(self)
        
        self.fourcc = cv2.VideoWriter_fourcc(*'XVID')  # Initialisation du code vidéo
        datestr = datetime.datetime.now().strftime("%y%m%d_%H%M%S")  # Date et heure
        self.video_orig = cv2.VideoWriter('../data/tmp/car_video%s.avi' % datestr, self.fourcc, 20.0, (self.__SCREEN_WIDTH, self.__SCREEN_HEIGHT))  # Enregistreur vidéo original
        self.video_lane = cv2.VideoWriter('../data/tmp/car_video_lane%s.avi' % datestr, self.fourcc, 20.0, (self.__SCREEN_WIDTH, self.__SCREEN_HEIGHT))  # Enregistreur vidéo avec line

        logging.debug('Setup Speed') 
        self.speedTurning = self.__SPEED_TURNING  # Initialisation de la vitesse de virage
        self.speed = self.__INITIAL_SPEED  # Initialisation de la vitesse


    def __enter__(self):
        """Entrée dans une instruction 'with'"""
        return self


    def __exit__(self, _type, value, traceback):
        """Sortie d'une instruction 'with'"""
        if traceback is not None:
            # Une exception s'est produite :
            logging.error('Exiting with statement with exception %s' % traceback)

        self.cleanup()  # Nettoyage des ressources


    def cleanup(self):
        """Réinitialise le matériel"""
        logging.info('Stopping the car, resetting hardware.') 

        ser.write('S'.encode()) #envoi S à l'arduino pour stopper la voiture

        self.picam2.stop()
        self.video_orig.release()  # Libération ressources enregistreur vidéo original
        self.video_lane.release()  # Libération ressources enregistreur vidéo ligne
        cv2.destroyAllWindows()  # Fermeture fenêtres OpenCV


    def drive(self, speed=__INITIAL_SPEED):
        """ Point d'entrée principal du véhicule, et le met en mode conduite"""

        ser.write('F'.encode()) #envoi F à l'arduino pour faire avancer la voiture

        my_file = open("Travaux.txt", "r")
        data = my_file.read()
        count=0
        class_list = data.split("\n")   
        logging.info('Starting to drive at speed %s...' % speed)

        while True:
            ser.write('F'.encode()) #envoi F à l'arduino pour faire avancer la voiture

            im = self.picam2.capture_array()
            cv2.imshow("Camera", im)

            results = self.model.predict(im)
            a = results[0].boxes.data
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

            self.video_orig.write(im)  # Enregistrement de l'image originale 

            if len(a)>0: #si on détecte un panneau
                """Arrête la voiture"""
                logging.info('Stopping the car.')

                ser.write('S'.encode()) #envoi S à l'arduino pour stopper la voiture

                ser.write('F'.encode()) #envoi F à l'arduino pour faire avancer la voiture
        
            #test de détection des lignes (envois R ou L à l'arduino)
            image = self.lane_follower_test.follow_lane(im)

            if cv2.waitKey(1) & 0xFF == ord('q'):  # Si la touche 'q' est enfoncée
                self.cleanup()  # Nettoyage
                #break  # Sortie de la boucle


    ############################
    # Fonctions utilitaires
    ############################
def show_image(title, frame, show=_SHOW_IMAGE):
        """Affiche une image"""
        if show:
            cv2.imshow(title, frame)
        
if __name__ == '__main__':
        logging.basicConfig(level=logging.DEBUG, format='%(levelname)-5s:%(asctime)s: %(message)s')  # Configuration du journalisation
        with LittleBerryCar() as car:  # Utilisation de la classe LittleBerryCar dans une instruction 'with'
            car.drive(100)  # Démarre la conduite
