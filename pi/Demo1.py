# Luke Walker and Aidan Paul
#EENG 350 - SEED Lab
# Mini Project
# October 2, 2023

import time
import board
import adafruit_character_lcd.character_lcd_rgb_i2c as character_lcd
import threading
import numpy as np
import cv2
import smbus2

from smbus2 import SMBus
from time import sleep
from cv2 import aruco

lcdlock = threading.Lock()

mtx = np.array([[643.93340907,0,319.16113039],
                [0,644.2645585,242.2359518 ],
                [0,0,1]])

dist = np.array([[ 0.03804058, -0.36938373,  0.00158751, 0.0083502, -0.36594211]])

def lcdPrint():
    
    # Display LCD
    lcdlock.acquire()
    lcd.clear()
    lcd.color = [100,0,0]
    time.sleep(.1)
    lcd.message = "Angle:\n"+str(angleDegree)
    lcdlock.release()

# I2C address of the Arduino, set in Arduino sketch
ARD_ADDR = 8

#Initialise I2C bus
lcd_columns = 16
lcd_rows = 2
i2c = board.I2C()
smbus = SMBus(1)
lcd = character_lcd.Character_LCD_RGB_I2C(i2c,lcd_columns, lcd_rows)
lcd.clear()
	
# initialize the camera. If channel 0 doesn't work, try channel 1
aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_50)

#Define camera image properties
Xres = 424
Yres = 240
camera = cv2.VideoCapture(0)
camera.set(cv2.CAP_PROP_FRAME_WIDTH, Xres)
camera.set(cv2.CAP_PROP_FRAME_HEIGHT, Yres)

# Let the camera warmup
sleep(0.5)
angleOld = 0
	
# Continuously get an image from the camera stream
while(True):
        # Get image
        ret, frame = camera.read()
        sleep(0.01) # wait for image to stabilize
        if not ret:
                print("Could not capture image from camera!")
                quit()
        else:
                # Save the image to the disk
                if(True):

                        # Make the image greyscale for ArUco detection
                        grey = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
                        cv2.imshow("overlay",grey)

                        #UNDISTORT
                        h, w = grey.shape[:2]
                        newcameramtx, roi = cv2.getOptimalNewCameraMatrix(mtx, dist, (w,h), 1, (w, h))

                        dst = cv2.undistort(grey, mtx, dist, None, newcameramtx)

                        x,y,w,h = roi
                        dst = dst[y:y+h, x:x+w]
                        
                        # Detecting ArUco Marker
                        corners,ids,rejected = aruco.detectMarkers(grey,aruco_dict)

                        # Convert to Greyscale
                        overlay = cv2.cvtColor(grey,cv2.COLOR_GRAY2RGB) # Convert back to RGB for imshow, as well as for the next step
                        overlay = aruco.drawDetectedMarkers(overlay,corners,borderColor = 4)

                        # Draw lines
                        # cv2.line(overlay,(0, int(Yres/2)),(int(Xres), int(Yres/2)),(0,255,0),1)
                        cv2.line(overlay,(int(Xres/2),0),(int(Xres/2), int(Yres)),(0,255,0),1)

                        
                        # If marker found, show marker on frame
                        if not ids is None:
                                ids = ids.flatten()
                                for (outline, id) in zip(corners, ids):
                                        markerCorners = outline.reshape((4,2))
                                        overlay = cv2.putText(overlay, str(id),(int(markerCorners[0,0]), int
                                                (markerCorners[0,1]) - 15),cv2.FONT_HERSHEY_SIMPLEX,0.5, (255,0,0), 2)

                                #Find the center value using corners
                                actualCorners = corners[0][0] #Gets indexed list of just the 4 corner locations, reduced number of indexing                                
                                topLeftX = actualCorners[0][0]
                                topLeftY = actualCorners[0][1]
                                botRightX = actualCorners[2][0]
                                botRightY = actualCorners[2][1]

                                # Center Point
                                Xcenter = (topLeftX+botRightX)/2
                                Ycenter = (topLeftY+botRightY)/2

                                # Find angle
                                FOVangle = -30
                                angleDegree = ((Xcenter-(Xres/2))/(Xres/2))*FOVangle

                                if (angleDegree > angleOld +.1 or angleDegree < angleOld - .1) and not lcdlock.locked():
                                    # Display LCD
                                    myThread = threading.Thread(target=lcdPrint)
                                    myThread.start()
                                angleOld = angleDegree
                                
                                                                
                        cv2.imshow("overlay",overlay)
                        k = cv2.waitKey(1) & 0xFF
                        if k == ord('q'):
                                break

camera.release()
cv2.destroyAllWindows()
