#!/usr/bin/env python3
import numpy as np
import cv2
import RPi.GPIO as gpio
import time
import math

"""Camera Res = 640x480"""
class Camera(object):
    def __init__(self):
        gpio.setwarnings(False)
        gpio.setmode(gpio.BCM)

        arduinoPin = 26
        gpio.setup(arduinoPin, gpio.OUT)
        self.arduinoSignal = gpio.PWM(arduinoPin, 50)

        #hsv: hue sat val
        self.lower_red = np.array([140,100,0])
        self.upper_red = np.array([180,255,255])

        self.cX = 0
        self.cY = 0
        self.theta = 0
        
        self.timer = 0
    
    def get_Centroid(self):
        hsv = cv2.cvtColor(self.img, cv2.COLOR_BGR2HSV)

        # Creating color mask, 
        mask = cv2.inRange(hsv, self.lower_red, self.upper_red)
        # Smoothing mask
        median = cv2.medianBlur(mask, 15)
        # Finding centroid of object
        __, thresh = cv2.threshold(median,127,255,0)
        M = cv2.moments(thresh)
        
        if M["m10"] == 0 or M["m01"] == 0 or M["m00"] == 0:
            self.cX = None
            self.cY = None
        else:
            self.cX = int(M["m10"] / M["m00"])
            self.cY = int(M["m01"] / M["m00"])

    def centroid_cart2pol(self):
        x = self.cX - 320
        y = self.cY - 240
        theta = math.atan2(x, y)

        if theta >= 0:
            self.theta = theta
        else:
            self.theta = theta + (math.pi*2)

    def send_msg(self, msg):
        # Map image X coordinate to 0-100% duty cycle
        dc = msg * (100/(2*math.pi))
        print(dc)
        self.arduinoSignal.start(round(dc))
        time.sleep(.03)
        self.arduinoSignal.start(0)
        

    def run(self):
        cap = cv2.VideoCapture(0)
        while True:
            while self.timer < 4:
                __,self.img = cap.read()
                # self.img = cv2.flip(cv2.flip(raw_img,0), 1)
                self.timer += 1

            self.timer = 0

            self.get_Centroid()
            if self.cX and self.cY:
                self.centroid_cart2pol()
                self.send_msg(self.theta)
            else:
                print("No Object in Sight")

            # Cleanup camera and gpio
        cap.release()
        self.arduinoSignal.stop()
        gpio.cleanup()

if __name__ == '__main__':
    node = Camera()
    node.run()