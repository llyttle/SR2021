#!/usr/bin/env python3
import numpy as np
import cv2
import RPi.GPIO as gpio
import time


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
    
    def get_Centroid(self):
        hsv = cv2.cvtColor(self.img, cv2.COLOR_BGR2HSV)

        # Creating color mask, 
        mask = cv2.inRange(hsv, self.lower_red, self.upper_red)
        res = cv2.bitwise_and(self.img, self.img, mask=mask)
        # Smoothing mask
        median = cv2.medianBlur(res, 15)
        # Finding object edges
        edges = cv2.Canny(median, 100, 100)
        # Finding centroid of object
        __, thresh = cv2.threshold(edges,127,255,0)
        M = cv2.moments(thresh)
        
        if M["m10"] == 0 or M["m00"] == 0:
            self.cX = None
        else:
            self.cX = int(M["m10"] / M["m00"])
        # cY = int(M["m01"] / M["m00"])

    def send_msg(self, msg):
        # Map image X coordinate to 0-100% duty cycle
        dc = msg * (100/640)
        self.arduinoSignal.start(round(dc))
        time.sleep(.1)
        self.arduinoSignal.start(0)
        print(dc)
    
    def run(self):
        while True:
            cap = cv2.VideoCapture(0)
            __,raw_img = cap.read()
            self.img = cv2.flip(cv2.flip(raw_img,0), 1)

            self.get_Centroid()
            if self.cX:
                self.send_msg(self.cX)
            else:
                print("No Object in Sight")

            # Cleanup camera and gpio
            cap.release()
        self.arduinoSignal.stop()
        gpio.cleanup()

if __name__ == '__main__':
    node = Camera()
    node.run()