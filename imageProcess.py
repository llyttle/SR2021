#!/usr/bin/env python3
import numpy as np
import cv2

cap = cv2.VideoCapture(0)
__,img = cap.read()
img = cv2.flip(cv2.flip(img,0), 1)

#img = cv2.imread("photos/camera1.jpg", 1)

hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

#hsv: hue sat val
lower_red = np.array([140,100,0])
upper_red = np.array([180,255,255])

mask = cv2.inRange(hsv, lower_red, upper_red)
res = cv2.bitwise_and(img, img, mask=mask)

median = cv2.medianBlur(res, 15)

edges = cv2.Canny(median, 100, 100)

highlight = cv2.bitwise_and(img, img, mask=cv2.bitwise_not(edges))

__, thresh = cv2.threshold(edges,127,255,0)

M = cv2.moments(thresh)

cX = int(M["m10"] / M["m00"])
cY = int(M["m01"] / M["m00"])

# put text and highlight the center
cv2.circle(highlight, (cX, cY), 5, (255, 255, 255), -1)
cv2.putText(highlight, "centroid", (cX - 25, cY - 25),cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
cv2.circle(highlight, (640, 480), 5, (255, 50, 50), -1)

cv2.imwrite("photos/filtered.jpg", highlight)

print(cX-320, cY-240)

cap.release()
