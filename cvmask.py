import cv2 as cv
import numpy as np
from picamera import PiCamera
from time import sleep

mtx = np.fromfile('/home/pi/comp_vis/mtx.npy').reshape(3,3)
dist = np.fromfile('/home/pi/comp_vis/dist.npy')
width = 512
height = 288
#width = 1024
#height = 576
camera = PiCamera()
camera.resolution = (width,height)
camera.framerate = 30
camera.iso = 100
sleep(2)
camera.shutter_speed = camera.exposure_speed
camera.exposure_mode = 'off'
g = camera.awb_gains 
camera.awb_mode = 'off'
camera.awb_gains = g
def nothing(x):
    pass

camera.capture('/home/pi/comp_vis/Assignment2/cali.jpg')
img2=cv.imread('/home/pi/comp_vis/Assignment2/cali.jpg')
h,w=img2.shape[:2]
ncm,roi=cv.getOptimalNewCameraMatrix(mtx,dist,(w,h),1,(w,h))
new = cv.undistort(img2,mtx,dist,None,ncm)
x,y,w,h = roi
global wi
wi=w+1
global hi
hi=h
new=new[y:y+h,x:x+w]
cv.imwrite('/home/pi/comp_vis/new.jpg',new)
img = cv.imread('/home/pi/comp_vis/new.jpg')
#cv.imshow('first', img)
hsv = cv.cvtColor(img,cv.COLOR_BGR2HSV)
    
cv.namedWindow('blue')
cv.createTrackbar('H low', 'blue', 96, 255, nothing)
cv.createTrackbar('H hi', 'blue', 135, 255, nothing)
cv.createTrackbar('S low', 'blue', 105, 255, nothing)
cv.createTrackbar('S hi', 'blue', 255, 255, nothing)
cv.createTrackbar('V low', 'blue', 65, 255, nothing)
cv.createTrackbar('V hi', 'blue', 255, 255, nothing)

while(1):
    try:
        cv.imshow('blue', out)
        cv.imshow('first', img)
    except:
        pass
    
    Hhi = cv.getTrackbarPos('H hi', 'blue')
    Hlo = cv.getTrackbarPos('H low', 'blue')
    Shi = cv.getTrackbarPos('S hi', 'blue')
    Slo = cv.getTrackbarPos('S low', 'blue')
    Vhi = cv.getTrackbarPos('V hi', 'blue')
    Vlo = cv.getTrackbarPos('V low', 'blue')
    
    lowerbound = (Hlo,Slo,Vlo)
    upperbound = (Hhi,Shi,Vhi)
    
    mask = cv.inRange(hsv, lowerbound, upperbound)
    out = cv.bitwise_and(img,img,mask=mask)
    
    k=cv.waitKey(1)& 0xFF
    if k == 27:
        break
    
cv.destroyAllWindows()
