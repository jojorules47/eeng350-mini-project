import cv2 as cv
from picamera import PiCamera
from time import sleep
import numpy as np
import sys

#Camera Setup

width = 512
height = 288
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

#Color Mask

lowColor = np.array([60,128,70],np.uint8)
upColor = np.array([80,255,255],np.uint8)
kernel = np.ones((5,5),np.uint8)

#Pic Functions

def pic():
    global name
    name = input("Input a name for your image: ")

    camera.start_preview()
    sleep(5)
    camera.capture('/home/pi/comp_vis/MiniProject/%s.jpg' % (name))
    camera.stop_preview()

def constant():
    camera.capture('/home/pi/comp_vis/MiniProject/constant.jpg')
    
#Image processing test
    
pic()
img2 = cv.imread('/home/pi/comp_vis/MiniProject/%s.jpg' % (name),1)
cv.imshow('pic', img2)
cv.waitKey(0)
cv.destroyAllWindows()
hsv = cv.cvtColor(img2,cv.COLOR_BGR2HSV)
cv.imshow('hsv', hsv)
cv.waitKey(0)
cv.destroyAllWindows()
mask = cv.inRange(hsv,lowColor,upColor)
final = cv.bitwise_and(img2,img2, mask = mask)
cv.imshow('Final', final)
cv.waitKey(0)
cv.destroyAllWindows()
kernel = np.ones((5,5),np.uint8)
clean = cv.blur(final,(3,3))
clean2 = cv.morphologyEx(clean, cv.MORPH_OPEN, kernel)
clean3 = cv.morphologyEx(clean2, cv.MORPH_CLOSE, kernel)
cv.imshow('Clean', clean3)
cv.waitKey(0)
cv.destroyAllWindows()
grey = cv.cvtColor(clean3,cv.COLOR_BGR2GRAY)

cv.imshow("contours", grey)
cv.waitKey(0)
cv.destroyAllWindows()
    

ret,thresh = cv.threshold(grey,43,255,cv.THRESH_BINARY)
img0,contours,heirarchy = cv.findContours(thresh,cv.RETR_TREE,cv.CHAIN_APPROX_SIMPLE)
cv.imshow("contours", img0)
cv.waitKey(0)
cv.destroyAllWindows()
   
print("The program will now find the quadrant the marker is in. Press Ctl+c to exit.")

try:
    while(True):
        constant()
        c = cv.imread('/home/pi/comp_vis/MiniProject/constant.jpg')
        hsv_c = cv.cvtColor(c,cv.COLOR_BGR2HSV)
        mask = cv.inRange(hsv_c,lowColor,upColor)
        final_c = cv.bitwise_and(c,c, mask = mask)
        clean_c = cv.blur(final_c,(3,3))
        clean2_c = cv.morphologyEx(clean_c, cv.MORPH_OPEN, kernel)
        clean3_c = cv.morphologyEx(clean2_c, cv.MORPH_CLOSE, kernel)
        grey_c = cv.cvtColor(clean3_c,cv.COLOR_BGR2GRAY)
        retc,thresh_c = cv.threshold(grey_c,43,255,cv.THRESH_BINARY)
        img0c,contours_c,heirarchy_c = cv.findContours(thresh_c,cv.RETR_TREE,cv.CHAIN_APPROX_SIMPLE)
        
        
        try:
            cnt_c = contours_c[0]
            M_c = cv.moments(cnt_c)
            cxc = int(M_c['m10']/M_c['m00'])
            cyc = int(M_c['m01']/M_c['m00'])
            
            
            fov = 62.2
            if(cxc<(width/2))and(cyc<(height/2)):
                Quad = 0
                print("Zone %d" % Quad)
            elif(cxc>(width/2))and(cyc<(height/2)):
                Quad = 1
                print("Zone %d" % Quad)
            elif(cxc<(width/2))and(cyc>(height/2)):
                Quad = 2
                print("Zone %d" % Quad)
            else:
                Quad = 3
                print("Zone %d" % Quad)
                
        except IndexError:
            print("No markers found.")
except KeyboardInterrupt:
    pass
    
print("Done!")