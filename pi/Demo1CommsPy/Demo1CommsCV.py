# EENG350 Demo1 Reference Functions Task 1

import smbus
import time
import board
import adafruit_character_lcd.character_lcd_rgb_i2c as character_lcd

import cv2 as cv
from picamera import PiCamera
from time import sleep
import numpy as np
import sys

mtx = np.fromfile('mtx.npy').reshape(3,3)
dist = np.fromfile('dist.npy')

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

lowColor = np.array([100,25,50],np.uint8)
upColor = np.array([135,255,255],np.uint8)
#lowColor = np.array([100,100,100],np.uint8)
#upColor = np.array([120,255,255],np.uint8)
kernel = np.ones((5,5),np.uint8)

def constant():
    camera.capture('/home/pi/comp_vis/Assignment2/constant.jpg')
    new=cv.imread('/home/pi/comp_vis/Assignment2/constant.jpg')
    h,w=img.shape[:2]
    ncm,roi=cv.getOptimalNewCameraMatrix(mtx,dist,(w,h),1,(w,h))
    new = cv.undistort(img,mtx,dist,None,ncm)
    x,y,w,h = roi
    global wi
    wi=w+1
    new=new[y:y+h,x:x+w]
    cv.imwrite('/home/pi/comp_vis/new.jpg',new)

# I2C
bus = smbus.SMBus(1)
address = 0x04
i2c = board.I2C()  # uses board.SCL and board.SDA

# RPI - LCD
# Modify this if you have a different sized Character LCD
lcd_columns = 16
lcd_rows = 2
# Initialise the LCD class
lcd = character_lcd.Character_LCD_RGB_I2C(i2c, lcd_columns, lcd_rows)
# Set LCD color to whiteish
lcd.color = [200, 25, 25]

def none(tape): # no tape found
    lcd.text_direction = lcd.LEFT_TO_RIGHT
    lcd.message = "No Tape Found"
    if tape:
        lcd.clear()
        return found
    else:
        return none

def found(tape): # tape found
    lcd.text_direction = lcd.LEFT_TO_RIGHT
    lcd.message = "Tape Found!\nAngle: " + "{:.2f}".format(angle)
    
    if not tape:
        lcd.clear()
        return none
    else:
        return found
    
state = none

while(True):
    try:
        constant()
        img2 = cv.imread('/home/pi/comp_vis/new.jpg')
#        cv.imshow('pic', img2)
#        cv.waitKey(0)
#        cv.destroyAllWindows()
        hsv = cv.cvtColor(img2,cv.COLOR_BGR2HSV)
#        cv.imshow('hsv', hsv)
#        cv.waitKey(0)
#        cv.destroyAllWindows()
        mask = cv.inRange(hsv,lowColor,upColor)
        final = cv.bitwise_and(img2,img2, mask = mask)
#        cv.imshow('Final', final)
#        cv.waitKey(0)
#        cv.destroyAllWindows()
        kernel = np.ones((5,5),np.uint8)
        clean3 = cv.blur(final,(3,3))
#        clean2 = cv.morphologyEx(clean, cv.MORPH_OPEN, kernel)
#        clean3 = cv.morphologyEx(clean2, cv.MORPH_CLOSE, kernel)
#        cv.imshow('Clean', clean3)
#        cv.waitKey(0)
#        cv.destroyAllWindows()
        grey = cv.cvtColor(clean3,cv.COLOR_BGR2GRAY)

        #cv.imshow("contours", grey)
        #cv.waitKey(0)
        #cv.destroyAllWindows()
    

        ret,thresh = cv.threshold(grey,15,255,cv.THRESH_BINARY)
        img0,contours,heirarchy = cv.findContours(thresh,cv.RETR_TREE,cv.CHAIN_APPROX_SIMPLE)
#        cv.imshow("contours", img0)
#        cv.waitKey(0)
#        cv.destroyAllWindows()
        try:
            cnt_c = contours[0]
            M_c = cv.moments(cnt_c)
            if (M_c['m00'] == 0):
                continue
            cxc = int(M_c['m10']/M_c['m00'])
            cyc = int(M_c['m01']/M_c['m00'])
            
            tape = True
            state = state(tape)
#            print(cxc)
#            print(cyc)
            
            fov = 62.2
            fudge = 1.00
            if(cxc<(wi/2)): 
                angle = (fov/2)*fudge*(((wi/2)-cxc)/((wi/2)))
                print("The camera needs to turn {:.2f} degrees to center the marker." .format(angle))
            else:
                angle = -(fov/2)*((cxc-(wi/2))/(wi/2))
                print("The camera needs to turn {:.2f} degrees to center the marker." .format(angle))
#            cv.imshow("contours", img0)
#            cv.waitKey(0)
#            cv.destroyAllWindows()
            
        except IndexError:
            tape= False
            state = state(tape)
            print("No markers found.")
    except KeyboardInterrupt:
        pass

# temp (taken from CV)



# shell testing lcd
##while(True):
##    time.sleep(1)
##    print("state: ", state)
##    tape = True
##    distance = input("Distance: ")
##    angle = input("Angle: ")
##    state = state(tape)
##    print("newstate: ", state)
##    time.sleep(1)
##    print("state: ", state)
##    state = state(tape)
##    print("newstate: ", state)
##    time.sleep(3)
##    tape = False
        
