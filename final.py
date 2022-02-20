# EENG350 Mini Project

import smbus
import time
import board
import adafruit_character_lcd.character_lcd_rgb_i2c as character_lcd
import cv2 as cv
from picamera import PiCamera
from time import sleep
import numpy as np
import sys

#Camera setup
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


# RPI - Arduino
# for RPI version 1, use “bus = smbus.SMBus(0)”
bus = smbus.SMBus(1)

# This is the address we setup in the Arduino Program
address = 0x04


# RPI - LCD
# Modify this if you have a different sized Character LCD
lcd_columns = 16
lcd_rows = 2

# Initialise I2C bus.
i2c = board.I2C()  # uses board.SCL and board.SDA

# Initialise the LCD class
lcd = character_lcd.Character_LCD_RGB_I2C(i2c, lcd_columns, lcd_rows)

# Set LCD color to blue (green ig?)
lcd.color = [0, 100, 0]


def writeNumber(value): 
    bus.write_byte(address, value)
    #bus.write_byte_data(address, 0, value)
    #bus.write_i2c_block_data(address, 0, data)
    return -1

def readNumber():
    number = bus.read_byte(address)
    #number = bus.read_byte_data(address, 0)
    #data = bus.read_i2c_block_data(address, 0)
    return number

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
                #print("Zone %d" % Quad)
            elif(cxc>(width/2))and(cyc<(height/2)):
                Quad = 1
                #print("Zone %d" % Quad)
            elif(cxc<(width/2))and(cyc>(height/2)):
                Quad = 2
                #print("Zone %d" % Quad)
            else:
                Quad = 3
                #print("Zone %d" % Quad)
            
            # CV assign desired setpoint (0, 1, 2, 3 as numerator of angle)
            setpoint = Quad
            writeNumber(setpoint)
#            print("RPI: Hi Arduino, I sent you ", setpoint)
    
            # Print message left to right
            lcd.text_direction = lcd.LEFT_TO_RIGHT
            lcd.message = "Setpoint: " + str(setpoint)
            # sleep one second
            time.sleep(1)

            number = readNumber()
#            print("Arduino: Hey RPI, I received a setpoint ", number)
#            print()
                
        except IndexError:
            #print("No markers found.")
            
            
except KeyboardInterrupt:
    pass

    

     


