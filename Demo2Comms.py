# EENG350 Demo1 Reference Functions

import smbus
import time
import board
import numpy as np
##import adafruit_character_lcd.character_lcd_rgb_i2c as character_lcd

# I2C
bus = smbus.SMBus(1)
address = 0x04
i2c = board.I2C()  # uses board.SCL and board.SDA

### RPI - LCD
### Modify this if you have a different sized Character LCD
##lcd_columns = 16
##lcd_rows = 2
### Initialise the LCD class
##lcd = character_lcd.Character_LCD_RGB_I2C(i2c, lcd_columns, lcd_rows)
### Set LCD color to whiteish
##lcd.color = [200, 25, 25]


def writeNumber(s, tape): # distance and angle
    if tape:
        t = 1
    else:
        t = 0
        
    angleRad = (angle/180)*np.pi
    distanceFt = distance/12
    integer = int(angleRad * (10**3))/(10**3)
    angleRad = float(integer)
    integer = int(distanceFt * (10**3))/(10**3)
    distanceFt = float(integer)
    
    block = str(s) + "a" + str(distanceFt) + "b" + str(angleRad) + "c" + str(t)
    data = []
    i = 0
    print(block)
    for ch in block:
        data.append(ord(ch))
        i = i + 1
    bus.write_i2c_block_data(address, 0, data)
    return -1

def readNumber():
    #number = bus.read_byte(address)
    #number = bus.read_byte_data(address, 0)
    data = bus.read_i2c_block_data(address, 0, 4) # reading back float from Arduino
    return data


def none(tape): # 0 (do nothing)
##    lcd.text_direction = lcd.LEFT_TO_RIGHT
##    lcd.message = "stop"
    if not stop:
        if tape:
##            lcd.clear()
            writeNumber(2, tape)
            return rotate
        else:
##            lcd.clear()
            writeNumber(3, tape)
            return find
    else:
        return none

def straight(tape): # 1 (distance)
##    lcd.text_direction = lcd.LEFT_TO_RIGHT
##    lcd.message = "Moving\nDistance: " + str(distance)
    if angle < 1:
        writeNumber(1, tape)
        return straight
    else:
##        lcd.clear()
        return straight

def rotate(tape): # 2 (angle)
##    lcd.text_direction = lcd.LEFT_TO_RIGHT
##    lcd.message = "Rotating\nAngle: " + str(angle)
    if float(angle) < 1:
##        lcd.clear()
        writeNumber(1, tape)
        return straight
    else:
        #writeNumber(2, tape)
        return rotate

def find(tape): # 3 (no tape found)
##    lcd.text_direction = lcd.LEFT_TO_RIGHT
##    lcd.message = "Finding Tape..."

    #print("len", da_input.len)
    #temp = int(da_input.strip()) ################### TESTING
##    if temp == 1:
##        tape = True
##    else:
##        tape = False
        
    if tape:
##        lcd.clear()
        writeNumber(0, tape)
        return none
    else:
        return find

##states = {'DO_NOTHING': 0, 'TURN': 2, 'GO_FORWARD': 1, 'FIND_TAPE': 3}
##
##class messenger(object):
##    def __init__(self, distance, angle):
##        self.distance = distance
##        self.angle = angle
##
##        self.tape_found = False
##        self.state = states['DO_NOTHING']
##
##    def stop(self):
##    def turn(self):
##    def drive(self):
##    def find_tape(self):
##
##    def write_robot

        

# temp (taken from CV)
angle = 69.0
state = none
tape = False
stop = False
distance = 42.0

i = 0
if __name__ == '__main__':
    print("Starting Robot")
    time.sleep(1)
    #print("state: ", state) # none
    state = state(tape)
    print("Finding tape")
    #print("newstate: ", state) # find
    time.sleep(1)
    state = state(tape)
    #print("newstate: ", state) # find
    print("Finding tape")
    time.sleep(1)
    tape = True
    print("Stopping Robot")
    state = state(tape) # return none
    #print("newstate: ", state) # none
    
    #time.sleep(1)
    #state = state(tape)
    #print("newstate: ", state)

    
    #time.sleep(1)
    #print("Rotating Robot")
    #angle = 1.4
    #state = state(tape)

    time.sleep(1)
    print("Driving Robot")
    angle = 2.0
    distance = 4.0
    writeNumber(1, tape)
    writeNumber(2, tape)
    time.sleep(1)
    writeNumber(0, tape)
    #print("newstate: ", state) # rotate
##    angle = .2
##    state = state(tape)
##    print("newstate: ", state) # straight
##    time.sleep(3)
##    angle = float(input("Angle: "))
##    state = state(tape)
##    print("newstate: ", state) # none

    #print("New State: ", state)
##    distance = input("Distance: ")
##    angle = input("Angle: ")
##
##    state = state(tape)
##    print("Current State: ", state)
##    
