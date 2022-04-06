# EENG350 Demo1 Reference Functions

import smbus
import time
import board
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


def writeNumber(s): # distance and angle
    
    block = str(s) + "a" + str(distance) + "b" + str(angle) + "c"
    data = []
    i = 0
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
    writeNumber(0)
    if not stop:
        if tape:
##            lcd.clear()
            return rotate
        else:
##            lcd.clear()
            return find
    else:
        return none

def straight(tape): # 1 (distance)
##    lcd.text_direction = lcd.LEFT_TO_RIGHT
##    lcd.message = "Moving\nDistance: " + str(distance)
    writeNumber(1)
    if angle < 1:
        return straight
    else:
        lcd.clear()
        return rotate

def rotate(tape): # 2 (angle)
##    lcd.text_direction = lcd.LEFT_TO_RIGHT
##    lcd.message = "Rotating\nAngle: " + str(angle)
    writeNumber(2)
    if float(angle) < 1:
##        lcd.clear()
        return straight
    else:
        return rotate

def find(tape): # 3 (no tape found)
##    lcd.text_direction = lcd.LEFT_TO_RIGHT
##    lcd.message = "Finding Tape..."
    
    temp = int(input("Is tape found? (0/1): ")) ################### TESTING
    if temp:
        tape = True
    else:
        tape = False
    
    writeNumber(3)
    if tape == True:
##        lcd.clear()
        return none
##        return rotate
        
    else:
        return find
    

# temp (taken from CV)
angle = -3.0
state = none
tape = False
stop = False
distance = 1.0

i = 0
while(True):
##    time.sleep(1)
##    print("state: ", state) # none
##    state = state(tape)
##    print("newstate: ", state) # find
##    time.sleep(1)
##    tape = True
##    angle = input("Angle: ")
##    state = state(tape)
##    print("newstate: ", state) # rotate
##    time.sleep(1)
##    angle = 1.4
##    state = state(tape)
##    print("newstate: ", state) # rotate
##    angle = .2
##    state = state(tape)
##    print("newstate: ", state) # straight
##    time.sleep(3)
##    angle = float(input("Angle: "))
##    state = state(tape)
##    print("newstate: ", state) # none

    print("Current State: ", state)
    distance = input("Distance: ")
    angle = input("Angle: ")

    state = state(tape)
