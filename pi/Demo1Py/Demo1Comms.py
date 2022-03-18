# EENG350 Demo1 Reference Functions

import smbus
import time
import board
import adafruit_character_lcd.character_lcd_rgb_i2c as character_lcd

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


def writeNumber(distance, angle): # distance and angle
    
    block = str(distance) + "a" + str(angle) + "b"
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

def none(tape): # no tape found
    lcd.text_direction = lcd.LEFT_TO_RIGHT
    lcd.message = "No Tape Found"
    if tape:
        lcd.clear()
        return found
    else:
        return none

def found(tape): # tape found
    writeNumber(distance, angle)
    lcd.text_direction = lcd.LEFT_TO_RIGHT
    lcd.message = "Distance: " + str(distance) + "\nAngle: " + str(angle)
    
    if not tape:
        lcd.clear()
        return none
    else:
        return found
    

# temp (taken from CV)
angle = 0.0
state = none
tape = False
distance = 0.0

while(True):
    tape = True
    distance = input("Distance: ")
    angle = input("Angle: ")
    state = state(tape)
    time.sleep(3)
    tape = False
    state = state(tape)
    time.sleep(1)
    
        
