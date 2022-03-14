# EENG350 Demo1 Reference Functions Task 1

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
    lcd.message = "Tape Found!\nAngle: " + str(angle)
    
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
        
