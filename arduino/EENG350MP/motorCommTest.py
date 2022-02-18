# EENG350 Mini Project comms test

import smbus
import time
import board
import adafruit_character_lcd.character_lcd_rgb_i2c as character_lcd

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

# Set LCD color to blue
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

lcd.clear()
print("Starting test")
# CV assign desired setpoint (0, 1, 2, 3 as numerator of angle)
print("Setting setpoint: 3")
writeNumber(3)
# Print message left to right
lcd.text_direction = lcd.LEFT_TO_RIGHT
lcd.message = "Setpoint: " + str(3)
# sleep one second
time.sleep(8)

print("Setting setpoint: 0")
writeNumber(0)
# Print message left to right
lcd.text_direction = lcd.LEFT_TO_RIGHT
lcd.message = "Setpoint: " + str(0)
# sleep one second
time.sleep(8)

print("Setting setpoint: 2")
writeNumber(2)
# Print message left to right
lcd.text_direction = lcd.LEFT_TO_RIGHT
lcd.message = "Setpoint: " + str(2)
# sleep one second
time.sleep(8)

print("Setting setpoint: 1")
writeNumber(1)
# Print message left to right
lcd.text_direction = lcd.LEFT_TO_RIGHT
lcd.message = "Setpoint: " + str(1)

print("Done")

    

     

