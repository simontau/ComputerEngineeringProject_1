import smbus2 as smbus
import time

# Get I2C bus
bus = smbus.SMBus(1)

bus.write_byte_data(0x44, 0x01, 0x05)

time.sleep(1)

print("Reading colour values and displaying them in a new window\n")

def getAndUpdateColour():
    while True:
        data = bus.read_i2c_block_data(0x44, 0x09, 6) #Selects the right registers
        green = data[1] + data[0]/256 # Calculates the levels of each color [0, 255]
        red = data[3] + data[2]/256
        blue = data[5] + data[4]/256
        
        
        # Determines the color based on which has the higher value
        color = ""
        if green > red and green > blue: 
            color = "Green"
        elif blue > red:
            color = "Blue"
        else:
            color = "Red"

        print("RGB(%d %d %d)" % (red, green, blue))

        print("The color is: " + color)
        
        
        time.sleep(2) 
        
getAndUpdateColour()