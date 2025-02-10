import smbus
import time

# Get I2C bus
bus = smbus.SMBus(1)  # or smbus.SMBus(0)

# ISL29125 address, 0x44(68)
# Select configuration-1 register, 0x01(01)
# 0x05: Operation: RGB, Range: 360 lux, Res: 16 Bits (as per datasheet)
bus.write_byte_data(0x44, 0x01, 0x05)

# Sleep for 1 second
time.sleep(1)

print("Reading colour values and displaying them in a new window\n")

def getAndUpdateColour():
    while True:
        # Read 6 bytes starting from register 0x09
        data = bus.read_i2c_block_data(0x44, 0x09, 6)
        
        # Correctly convert the data to red, green, and blue int values
        green   = data[1] + data[0] / 256 # Index 0 (green low) and Index 1 (green high)
        red = data[3] + data[2] / 256 # Index 2 (red low) and Index 3 (red high)
        blue  = (data[5] + data[4] / 256) * 2 # Index 4 (blue low) and Index 5 (blue high). Multiplied by 2 due to measurement failure.

        # Calibrate the measured colors and printing:

        # Checking the different conditions and printing the color:
        if red > green and red > blue:
            print("Red")
        elif green > red and green > blue:
            print("Green")
        else:
            print("Blue")

        # Output data to the console as RGB values
        #print("RGB(%d, %d, %d)" % (red, green, blue))
        
        # Sleep for two seconds before it reads again
        time.sleep(2)

# Call the function to start reading and updating colour values
getAndUpdateColour()
