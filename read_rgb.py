import smbus
import time

# Get I2C bus
bus = smbus.SMBus(1)  # or smbus.SMBus(0) depending on your Pi revision

# ISL29125 address, 0x44(68)
# Select configuation-1 register, 0x01(01)
# 0x05 = RGB mode, 360 lux, 16-bit resolution
bus.write_byte_data(0x44, 0x01, 0x05)

time.sleep(1)

print("Reading colour values and displaying them in a new window\n")

def getAndUpdateColour():
    while True:
        # Read the data from the sensor (6 bytes: Green L/H, Red L/H, Blue L/H)
        data = bus.read_i2c_block_data(0x44, 0x09, 6)
        
        # Convert the data to green, red, and blue *integer* values [0-255]
        # The ISL29125 is 16-bit, so we scale down by 256 to get approximate 8-bit values.
        green = int(data[1] + data[0] / 256.0)
        red   = int(data[3] + data[2] / 256.0)
        blue  = int(data[5] + data[4] / 256.0)
        
        # Output data to the console in (R, G, B) format
        print("RGB(%d %d %d)" % (red, green, blue))
        
        time.sleep(2)  # Adjust delay as needed

getAndUpdateColour()