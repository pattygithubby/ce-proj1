import smbus
import time

# Get I2C bus
# Open I2C-databus nr-1 on the Rasperry PI
bus = smbus.SMBus(1) # or smbus.SMBus(0)

# ISL29125 address, 0x44(68)
# Select configuation-1register, 0x01(01)
# 0x0D(13) Operation: RGB, Range: 360 lux, Res: 16 Bits
bus.write_byte_data(0x44, 0x01, 0x05)

time.sleep(1)

print("Reading colour values and displaying them in a new window\n")

# Data registers for the colors:
# Startregister
green_low_byte = 0x09


def getAndUpdateColour():
    last_color = None
    
    while True:
	# Read the data from the sensor
        # Insert code here
        data = bus.read_i2c_block_data(0x44, green_low_byte, 6)
        

            # Convert the data to green, red and blue int values
            # Insert code here
        green16bit = (data[1] << 8) | data[0]
        red16bit = 	(data[3] << 8) | data[2]
        blue16bit = 	(data[5] << 8) | data[4]


        red = (red16bit / 65535) * 255 
        green = (green16bit / 65535) * 255 
        blue = (blue16bit / 65535) * 255 

        sum = red + green + blue
        norm_red = (red / sum)
        norm_green = (green / sum)
        norm_blue = (blue / sum) 


                # Output data to the console RGB values
        # Uncomment the line below when you have read the red, green and blue
        print("RGB(%.2f %.2f %.2f)" % (norm_red, norm_green, norm_blue))

        print()
        
        time.sleep(2)

getAndUpdateColour()
            
