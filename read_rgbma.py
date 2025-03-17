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


        
        total_sum = red + green + blue
        if(total_sum == 0):
            norm_red = norm_green = norm_blue = 0
        else:
            norm_red = (red / total_sum)
            norm_green = (green / total_sum)
            norm_blue = (blue / total_sum) 
        

        dominating_color = None
        
        # Detect RED: High red, lower green & blue
        if 0.45 <= norm_red <= 0.55 and 0.36 <= norm_green <= 0.42: 
            dominating_color = "red"
        elif 0.24 <= norm_red <= 0.34 and 0.47 <= norm_green <= 0.58 :
            dominating_color = "green"

        # Print a message only if the color changes
        if dominating_color != last_color:
            if dominating_color == "red":
                print("Dominating color is RED: Victim has been located")
            elif dominating_color == "green":
                print("Dominating color is GREEN: Somebody is in need of help!")
            
            # Update last_color
            last_color = dominating_color
        
        time.sleep(2)

getAndUpdateColour()

