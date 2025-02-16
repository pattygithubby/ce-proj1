import smbus
import time

# Get I2C bus
bus = smbus.SMBus(1)  # or smbus.SMBus(0) on older Pis

# ISL29125 address, 0x44 (68)
# Configuration register 1, 0x01
# 0x05 = RGB mode, 360 lux, 16-bit resolution
bus.write_byte_data(0x44, 0x01, 0x05)

time.sleep(1)

print("Reading colour values and detecting RED / GREEN\n")

def getAndUpdateColour():
    # Keep track of the last dominating color reported: 'red', 'green', or None
    last_color = None
    
    while True:
        # 1) Read 6 bytes from the sensor: (Green L/H, Red L/H, Blue L/H)
        data = bus.read_i2c_block_data(0x44, 0x09, 6)
        
        # 2) Convert raw data into approximate 8-bit values
        green = int(data[1] + data[0] / 256.0)
        red   = int(data[3] + data[2] / 256.0)
        blue  = int(data[5] + data[4] / 256.0)
        
        # 3) Determine if RED or GREEN is dominating
        # Adjust threshold and logic as you see fit
        threshold = 80  # example threshold, you can tune this
        
        dominating_color = None
        
        # Check if red is clearly higher than the other channels
        if red > threshold and red > green and red > blue:
            dominating_color = "red"
        
        # Check if green is clearly higher than the other channels
        elif green > threshold and green > red and green > blue:
            dominating_color = "green"
        
        # 4) Print a message only if we have a new dominating color (red/green) 
        #    and it's different from the last color we announced
        if dominating_color is not None and dominating_color != last_color:
            if dominating_color == "red":
                print("Dominating color is RED: Victim has been located")
            elif dominating_color == "green":
                print("Dominating color is GREEN: Somebody is in need of help!")
            
            # Update the last_color we printed
            last_color = dominating_color
        
        time.sleep(2)  # Adjust the sleep time as necessary

getAndUpdateColour()