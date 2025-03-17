from turtlebot_lib.setup_sensor import setup_sensor
from turtlebot_lib.procces_color import normalize_colors

# Initialiser sensoren
bus = setup_sensor()
green_low_byte = 0x09  # Startregister for farver

def read_color_data():
    """Læser RGB-data fra ISL29125-sensoren."""
    data = bus.read_i2c_block_data(0x44, green_low_byte, 6)
    
    green16bit = (data[1] << 8) | data[0]
    red16bit = (data[3] << 8) | data[2]
    blue16bit = (data[5] << 8) | data[4]

    # Normaliser farverne her
    return normalize_colors(red16bit, green16bit, blue16bit)
