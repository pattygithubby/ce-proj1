import serial
import time

# Open serial connection to LV-MaxSonar sensor
# The sensor is connected to the serial port /dev/serial10: The primary UART on the Raspberry Pi 3
ser = serial.Serial('/dev/serial10', 9600, timeout=1) # Timeout after 1 second

def read_distance():
    """Reads the distance from the LV-MaxSonar sensor constantly."""
    

