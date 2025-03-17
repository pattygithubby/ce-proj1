import smbus
import time
# Get I2C bus
# Open I2C-databus nr-1 on the Rasperry PI

def setup_sensor():
    """Initialiserer I2C-sensoren."""
    bus = smbus.SMBus(1)  # Åbner I2C-bus nr. 1
    # ISL29125 address, 0x44(68)
    # Select configuation-1register, 0x01(01)
    # 0x0D(13) Operation: RGB, Range: 360 lux, Res: 16 Bits
    bus.write_byte_data(0x44, 0x01, 0x05)  # Konfigurer ISL29125
    time.sleep(1)  # Vent på at sensoren starter
    return bus  # Returner bus-objektet
