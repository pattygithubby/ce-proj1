import time  # Import the time module for handling delays
import RPi.GPIO as GPIO  # Import the GPIO library to interface with Raspberry Pi's GPIO pins

# Use BCM GPIO references instead of physical pin numbers
GPIO.setmode(GPIO.BCM)

# Define GPIO pin to use on Raspberry Pi for both trigger and echo functionality
GPIO_TRIGECHO = 17

print("Ultrasonic Measurement")  # Print a message to indicate the script has started

# Set GPIO pin as an output initially
GPIO.setup(GPIO_TRIGECHO, GPIO.OUT)

# Ensure the trigger is set to False (Low) before starting measurements
GPIO.output(GPIO_TRIGECHO, False)

def measure():
    # Measures the distance by sending an ultrasonic pulse and measuring the 
    # time it takes for the echo to return.
    
    # Pulse the trigger/echo line to initiate a measurement
    GPIO.output(GPIO_TRIGECHO, True)  # Set the pin HIGH
    time.sleep(0.00001)  # Wait for 10 microseconds
    GPIO.output(GPIO_TRIGECHO, False)  # Set the pin LOW
    
    # Ensure start time is set in case of very quick return
    start = time.time()
    
    # Change the pin to input mode to detect the returning echo signal
    GPIO.setup(GPIO_TRIGECHO, GPIO.IN)
    while GPIO.input(GPIO_TRIGECHO) == 0:  # Wait for the signal to start
        start = time.time()
    
    # Capture the time when the echo signal is received
    stop = start  # Default value to avoid potential errors
    while GPIO.input(GPIO_TRIGECHO) == 1:  # Wait for the signal to end
        stop = time.time()
    
    # Reset the pin back to output mode for the next measurement
    GPIO.setup(GPIO_TRIGECHO, GPIO.OUT)
    GPIO.output(GPIO_TRIGECHO, False)
    
    # Calculate the elapsed time for the echo to return
    elapsed = stop - start
    
    # Convert time to distance using the speed of sound (34300 cm/s)
    distance = (elapsed * 34300) / 2.0  # Divide by 2 to account for round trip
    
    time.sleep(0.1)  # Short delay to avoid excessive measurements
    return distance  # Return the measured distance

try:
    while True:
        distance = measure()  # Call the measure function
        print(f"  Distance : {distance:.1f} cm")  # Print the measured distance in centimeters
        time.sleep(1)  # Wait for 1 second before measuring again

except KeyboardInterrupt:
    print("Stop")  # Print message when user interrupts the script with Ctrl+C
    GPIO.cleanup()  # Reset GPIO pins to prevent issues when rerunning the script
