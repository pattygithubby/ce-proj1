# coding: utf-8
import time
import RPi.GPIO as GPIO

# Use BCM GPIO references
# instead of physical pin numbers
GPIO.setmode(GPIO.BCM)

# Define GPIO to use on Pi
GPIO_TRIGECHO = 17
GPIO_LED = 26

print("Ultrasonic Measurement")

# Set pins as output and input
GPIO.setup(GPIO_TRIGECHO,GPIO.OUT)  # Initial state as output
GPIO.setup(GPIO_LED, GPIO.OUT) # 



# Set trigger to False (Low)
GPIO.output(GPIO_TRIGECHO, False)

def measure():
  # This function measures a distance
  # Pulse the trigger/echo line to initiate a measurement
    GPIO.output(GPIO_TRIGECHO, True)
    time.sleep(0.00001)
    GPIO.output(GPIO_TRIGECHO, False)
  #ensure start time is set in case of very quick return
    start = time.time()

  # set line to input to check for start of echo response
    GPIO.setup(GPIO_TRIGECHO, GPIO.IN)
    while GPIO.input(GPIO_TRIGECHO)==0:
        start = time.time()

  # Wait for end of echo response
    while GPIO.input(GPIO_TRIGECHO)==1:
        stop = time.time()
  
    GPIO.setup(GPIO_TRIGECHO, GPIO.OUT)
    GPIO.output(GPIO_TRIGECHO, False)

    elapsed = stop-start
    distance = (elapsed * 34300)/2.0
    time.sleep(0.1)
    return distance

def control_led(distance):
    """
    Styrer LED’en ud fra den målte afstand.
    - 25 cm < distance < 30 cm: LED tændes i 1 sekund, slukkes i 2 sekunder.
    - 18 cm < distance < 25 cm: LED tændes i 1 sekund, slukkes i 1 sekund.
    - distance < 18 cm: LED tændes (her kan du vælge, om den skal forblive tændt eller blinke).
    """
    if 25 < distance < 30:
        GPIO.output(GPIO_LED, True)
        time.sleep(1)
        GPIO.output(GPIO_LED, False)
        time.sleep(1)
    elif 18 < distance < 25:
        GPIO.output(GPIO_LED, True)
        time.sleep(0.5)
        GPIO.output(GPIO_LED, False)
        time.sleep(0.5)
    elif distance < 18:
        GPIO.output(GPIO_LED, True)
    else:
        # Hvis afstanden er større end 30 cm, sørg for, at LED’en er slukket
        GPIO.output(GPIO_LED, False)

try:
    while True:
        # Mål afstanden
        distance = measure()
        print("Distance: %.1f cm", distance)
        
        # Styr LED’en baseret på den målte afstand
        control_led(distance)
        
        # Ekstra kørselslogik ud fra afstanden (f.eks. styring af en robot)
        if distance < 15:
            print("Stop! Drive backwards")
        elif distance < 30:
            print("Turn right!")
        else:
            print("Drive forward!")
            
        time.sleep(1)

except KeyboardInterrupt:
    print("Stop")
    GPIO.cleanup()
        