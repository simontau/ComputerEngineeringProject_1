import time
import RPi.GPIO as GPIO

GPIO.setmode(GPIO.BCM)

# The GPIO that we use on the PI
PW_PIN = 18 
# The pins as input and output:
GPIO.setup(PW_PIN, GPIO.IN)

# Function to measure the distance
def measure():
    # Wait for the pulse to start
    while GPIO.input(PW_PIN) == 0:
        pass
    start = time.time()

    # Wait for the pulse to end
    while GPIO.input(PW_PIN) == 1:
        pass
    end = time.time()

    pulse_width = end - start  # in seconds
    # According to many LV-MaxSonar datasheets:
    # approximately 58 microseconds of pulse width per cm
    distance_cm = pulse_width / (58e-6)
    return distance_cm

# Calling the function and printing the distance until keyboard interrupt
try:
    while True:
        # Printing the distance every .1 seconds:
        dist = measure()
        print("Distance: {:.1f} cm".format(dist))
        time.sleep(0.1)

# Cleaning up when interrupted
except KeyboardInterrupt:
    GPIO.cleanup()
