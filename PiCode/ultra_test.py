import time
import RPi.GPIO as GPIO
GPIO.setmode(GPIO.BCM)

TRIG = 5
ECHO = 6

print("Distance measurement in prgoress")
GPIO.setwarnings(False)
GPIO.setup(TRIG, GPIO.OUT)
GPIO.setup(ECHO, GPIO.IN)


try: 
    while True: 
        GPIO.output(TRIG, False)
        print("waiting for sensor to sette")
        time.sleep(2)

        GPIO.output(TRIG, True)
        time.sleep(0.00001)
        GPIO.output(TRIG, False)

        print(GPIO.input(ECHO))

        # while GPIO.input(ECHO)==0:
        #     pulse_start = time.time()

        # while GPIO.input(ECHO)==1:
        #     pulse_end = time.time()

        # try: 
        #     pulse_duration = pulse_end - pulse_start
        # except:
        #     continue

        # distance = pulse_duration * 17150

        # distance = round(distance, 2)

        # print("Distance: ", distance, "cm")

except KeyboardInterrupt:
    print("Cleaning up!")
    gpio.cleanup()