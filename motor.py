import RPi.GPIO as GPIO
import time
# servo = GPIO.PWM(11,50) # Define servo as PWM, pulse 50Hz
def init_seed_drill_motor():
    # GPIO.setmode(GPIO.BOARD)
    # GPIO.setmode(GPIO.BCM)
    # GPIO.setup(29,GPIO.OUT) # Set pin 29 as output)
    global SEED_DRILL
    SEED_DRILL = 5
    # GPIO.setup(5,GPIO.OUT) # Set pin 29 as output)
    GPIO.setup(SEED_DRILL, GPIO.OUT) # Set pin 29 as output)
    
    global servo
    servo = GPIO.PWM(11,50) # Define servo as PWM, pulse 50Hz
    servo.start(0) # Start PWM with pulse off

def rotate(angle):
    servo.ChangeDutyCycle(2+(angle/18))
    time.sleep(0.5) # gives time for motor to turn
    servo.ChangeDutyCycle(0)

def seed1():
    rotate(80) # plant seed
    time.sleep(0.75) # give time to drop seeds
    rotate(93) # return to neutral
    time.sleep(0.5)

def seed2():
    rotate(93) # plant seed
    time.sleep(0.75) # give time to drop seeds
    rotate(80) # return to neutral
    time.sleep(0.5)

# seed1()
# seed2()

def stop_seed_drill_motor():
    servo.stop()
    GPIO.cleanup()
    
    print("Finished planting seeds.")