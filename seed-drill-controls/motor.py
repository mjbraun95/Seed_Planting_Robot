import RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BOARD)
GPIO.setup(3,GPIO.OUT)

servo = GPIO.PWM(3,50)
servo.start(0)

def plantSeed(angle):
    # rotate to drop seeds
    duty = angle / 18 + 2
    GPIO.output(3, True) # sets pin 3 for output
    servo.ChangeDutyCycle(duty)
    time.sleep(2) # gives time for motor to turn and seeds to drop

    # return to neutral
    duty = 90 / 18 + 2
    servo.ChangeDutyCycle(duty)
    time.sleep(1)
    GPIO.output(3, False) # turns off pin 3
    servo.ChangeDutyCycle(0)

def seed1():
    plantSeed(45)

def seed2():
    plantSeed(135)

seed1()
seed2()

servo.stop()
GPIO.cleanup()
print("Finished planting seeds.")