import RPi.GPIO as GPIO
import time

# Set up the Raspberry Pi's GPIO mode
GPIO.setmode(GPIO.BCM)  # Use Broadcom (BCM) pin numbering

# Define the GPIO pins connected to the motor driver board
M1PWM = 17
M1DIR = 27
M2PWM = 22
M2DIR = 23

# Set up the GPIO pins as output pins
GPIO.setup(M1PWM, GPIO.OUT)
GPIO.setup(M1DIR, GPIO.OUT)
GPIO.setup(M2PWM, GPIO.OUT)
GPIO.setup(M2DIR, GPIO.OUT)

# Set up the PWM signals for motor speed control (with a 100Hz frequency)
motor1_pwm = GPIO.PWM(M1PWM, 100)
motor2_pwm = GPIO.PWM(M2PWM, 100)

# Start the PWM signals with a 0% duty cycle (stopped motors)
motor1_pwm.start(0)
motor2_pwm.start(0)

try:
    # Drive Motor 1 forward at 50% speed for 2 seconds
    GPIO.output(M1DIR, GPIO.HIGH)  # Set the direction pin to HIGH for forward movement
    motor1_pwm.ChangeDutyCycle(50)  # Set the PWM duty cycle to 50% for half speed
    time.sleep(2)  # Wait for 2 seconds

    # Drive Motor 2 forward at 75% speed for 3 seconds
    GPIO.output(M2DIR, GPIO.HIGH)  # Set the direction pin to HIGH for forward movement
    motor2_pwm.ChangeDutyCycle(75)  # Set the PWM duty cycle to 75% for 3/4 speed
    time.sleep(3)  # Wait for 3 seconds

    # Stop both motors
    motor1_pwm.ChangeDutyCycle(0)
    motor2_pwm.ChangeDutyCycle(0)
    time.sleep(1)  # Wait for 1 second

    # Drive both motors backward at 25% speed for 1 second
    GPIO.output(M1DIR, GPIO.LOW)  # Set the direction pin to LOW for backward movement
    GPIO.output(M2DIR, GPIO.LOW)  # Set the direction pin to LOW for backward movement
    motor1_pwm.ChangeDutyCycle(25)  # Set the PWM duty cycle to 25% for 1/4 speed
    motor2_pwm.ChangeDutyCycle(25)  # Set the PWM duty cycle to 25% for 1/4 speed
    time.sleep(1)  # Wait for 1 second

finally:
    # Clean up the GPIO pins and stop the PWM signals
    motor1_pwm.stop()
    motor2_pwm.stop()
    GPIO.cleanup()