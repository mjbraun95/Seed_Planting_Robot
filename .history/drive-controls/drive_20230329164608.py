import RPi.GPIO as GPIO
import time

# Define the GPIO pins connected to the motor driver board
M1PWM = 17
M1DIR = 27
M2PWM = 22
M2DIR = 23

# Set up the Raspberry Pi's GPIO mode
GPIO.setmode(GPIO.BCM)

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

# TODO: Remove after testing new code
# def drive_motor(motor_pwm, motor_dir, direction, speed, duration):
#     GPIO.output(motor_dir, direction)
#     motor_pwm.ChangeDutyCycle(speed)
#     time.sleep(duration)
#     motor_pwm.ChangeDutyCycle(0)

# def drive_forward(speed, duration):
#     drive_motor(motor1_pwm, M1DIR, GPIO.HIGH, speed, duration)
#     drive_motor(motor2_pwm, M2DIR, GPIO.HIGH, speed, duration)

# def drive_backward(speed, duration):
#     drive_motor(motor1_pwm, M1DIR, GPIO.LOW, speed, duration)
#     drive_motor(motor2_pwm, M2DIR, GPIO.LOW, speed, duration)

# def turn_left(speed, duration):
#     drive_motor(motor1_pwm, M1DIR, GPIO.LOW, speed, duration)
#     drive_motor(motor2_pwm, M2DIR, GPIO.HIGH, speed, duration)

# def turn_right(speed, duration):
#     drive_motor(motor1_pwm, M1DIR, GPIO.HIGH, speed, duration)
#     drive_motor(motor2_pwm, M2DIR, GPIO.LOW, speed, duration)
def drive_motor(motor1_pwm, motor1_dir, motor2_pwm, motor2_dir, direction1, direction2, speed, duration):
    GPIO.output(motor1_dir, direction1)
    GPIO.output(motor2_dir, direction2)
    motor1_pwm.ChangeDutyCycle(speed)
    motor2_pwm.ChangeDutyCycle(speed)
    time.sleep(duration)
    motor1_pwm.ChangeDutyCycle(0)
    motor2_pwm.ChangeDutyCycle(0)

def drive_forward(speed, duration):
    drive_motor(motor1_pwm, M1DIR, motor2_pwm, M2DIR, GPIO.HIGH, GPIO.HIGH, speed, duration)

def drive_backward(speed, duration):
    drive_motor(motor1_pwm, M1DIR, motor2_pwm, M2DIR, GPIO.LOW, GPIO.LOW, speed, duration)

def turn_left(speed, duration):
    drive_motor(motor1_pwm, M1DIR, motor2_pwm, M2DIR, GPIO.LOW, GPIO.HIGH, speed, duration)

def turn_right(speed, duration):
    drive_motor(motor1_pwm, M1DIR, motor2_pwm, M2DIR, GPIO.HIGH, GPIO.LOW, speed, duration)

try:
    # Drive forward at 50% speed for 2 seconds
    drive_forward(50, 2)

    # Drive backward at 25% speed for 1 second
    drive_backward(25, 1)

    # Turn left at 50% speed for 1 second
    turn_left(50, 1)

    # Turn right at 50% speed for 1 second
    turn_right(50, 1)

finally:
    # Clean up the GPIO pins and stop the PWM signals
    motor1_pwm.stop()
    motor2_pwm.stop()
    GPIO.cleanup()
