import RPi.GPIO as GPIO

# TODO: Remove after testing new code
def drive_motor(motor_pwm, motor_dir, direction, speed, duration):
    GPIO.output(motor_dir, direction)
    motor_pwm.ChangeDutyCycle(speed)
    time.sleep(duration)
    motor_pwm.ChangeDutyCycle(0)

def drive_forward(speed, duration):
    drive_motor(motor1_pwm, M1DIR, GPIO.HIGH, speed, duration)
    drive_motor(motor2_pwm, M2DIR, GPIO.HIGH, speed, duration)

def drive_backward(speed, duration):
    drive_motor(motor1_pwm, M1DIR, GPIO.LOW, speed, duration)
    drive_motor(motor2_pwm, M2DIR, GPIO.LOW, speed, duration)

def turn_left(speed, duration):
    drive_motor(motor1_pwm, M1DIR, GPIO.LOW, speed, duration)
    drive_motor(motor2_pwm, M2DIR, GPIO.HIGH, speed, duration)

def turn_right(speed, duration):
    drive_motor(motor1_pwm, M1DIR, GPIO.HIGH, speed, duration)
    drive_motor(motor2_pwm, M2DIR, GPIO.LOW, speed, duration)