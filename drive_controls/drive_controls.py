import RPi.GPIO as GPIO
import time
import math

# Define the GPIO pins connected to the motor driver board
# TODO?: Convert to BOARD format
def init_drive_controls():
    M1PWM = 17
    M1DIR = 27
    M2PWM = 22
    M2DIR = 23

    # Define the encoder pins for both motors
    ENCODER1_PIN = 24 # TODO: Change
    ENCODER2_PIN = 25 # TODO: Change

    # Set up the GPIO pins
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(ENCODER1_PIN, GPIO.IN)
    GPIO.setup(ENCODER2_PIN, GPIO.IN)

    # Initialize the encoder counters
    encoder1_count = 0
    encoder2_count = 0

    # Define wheel parameters
    # TODO: Confirm in lab
    WHEELBASE = 393 #mm
    WHEEL_DIAMETER = 152 #mm

    # Set up the Raspberry Pi's GPIO mode
    # TODO?: Convert to BOARD format
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
    
    

    # Attach the callback functions to the encoder pins
    GPIO.add_event_detect(ENCODER1_PIN, GPIO.RISING, callback=encoder1_callback)
    GPIO.add_event_detect(ENCODER2_PIN, GPIO.RISING, callback=encoder2_callback)

def reset_encoder_counts():
    global encoder1_count, encoder2_count
    encoder1_count = 0
    encoder2_count = 0

def encoder1_callback(channel):
    global encoder1_count
    encoder1_count += 1
    print("encoder1_count: {}".format(encoder1_count))

def encoder2_callback(channel):
    global encoder2_count
    encoder2_count += 1
    print("encoder2_count: {}".format(encoder2_count))



# TODO: Test
# TODO: Calibrate turning_speed_dps
# def angle_to_duration(angle_degrees, speed, turning_speed_dps):
#     speed_ratio = speed / 100
#     adjusted_turning_speed = turning_speed_dps * speed_ratio
#     duration = angle_degrees / adjusted_turning_speed
#     return duration

# def turn_left_degrees(speed, angle_degrees, turning_speed_dps):
#     duration = angle_to_duration(angle_degrees, turning_speed_dps)
#     turn_left(speed, duration)

# def turn_right_degrees(speed, angle_degrees, turning_speed_dps):
#     duration = angle_to_duration(angle_degrees, turning_speed_dps)
#     turn_right(speed, duration)


# TODO: Find out pulses per rotation
def turn_left_degrees(speed, angle_degrees, pulses_per_rotation, wheelbase):
    reset_encoder_counts()
    target_pulses = angle_degrees * (wheelbase * pulses_per_rotation) / (2 * math.pi * 360) #TODO: Confirm math
    
    while encoder1_count < target_pulses or encoder2_count < target_pulses:
        if encoder1_count < target_pulses:
            motor1_pwm.ChangeDutyCycle(speed)
        else:
            motor1_pwm.ChangeDutyCycle(0)

        if encoder2_count < target_pulses:
            motor2_pwm.ChangeDutyCycle(speed)
        else:
            motor2_pwm.ChangeDutyCycle(0)
        
    motor1_pwm.ChangeDutyCycle(0)
    motor2_pwm.ChangeDutyCycle(0)

def turn_right_degrees(speed, angle_degrees, pulses_per_rotation, wheelbase):
    reset_encoder_counts()
    target_pulses = angle_degrees * (wheelbase * pulses_per_rotation) / (2 * math.pi * 360) #TODO: Confirm math
    
    while encoder1_count < target_pulses or encoder2_count < target_pulses:
        if encoder1_count < target_pulses:
            motor1_pwm.ChangeDutyCycle(speed)
        else:
            motor1_pwm.ChangeDutyCycle(0)

        if encoder2_count < target_pulses:
            motor2_pwm.ChangeDutyCycle(speed)
        else:
            motor2_pwm.ChangeDutyCycle(0)
        
    motor1_pwm.ChangeDutyCycle(0)
    motor2_pwm.ChangeDutyCycle(0)

# TODO: Test
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

def stop(duration):
    drive_motor(motor1_pwm, M1DIR, motor2_pwm, M2DIR, GPIO.LOW, GPIO.LOW, 0, duration)

def pivot_left(speed, duration):
    drive_motor(motor1_pwm, M1DIR, motor2_pwm, M2DIR, GPIO.LOW, GPIO.HIGH, speed, duration/2)
    drive_motor(motor1_pwm, M1DIR, motor2_pwm, M2DIR, GPIO.HIGH, GPIO.HIGH, speed, duration/2)

def pivot_right(speed, duration):
    drive_motor(motor1_pwm, M1DIR, motor2_pwm, M2DIR, GPIO.HIGH, GPIO.LOW, speed, duration/2)
    drive_motor(motor1_pwm, M1DIR, motor2_pwm, M2DIR, GPIO.HIGH, GPIO.HIGH, speed, duration/2)

def test1(speed = 25):
    
    # Drive forward at 25% speed for 2 seconds
    drive_forward(speed, 1)

    stop(1)

    # Drive backward at 25% speed for 1 second
    drive_backward(speed, 1)

    stop(1)

    # Turn left at 25% speed for 1 second
    turn_left(speed, 1)

    stop(1)

    # Turn right at 25% speed for 1 second
    turn_right(speed, 1)

    stop(1)

def test2():
    pulses_per_rotation = 50

    stop(1)

    # Turn left 90 degrees at 25% speed
    turn_left_degrees(25, 90, pulses_per_rotation, WHEELBASE)

    stop(1)

    # Turn right 90 degrees at 25% speed
    turn_right_degrees(25, 90, pulses_per_rotation, WHEELBASE)

    stop(1)

    # # Pivot left in place at 25% speed for 1 second
    # pivot_left(25, 1)

    # stop(1)

    # # Pivot right in place at 25% speed for 1 second
    # pivot_right(25, 1)

if __name__ == '__main__':
    init_drive_controls()
    try:
        test1()
        test1(30)
        # test2()

    finally:
        # Clean up the GPIO pins and stop the PWM signals
        motor1_pwm.stop()
        motor2_pwm.stop()
        GPIO.cleanup()
