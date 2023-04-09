import time
import threading
import math
import drive_controls
import adafruit_gps
import serial
# from bluetooth_gatt_server.example_gatt_server import main as bluetooth_server
# from tau_lidar_camera.distance import cleanup, start, run_once, run
from distance import cleanup, start_lidar, run_once, run
import motor as seed_drill_motor
import RPi.GPIO as GPIO
global motor1_pwm
global motor2_pwm

# uart = serial.Serial("/dev/ttyS0", baudrate=9600, timeout=10)
# gps = adafruit_gps.GPS(uart, debug=False)  # Use UART/pyserial

# location_stack = []
# next_location = [53,-113]

# last_gps_update = time.monotonic()
# gps_location = [0,0]
# last_gps_location = [0,0]
# gps_vector = [0,0]
# location_vector = [0,0]



def calculate_bearing(point1, point2):
    lat1, lon1 = math.radians(point1[0]), math.radians(point1[1])
    lat2, lon2 = math.radians(point2[0]), math.radians(point2[1])

    d_lon = lon2 - lon1
    y = math.sin(d_lon) * math.cos(lat2)
    x = math.cos(lat1) * math.sin(lat2) - math.sin(lat1) * math.cos(lat2) * math.cos(
        d_lon
    )
    bearing = math.atan2(y, x)

    return math.degrees(bearing)


def check_reached(location_vector):
    threshold = 0.00001
    if location_vector[0] < threshold or location_vector[1] < threshold:
        return True
    else:
        return False

def calc_orientation(gps_location, last_gps_location):
    gps_vector = [gps_location[0]-last_gps_location[0],gps_location[1]-last_gps_location[1]]
    print(gps_vector)
    return gps_vector

def update_gps_data():
    while True:
        gps.update()
        current = time.monotonic()
        if current - last_gps_update >= 1.0:
            last_gps_update = current
            if not gps.has_fix:
                # Try again if we don't have a fix yet.
                print("Waiting for fix...")
                continue
            gps_location = [gps.latitude, gps.longitude]
            calc_orientation()

# servo = GPIO.PWM(11,50) # Define servo as PWM, pulse 50Hz


def rotate(angle, servo):
    servo.ChangeDutyCycle(2+(angle/18))
    time.sleep(0.5) # gives time for motor to turn
    servo.ChangeDutyCycle(0)

def seed1(servo):
    rotate(80, servo) # plant seed
    time.sleep(0.75) # give time to drop seeds
    rotate(93, servo) # return to neutral
    time.sleep(0.5)

def seed2(servo):
    rotate(93, servo) # plant seed
    time.sleep(0.75) # give time to drop seeds
    rotate(80, servo) # return to neutral
    time.sleep(0.5)

# seed1()
# seed2()



if __name__ == "__main__":
    # Set robot speed and turning speed
    speed = 30
    duration = 5
    # turning_speed_dps = 100
    
    # Start bluetooth server
    # bluetooth_server()

    # #initialize gps
    # gps.send_command(b"PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0")
    # gps.send_command(b"PMTK220,1000")

    lidar_camera = start_lidar()
    
    # GPIO.setmode(GPIO.BOARD)
    GPIO.setmode(GPIO.BCM)
    # GPIO.setup(29,GPIO.OUT) # Set pin 29 as output)
    GPIO.setup(5,GPIO.OUT) # Set pin 29 as output)
    
    # global servo
    servo = GPIO.PWM(11,50) # Define servo as PWM, pulse 50Hz
    servo.start(0) # Start PWM with pulse off
    
    # # Start LiDAR and GPS data update threads
    # lidar_thread = threading.Thread(target=update_lidar_data)
    # gps_thread = threading.Thread(target=update_gps_data)

    # lidar_thread.start()
    # gps_thread.start()

    drive_controls.init_drive_controls()
    seed_drill_motor.init_seed_drill_motor()

    # # Plant seeds


    # while True:
    try:
        drive_controls.drive_forward(speed, duration)
        # seed_drill_motor.seed1()
        drive_controls.stop(1)
        # seed_planter.seed1(servo)
        # seed_planter.seed2(servo)
        drive_controls.turn_left(speed, 0.5)
        
        # for i in range(0,9):
        #     next_step = run_once(lidar_camera)
        #     print("next_step: ", next_step)
        #     if next_step == "turn left":
        #         # Stop the robot and decide which way to turn
        #         drive_controls.turn_left(speed/3, duration*3)
        #         drive_controls.drive_forward(speed/2, duration*2)
        #         drive_controls.turn_right(speed/3, duration*3)
        #         drive_controls.drive_forward(speed, 0.5)
        #         drive_controls.drive_forward(speed, 0.5)
        #         drive_controls.drive_forward(speed, 0.5)
        #         drive_controls.drive_forward(speed, 0.5)
        #         drive_controls.drive_forward(speed, 0.5)
        #         drive_controls.drive_forward(speed, 0.5)
        #         continue
            
        #     elif next_step == "turn right":
        #         drive_controls.turn_right(speed/3, duration*3)
        #         drive_controls.drive_forward(speed/2, duration*2)
        #         drive_controls.turn_left(speed/3, duration*3)
        #         drive_controls.drive_forward(speed, 0.5)
        #         drive_controls.drive_forward(speed, 0.5)
        #         drive_controls.drive_forward(speed, 0.5)
        #         drive_controls.drive_forward(speed, 0.5)
        #         drive_controls.drive_forward(speed, 0.5)
        #         drive_controls.drive_forward(speed, 0.5)
        #         continue
        #     else:
        #         # Drive forward
        #         drive_controls.drive_forward(speed, 0.5)


    finally:
        # Clean up the GPIO pins and stop the PWM signals
        motor1_pwm.stop()
        motor2_pwm.stop()
        seed_drill_motor.stop_seed_drill_motor()
        print("Finished planting seeds.")
        
        GPIO.cleanup()

        # Clean up everything
        cleanup(lidar_camera)