import time
import threading
import math
import drive_controls.drive_controls
import board
import adafruit_gps
import serial
import lidar_module
import seed_planter
from bluetooth_gatt_server import main as bluetooth_server
from tau_lidar_camera.distance import cleanup, start, run_once, run
# from tau_lidar_camera import distance

uart = serial.Serial("/dev/ttyS0", baudrate=9600, timeout=10)
gps = adafruit_gps.GPS(uart, debug=False)  # Use UART/pyserial

location_stack = []
next_location = [53,-113]

last_gps_update = time.monotonic()
gps_location = [0,0]
last_gps_location = [0,0]
gps_vector = [0,0]
location_vector = [0,0]

# Set robot speed and turning speed
speed = 30
# turning_speed_dps = 100

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


def update_lidar_data():
    global obstacle_detected
    while True:
        obstacle_detected = lidar_module.detect_obstacle(obstacle_threshold)
        time.sleep(1)


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



if __name__ == "__main__":
    # Start bluetooth server
    bluetooth_server()

    #initialize gps
    gps.send_command(b"PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0")
    gps.send_command(b"PMTK220,1000")

    lidar_camera = start_lidar()
    

    # # Start LiDAR and GPS data update threads
    # lidar_thread = threading.Thread(target=update_lidar_data)
    # gps_thread = threading.Thread(target=update_gps_data)

    lidar_thread.start()
    gps_thread.start()

    drive_controls.init_drive_controls()

    # # Plant seeds
    # seed_planter.seed1()
    # seed_planter.seed2()

    # while True:
    try:
        for i in range(0,9):

            if run_once(lidar_camera) == "turn left":
                # Stop the robot and decide which way to turn
                drive_controls.stop(1)
                drive_controls.turn_left(speed, 1)
                drive_controls.stop(1)
                drive_controls.drive_forward(speed, 1)
                drive_controls.stop(1)
                drive_controls.turn_right(speed, 1)
                drive_controls.stop(1)
                continue
            
            elif run_once(lidar_camera) == "turn right":
                drive_controls.stop(1)
                drive_controls.turn_right(speed, 1)
                drive_controls.stop(1)
                drive_controls.drive_forward(speed, 1)
                drive_controls.stop(1)
                drive_controls.turn_left(speed, 1)
                drive_controls.stop(1)
                continue
            # else:
            #     # Calculate angle to target
            #     angle_to_target = calculate_bearing(
            #         (gps_location[0], gps_location[1]), (next_location[0], next_location[1])
            #     )

            #     # Turn the robot towards the target
            #     if angle_to_target > 0:
            #         drive_controls.turn_right_degrees(
            #             speed, angle_to_target, turning_speed_dps
            #         )
            #     else:
            #         drive_controls.turn_left_degrees(
            #             speed, -angle_to_target, turning_speed_dps
            #         )
            else:
                # Drive forward
                drive_controls.drive_forward(speed, 1)


    finally:
        # Clean up the GPIO pins and stop the PWM signals
        motor1_pwm.stop()
        motor2_pwm.stop()
        GPIO.cleanup()

        # Clean up everything
        cleanup(lidar_camera)