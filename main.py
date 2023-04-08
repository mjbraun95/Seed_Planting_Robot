import time
import threading
import math
import drive_controls
import adafruit_gps
import serial
# from bluetooth_gatt_server.example_gatt_server import main as bluetooth_server
# from tau_lidar_camera.distance import cleanup, start, run_once, run
from distance import cleanup, start_lidar, run_once, run

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



if __name__ == "__main__":
    # Set robot speed and turning speed
    speed = 30
    duration = 1
    # turning_speed_dps = 100
    
    # Start bluetooth server
    # bluetooth_server()

    # #initialize gps
    # gps.send_command(b"PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0")
    # gps.send_command(b"PMTK220,1000")

    lidar_camera = start_lidar()
    

    # # Start LiDAR and GPS data update threads
    # lidar_thread = threading.Thread(target=update_lidar_data)
    # gps_thread = threading.Thread(target=update_gps_data)

    # lidar_thread.start()
    # gps_thread.start()

    drive_controls.init_drive_controls()

    # # Plant seeds
    # seed_planter.seed1()
    # seed_planter.seed2()

    # while True:
    try:
        for i in range(0,9):
            next_step = run_once(lidar_camera)
            print("next_step: ", next_step)
            if next_step == "turn left":
                # Stop the robot and decide which way to turn
                # drive_controls.stop(0.25)
                drive_controls.turn_left(speed/3, duration*3)
                # drive_controls.stop(0.25)
                drive_controls.drive_forward(speed/2, duration*2)
                # drive_controls.stop(0.25)
                drive_controls.turn_right(speed/3, duration*3)
                # drive_controls.stop(0.25)
                continue
            
            elif next_step == "turn right":
                # drive_controls.stop(0.25)
                drive_controls.turn_right(speed/3, duration*3)
                # drive_controls.stop(0.25)
                drive_controls.drive_forward(speed, duration*2)
                # drive_controls.stop(0.25)
                drive_controls.turn_left(speed/3, duration*3)
                # drive_controls.stop(0.25)
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
                drive_controls.drive_forward(speed, 0.5)


    finally:
        # Clean up the GPIO pins and stop the PWM signals
        motor1_pwm.stop()
        motor2_pwm.stop()
        GPIO.cleanup()

        # Clean up everything
        cleanup(lidar_camera)