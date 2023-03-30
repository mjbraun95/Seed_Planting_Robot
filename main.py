import time
import threading
import math
import drive_controls.drive_controls
import gps_module
import lidar_module
import seed_planter
from bluetooth_gatt_server import main as bluetooth_server

# Set target GPS coordinates here
target_lat = 0.0
target_lon = 0.0

# Set robot speed and turning speed
speed = 50
turning_speed_dps = 100

# Define obstacle detection threshold
obstacle_threshold = 1000


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


def update_gps_data():
    global current_lat, current_lon
    while True:
        current_lat, current_lon = gps_module.get_lat_lon()
        time.sleep(1)


if __name__ == "__main__":
    # Start bluetooth server
    bluetooth_server()

    # Start LiDAR and GPS data update threads
    lidar_thread = threading.Thread(target=update_lidar_data)
    gps_thread = threading.Thread(target=update_gps_data)

    lidar_thread.start()
    gps_thread.start()

    # Plant seeds
    seed_planter.seed1()
    seed_planter.seed2()

    while True:
        if obstacle_detected:
            # Stop the robot and decide which way to turn
            drive_controls.drive_forward(0, 0)
            drive_controls.turn_left(speed, 1)

            # Check again for obstacles after turning
            if obstacle_detected:
                # Turn the other way
                drive_controls.turn_right(speed, 2)
        else:
            # Calculate angle to target
            angle_to_target = calculate_bearing(
                (current_lat, current_lon), (target_lat, target_lon)
            )

            # Turn the robot towards the target
            if angle_to_target > 0:
                drive_controls.turn_right_degrees(
                    speed, angle_to_target, turning_speed_dps
                )
            else:
                drive_controls.turn_left_degrees(
                    speed, -angle_to_target, turning_speed_dps
                )

            # Drive forward
            drive_controls.drive_forward(speed, 1)
