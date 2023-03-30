import time
import threading
import gps_module
import lidar_module
import seed_planter
import drive_controls

# Set target GPS coordinates here
target_lat = 0.0
target_lon = 0.0

# Set robot speed and turning speed
speed = 50
turning_speed_dps = 100

# Define obstacle detection threshold
obstacle_threshold = 1000

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
        angle_to_target = gps_module.calculate_angle_to_target(current_lat, current_lon, target_lat, target_lon)
        
        # Turn the robot towards the target
        if angle_to_target > 0:
            drive_controls.turn_right_degrees(speed, angle_to_target, turning_speed_dps)
        else:
            drive_controls.turn_left_degrees(speed, -angle_to_target, turning_speed_dps)
        
        # Drive forward
        drive_controls.drive_forward(speed, 1)

