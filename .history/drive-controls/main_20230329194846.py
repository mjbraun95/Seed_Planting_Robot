


import time
import math
from drive_controls import move_forward, turn_degrees
from gps_module import get_gps_coordinates

def calculate_bearing(point1, point2):
    lat1, lon1 = math.radians(point1[0]), math.radians(point1[1])
    lat2, lon2 = math.radians(point2[0]), math.radians(point2[1])

    d_lon = lon2 - lon1
    y = math.sin(d_lon) * math.cos(lat2)
    x = math.cos(lat1) * math.sin(lat2) - math.sin(lat1) * math.cos(lat2) * math.cos(d_lon)
    bearing = math.atan2(y, x)

    return math.degrees(bearing)

def main_loop():
    target_lat = -1 # TODO: import instructions code
    target_lon = -1 # TODO: import instructions code
    target_coordinates = (target_lat, target_lon)  # Enter your target GPS coordinates here

    while True:
        start_time = time.time()
        point1 = get_gps_coordinates()
        time.sleep(5)
        point2 = get_gps_coordinates()

        vector = (point2[0] - point1[0], point2[1] - point1[1])
        current_bearing = calculate_bearing(point1, point2)
        target_bearing = calculate_bearing(point2, target_coordinates)

        turn_angle = target_bearing - current_bearing
        if turn_angle > 180:
            turn_angle -= 360
        elif turn_angle < -180:
            turn_angle += 360

        turn_degrees(turn_angle)
        move_forward(5)

        end_time = time.time()
        loop_duration = end_time - start_time
        if loop_duration < 5:
            time.sleep(5 - loop_duration)

if __name__ == "__main__":
    main_loop()


    # Receive 1st GPS location
    # Drive straight (until obstacle) for x seconds
    # Receive 2nd GPS location