import time
import board

import adafruit_gps

import serial

uart = serial.Serial("/dev/ttyS0", baudrate=9600, timeout=10)
gps = adafruit_gps.GPS(uart, debug=False)  # Use UART/pyserial

location_stack = []
next_location = [53,-113]

last_gps_update = time.monotonic()
gps_location = [0,0]
last_gps_location = [0,0]
gps_vector = [0,0]
location_vector = [0,0]

def init_gps():
    gps.send_command(b"PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0")
    gps.send_command(b"PMTK220,1000")
    
def update_gps():
#     # Print out details about the fix like location, date, etc.
#     print("=" * 40)  # Print a separator line.
#     print(
#         "Fix timestamp: {}/{}/{} {:02}:{:02}:{:02}".format(
#             gps.timestamp_utc.tm_mon,  # Grab parts of the time from the
#             gps.timestamp_utc.tm_mday,  # struct_time object that holds
#             gps.timestamp_utc.tm_year,  # the fix time.  Note you might
#             gps.timestamp_utc.tm_hour,  # not get all data like year, day,
#             gps.timestamp_utc.tm_min,  # month!
#             gps.timestamp_utc.tm_sec,
#         )
#     )
    print("Latitude: {0:.6f} degrees".format(gps.latitude))
    print("Longitude: {0:.6f} degrees".format(gps.longitude))
    print("Fix quality: {}".format(gps.fix_quality))
    # Some attributes beyond latitude, longitude and timestamp are optional
    # and might not be present.  Check if they're None before trying to use!
#     if gps.satellites is not None:
#         print("# satellites: {}".format(gps.satellites))
#     if gps.altitude_m is not None:
#         print("Altitude: {} meters".format(gps.altitude_m))
#     if gps.speed_knots is not None:
#         print("Speed: {} knots".format(gps.speed_knots))
#     if gps.track_angle_deg is not None:
#         print("Track angle: {} degrees".format(gps.track_angle_deg))
#     if gps.horizontal_dilution is not None:
#         print("Horizontal dilution: {}".format(gps.horizontal_dilution))
#     if gps.height_geoid is not None:
#         print("Height geo ID: {} meters".format(gps.height_geoid))
    gps_location = [gps.latitude, gps.longitude]
    print(array)
    return array


def calc_orientation(gps_location, last_gps_location):
    gps_vector = [gps_location[0]-last_gps_location[0],gps_location[1]-last_gps_location[1]]
    print(gps_vector)
    return gps_vector

def orient_wheels(gps_vector, gps_location, next_location):
    location_vector = [next_location[0]-gps_location[0],next_location[1]-gps_location[1]]
    #while gps_vector != location_vector:
        #rotate
        #move
    return

def check_reached(location_vector):
    threshold = 0.00001
    if location_vector[0] < threshold or location_vector[1] < threshold:
        return True
    else:
        return False
    
    
    

init_gps()
last_print = time.monotonic()
while True:
    # Make sure to call gps.update() every loop iteration and at least twice
    
    gps.update()
    # as fast as data comes from the GPS unit (usually every second).
    # This returns a bool that's true if it parsed new data (you can ignore it
    # though if you don't care and instead look at the has_fix property).
    # Every second print out current location details if there's a fix.
    current = time.monotonic()
    if current - last_gps_update >= 1.0:
        last_gps_update = current
        if not gps.has_fix:
            # Try again if we don't have a fix yet.
            print("Waiting for fix...")
            continue
        update_gps()
        calc_orientation()
    
