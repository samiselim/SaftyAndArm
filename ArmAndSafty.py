import time
import math
import RPi.GPIO as GPIO
from pymavlink import mavutil
from pymavlink.mavutil import location

master=mavutil.mavlink_connection("/dev/ttyAMA0", baud=57600)

def distance(lat1, long1, lat2, long2):
    R = 6371  # Earth's radius in kilometers
    lat1, long1, lat2, long2 = map(math.radians, [lat1, long1, lat2, long2])

    dlat = lat2 - lat1
    dlong = long2 - long1
    a = math.sin(dlat/2)**2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlong/2)**2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
    distance = R * c

    return distance

def location1(relative_alt=False):
    '''return current location'''
#    self.wait_gps_fix()
    # wait for another VFR_HUD, to ensure we have correct altitude
    master.recv_match(type='VFR_HUD', blocking=True)
    master.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
    if relative_alt:
        alt = master.messages['GLOBAL_POSITION_INT'].relative_alt*0.001
    else:
        alt = master.messages['VFR_HUD'].alt
    return location(master.messages['GPS_RAW_INT'].lat*1.0e-7,
                    master.messages['GPS_RAW_INT'].lon*1.0e-7,
                    alt,
                    master.messages['VFR_HUD'].heading)

Arm_Flag = False
Loiter_Flag = False
home = None

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
GPIO.setup(18, GPIO.OUT)

while True:
    if master.motors_armed() != 0 and not Arm_Flag and location1().lat != 0:
        Arm_Flag = True
        home = location1()
        print(f"armed and home location is : lat --> {home.lat}  long --> {home.lng}")

    msg = master.recv_match()
    if not msg:
      continue


    if home is not None:
        cl = location1()
        print(round(distance(home.lat, home.lng, cl.lat, cl.lng) * 1000) , "meter", end="\r")
        if distance(home.lat, home.lng, cl.lat, cl.lng) > 0.01:
            GPIO.output(18, GPIO.HIGH)
            master.set_servo(7, 1900)
        else:
            GPIO.output(18, GPIO.LOW)
            master.set_servo(7, 1100)

        if cl.lat == 0 and not Loiter_Flag:
            master.set_mode_loiter()
            Loiter_Flag = True
            print("loiter")
        elif cl.lat != 0 and Loiter_Flag:
            master.set_mode_auto()
            Loiter_Flag = False
            print("auto")

