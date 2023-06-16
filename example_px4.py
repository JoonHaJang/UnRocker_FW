# Import DroneKit-Python
from dronekit import connect, Command, LocationGlobal
from pymavlink import mavutil
import time, sys, argparse, math
import random

################################################################################################
# Settings
################################################################################################

connection_string       = '127.0.0.1:14540'
MAV_MODE_AUTO   = 4
# https://github.com/PX4/Firmware/blob/master/Tools/mavlink_px4.py


# Parse connection argument
parser = argparse.ArgumentParser()
parser.add_argument("-c", "--connect", help="connection string")
args = parser.parse_args()

if args.connect:
    connection_string = args.connect


################################################################################################
# Init
################################################################################################

# Connect to the Vehicle
print ("Connecting")
vehicle = connect(connection_string, wait_ready=True)

def PX4setMode(mavMode):
    vehicle._master.mav.command_long_send(vehicle._master.target_system, vehicle._master.target_component,
                                               mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0,
                                               mavMode,
                                               0, 0, 0, 0, 0, 0)

def get_wind_speed(start: int, end: int) -> int:
    wind = random.randint(start, end)
    print(f"INFO [Wind] {wind}")
    time.sleep(10)
    return wind


def get_location_offset_meters(original_location, dNorth, dEast, alt):
    """
    Returns a LocationGlobal object containing the latitude/longitude `dNorth` and `dEast` metres from the
    specified `original_location`. The returned Location adds the entered `alt` value to the altitude of the `original_location`.
    The function is useful when you want to move the vehicle around specifying locations relative to
    the current vehicle position.
    The algorithm is relatively accurate over small distances (10m within 1km) except close to the poles.
    For more information see:
    http://gis.stackexchange.com/questions/2951/algorithm-for-offsetting-a-latitude-longitude-by-some-amount-of-meters
    """
    earth_radius=6378137.0 #Radius of "spherical" earth
    #Coordinate offsets in radians
    dLat = dNorth/earth_radius
    dLon = dEast/(earth_radius*math.cos(math.pi*original_location.lat/180))

    #New position in decimal degrees
    newlat = original_location.lat + (dLat * 180/math.pi)
    newlon = original_location.lon + (dLon * 180/math.pi)
    return LocationGlobal(newlat, newlon,original_location.alt+alt)


def get_random_integer():
    min_value = 1
    max_value = 20
    return random.randint(min_value, max_value)

################################################################################################
# Listeners
################################################################################################

home_position_set = False

#Create a message listener for home position fix
@vehicle.on_message('HOME_POSITION')
def listener(self, name, home_position):
    global home_position_set
    home_position_set = True

################################################################################################
# vareiable
################################################################################################
attack_amp_g = 0
attack_amp_a = 0
attack_amp_m = 0

attack_freq_g = 0
attack_freq_a = 0
attack_freq_m = 0

#set up attack_variable
attack_amp_g =  get_random_integer()*3
attack_amp_a =  get_random_integer()*3
attack_amp_m =  get_random_integer()*3

attack_freq_g = get_random_integer()/50
attack_freq_a = get_random_integer()/50
attack_freq_m = get_random_integer()/50
################################################################################################
# Start mission example
################################################################################################


# wait for a home position lock
while not home_position_set:
    print ("Waiting for home position...")
    time.sleep(1)

# Display basic vehicle state
print (" Type: %s" % vehicle._vehicle_type)
print (" Armed: %s" % vehicle.armed)
print (" System status: %s" % vehicle.system_status.state)
print (" GPS: %s" % vehicle.gps_0)
print (" Alt: %s" % vehicle.location.global_relative_frame.alt)
time.sleep(5)

# Change to AUTO mode
PX4setMode(MAV_MODE_AUTO)
time.sleep(1)

# Load commands
cmds = vehicle.commands
cmds.clear()

home = vehicle.location.global_relative_frame
# takeoff to 10 meters
wp = get_location_offset_meters(home, 0, 0, 20);
cmd = Command(0,0,0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 1, 0, 0, 0, 0, wp.lat, wp.lon, wp.alt)
cmds.add(cmd)
cmds.upload()

# Arm vehicle
vehicle.armed = True
time.sleep(30)
vehicle.airspeed = get_wind_speed(1, 10)
vehicle.parameters['sitl_gyr_amp'] = attack_amp_g
vehicle.parameters['sitl_acc_amp'] = attack_amp_a
vehicle.parameters['sitl_mag_amp'] = attack_amp_m

vehicle.parameters['sitl_gyr_freq'] = attack_freq_g
vehicle.parameters['sitl_acc_freq'] = attack_freq_a
vehicle.parameters['sitl_mag_freq'] = attack_freq_m
vehicle.parameters['sitl_emi_trg'] = 1
vehicle.parameters['sitl_def_trg'] = 0
vehicle.parameters['sitl_emi_log'] = 1

print("INFO [attack triggered] %s" % vehicle.parameters['sitl_emi_trg'])
print("INFO [attack gyro freqeuncy] %s" % vehicle.parameters['sitl_gyr_freq'])
print("INFO [attack accel freqeuncy] %s" % vehicle.parameters['sitl_acc_freq'])
print("INFO [attack mag freqeuncy] %s" % vehicle.parameters['sitl_mag_freq'])

print("INFO [attack gyro amplitude] %s" % vehicle.parameters['sitl_gyr_amp'])
print("INFO [attack accel amplitude] %s" % vehicle.parameters['sitl_acc_amp'])
print("INFO [attack mag amplitude] %s" % vehicle.parameters['sitl_mag_amp'])

time.sleep(30)

# land
wp = get_location_offset_meters(home, 0, 0, 10);
cmd = Command(0,0,0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_LAND, 0, 1, 0, 0, 0, 0, wp.lat, wp.lon, wp.alt)
cmds.add(cmd)

# Upload mission
cmds.upload()
time.sleep(2)
vehicle.armed = True


# monitor mission execution
#print(len(vehicle.commands))
#nextwaypoint = vehicle.commands.next
#while nextwaypoint < len(vehicle.commands):
#    if vehicle.commands.next > nextwaypoint:
#        display_seq = vehicle.commands.next+1
#        print ("Moving to waypoint %s" % display_seq)
#        nextwaypoint = vehicle.commands.next
#    time.sleep(1)

# wait for the vehicle to land
while vehicle.commands.next > 0:
    time.sleep(1)

vehicle.parameters['sitl_emi_trg'] = 0
vehicle.parameters['sitl_def_trg'] = 0
vehicle.parameters['sitl_emi_log'] = 0

# Disarm vehicle
vehicle.armed = False
time.sleep(1)

# Close vehicle object before exiting script
vehicle.close()
time.sleep(1)
