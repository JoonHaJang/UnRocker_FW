#!/usr/bin/env python

# Import DroneKit-Python
from dronekit import connect, Command, LocationGlobal
from pymavlink import mavutil
import time
import sys
import argparse
import math
import subprocess
import os

################################################################################################
# Settings
################################################################################################

DIALECT = 'custom'
MAV_MODE_AUTO = 4
ENCODING = 'ascii' if sys.version_info[0] >= 3 else None
MAV_MODE_AUTO = 4
# https://github.com/PX4/Firmware/blob/master/Tools/mavlink_px4.py


################################################################################################
# methods for mission
################################################################################################

def download_mission(vehicle):
    """
    Downloads the current mission and returns it in a list.
    It is used in save_mission() to get the file information to save.
    """
    print(" Download mission from vehicle")
    missionlist = []
    cmds = vehicle.commands
    cmds.download()
    cmds.wait_ready()
    for cmd in cmds:
        missionlist.append(cmd)
    return missionlist


def readmission(aFileName, vehicle):
    """
    Load a mission from a file into a list. The mission definition is in the Waypoint file
    format (http://qgroundcontrol.org/mavlink/waypoint_protocol#waypoint_file_format).

    This function is used by upload_mission().
    """
    print("\nReading mission from file: %s" % aFileName)
    cmds = vehicle.commands
    missionlist = []
    with open(aFileName) as f:
        for i, line in enumerate(f):
            if i == 0:
                print("ok")
                # if not line.startswith('QGC WPL 110'):
                #    raise Exception('File is not supported WP version')
            else:
                linearray = line.split('\t')
                ln_index = int(linearray[0])
                ln_currentwp = int(linearray[1])
                ln_frame = int(linearray[2])
                ln_command = int(linearray[3])
                ln_param1 = float(linearray[4])
                ln_param2 = float(linearray[5])
                ln_param3 = float(linearray[6])
                ln_param4 = float(linearray[7])
                ln_param5 = float(linearray[8])
                ln_param6 = float(linearray[9])
                ln_param7 = float(linearray[10])
                ln_autocontinue = int(linearray[11].strip())
                cmd = Command(0, 0, 0, ln_frame, ln_command, ln_currentwp, ln_autocontinue,
                              ln_param1, ln_param2, ln_param3, ln_param4, ln_param5, ln_param6, ln_param7)
                missionlist.append(cmd)
    return missionlist


def upload_mission(aFileName, vehicle):
    """
    Upload a mission from a file.
    """
    # Read mission from file
    missionlist = readmission(aFileName, vehicle)

    print("\nUpload mission from a file: %s" % aFileName)
    # Clear existing mission from vehicle
    print(' Clear mission')
    cmds = vehicle.commands
    cmds.clear()
    # Add new mission to vehicle
    for command in missionlist:
        cmds.add(command)
    print('Upload mission')
    vehicle.commands.upload()


def save_mission(aFileName, vehicle):
    """
    Save a mission in the Waypoint file format
    (http://qgroundcontrol.org/mavlink/waypoint_protocol#waypoint_file_format).
    """
    print("\nSave mission from Vehicle to file: %s" % aFileName)
    # Download mission from vehicle
    missionlist = download_mission(vehicle)
    # Add file-format information
    output = 'QGC WPL 110\n'
    # Add home location as 0th waypoint
    home = vehicle.home_location
    output += "%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\n" % (
        0, 1, 0, 16, 0, 0, 0, 0, home.lat, home.lon, home.alt, 1)
    # Add commands
    for cmd in missionlist:
        commandline = "%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\n" % (
            cmd.seq, cmd.current, cmd.frame, cmd.command, cmd.param1, cmd.param2, cmd.param3, cmd.param4, cmd.x, cmd.y, cmd.z, cmd.autocontinue)
        output += commandline
    with open(aFileName, 'w') as file_:
        print(" Write mission to file")
        file_.write(output)


################################################################################################
# methods
################################################################################################
refresh = subprocess.Popen("make clean", shell=True)


def PX4setMode(mavMode, vehicle):
    vehicle._master.mav.command_long_send(vehicle._master.target_system, vehicle._master.target_component,
                                          mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0,
                                          mavMode,
                                          0, 0, 0, 0, 0, 0)


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
    earth_radius = 6378137.0  # Radius of "spherical" earth
    # Coordinate offsets in radians
    dLat = dNorth/earth_radius
    dLon = dEast/(earth_radius*math.cos(math.pi*original_location.lat/180))

    # New position in decimal degrees
    newlat = original_location.lat + (dLat * 180/math.pi)
    newlon = original_location.lon + (dLon * 180/math.pi)
    print(LocationGlobal(newlat, newlon, original_location.alt+alt))
    return LocationGlobal(newlat, newlon, original_location.alt+alt)


def check_falldown(vehicle):
    if (vehicle.location.global_relative_frame.alt == None):
        vehicle.location.global_relative_frame.alt = 0
    if (vehicle.location.global_relative_frame.alt < 1.0):
        return True
    else:
        return False


def check_goheaven(vehicle):
    if (vehicle.location.global_relative_frame.alt > 150000.0):
        return True
    else:
        return False

    # TODO: need to add more checks


def is_abnormal(vehicle):
    if check_falldown(vehicle=vehicle):
        return True
    elif check_goheaven(vehicle=vehicle):
        return True
    else:
        return False


def check_roundtrip(vehicle, out_file, timeout=600):
    start_wp = vehicle.commands.next
    current_wp = start_wp
    mode = vehicle.mode
    last_wp_msg = 0
    roundtrip_start = False
    roundtrip_check = False
    tstart = time.time()
    while time.time() < tstart + timeout:
        if is_abnormal(vehicle):
            print("Found abnormal behavior!")
            return True

        seq = vehicle.commands.next
        if time.time() - last_wp_msg > 1:
            last_wp_msg = time.time()
        if seq == current_wp+1:
            print("test: Starting new waypoint %u" % seq)
            tstart = time.time()
            current_wp = seq
        if current_wp == 7 or seq >= 255:
            print("Reached final waypoint %u" % seq)
            return False
        if seq > current_wp+1:
            print("WHAT? Skipped waypoint: %u -> %u" % (seq, current_wp+1))

    msg = 'Time-out occurred\n'
    print(msg)
    out_file.write(msg)


################################################################################################
# main methods
################################################################################################
def fly_EMI():
    res_fname = 'results/unrocker_emi.txt'
    # iteration number
    MAX_COUNT = 1
    connection_string = '127.0.0.1:14540'
    # attack variables
    attack_amp_g = 0
    attack_freq_g = 0
    attack_freq_a = 0
    attack_freq_m = 0
    out = open(res_fname, 'w')
    for m_count in range(MAX_COUNT):
        # Parse connection argument
        parser = argparse.ArgumentParser()
        parser.add_argument("-c", "--connect", help="connection string")
        args = parser.parse_args()

        if args.connect:
            connection_string = args.connect


################################################################################################
# Init
################################################################################################
        time.sleep(5)

# Connect to the Vehicle
        print("Connecting")
        vehicle = connect(connection_string, wait_ready=True)


################################################################################################
# Listeners
################################################################################################
        home_position_set = False


################################################################################################
# Start mission example
################################################################################################

        # Display basic vehicle state
        print(" Type: %s" % vehicle._vehicle_type)
        print(" Armed: %s" % vehicle.armed)
        print(" System status: %s" % vehicle.system_status.state)
        print(" GPS: %s" % vehicle.gps_0)
        print(" Alt: %s" % vehicle.location.global_relative_frame.alt)
        time.sleep(5)
        home_position_set = True

        # Change to AUTO mode
        PX4setMode(MAV_MODE_AUTO, vehicle)
        time.sleep(1)
        # Load commands
        cmds = vehicle.commands
        cmds.clear()

        home = vehicle.location.global_relative_frame.alt
        # Change to AUTO mode
        wp = get_location_offset_meters(home, 0, 0, 10);
        cmd = Command(0,0,0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 1, 0, 0, 0, 0, wp.lat, wp.lon, wp.alt)
        cmds.add(cmd)

# Upload mission
        cmds.upload()
        time.sleep(30)


        crashed = check_roundtrip(vehicle=vehicle, out_file=out)
        if crashed:
            msg = 'Crashed at GYRO AMP: %f, FREQ: %d\n' % (
                attack_amp_g, attack_freq_g)
            print("INFO %s" % msg)
            out.write(msg)
            attack_amp_g = 0.5
            attack_freq_g += 1
            vehicle.armed = False
        else:
            msg = 'Not crashed at GYRO AMP: %f, FREQ: %d\n' % (
                attack_amp_g, attack_freq_g)
            print(msg)
            out.write(msg)
            attack_amp_g += 0.5
            vehicle.armed = False
        if attack_amp_g > 4.5:
            msg = 'Crashed at GYRO AMP: 4.5, FREQ: %d\n' % (attack_freq_g)
            print(msg)
            out.write(msg)
            attack_amp_g = 0.5
            attack_freq_g += 1

        vehicle.parameters['sitl_emi_trg'] = 0
        vehicle.armed = False
        time.sleep(5)
        subprocess.Popen(
            "ps aux | grep gaze | awk '{print $2}' | xargs kill -9 ", shell=True)
        subprocess.Popen(
            "ps aux | grep gzcli | awk '{print $2}' | xargs kill -9 ", shell=True)

        if attack_freq_g > 245:
            break


# monitor mission execution
# nextwaypoint = vehicle.commands.next
# while nextwaypoint < len(vehicle.commands):
#    if vehicle.commands.next > nextwaypoint:
#        display_seq = vehicle.commands.next+1
#        print("Moving to waypoint %s" % display_seq)
#        nextwaypoint = vehicle.commands.next
#    time.sleep(30)

# wait for the vehicle to land
        while vehicle.commands.next > 0:
            time.sleep(1)

# Disarm vehicle
        vehicle.armed = False
        time.sleep(1)

# Close vehicle object before exiting script
        vehicle.close()
        time.sleep(1)
        out.flush()
        out.close()
        print("Test Done")

################################################################################################
# Start Main
################################################################################################


if __name__ == "__main__":

    fly_EMI()

'''
# move 10 meters north
wp = get_location_offset_meters(wp, 10, 0, 0);
cmd = Command(0,0,0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 1, 0, 0, 0, 0, wp.lat, wp.lon, wp.alt)
cmds.add(cmd)

# move 10 meters east
wp = get_location_offset_meters(wp, 0, 10, 0);
cmd = Command(0,0,0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 1, 0, 0, 0, 0, wp.lat, wp.lon, wp.alt)
cmds.add(cmd)

# move 10 meters south
wp = get_location_offset_meters(wp, -10, 0, 0);
cmd = Command(0,0,0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 1, 0, 0, 0, 0, wp.lat, wp.lon, wp.alt)
cmds.add(cmd)

# move 10 meters west
wp = get_location_offset_meters(wp, 0, -10, 0);
cmd = Command(0,0,0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 1, 0, 0, 0, 0, wp.lat, wp.lon, wp.alt)
cmds.add(cmd)
'''
