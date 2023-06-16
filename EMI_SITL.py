#!/usr/bin/env python

import argparse
import os
import random
import signal
import subprocess
import sys
import threading
import time
from typing import Tuple
from dronekit import Command, connect, LocationGlobal
from pymavlink import mavutil
import sys
import math


DIALECT = 'custom'
MAV_MODE_AUTO = 4
ENCODING = 'ascii' if sys.version_info[0] >= 3 else None


def get_wind_speed(start: int, end: int) -> int:
    wind = random.randint(start, end)
    print(f"INFO [Wind] {wind}")
    time.sleep(10)
    return wind


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

    return False


def get_location_offset_meters(original_location, dNorth, dEast, alt):
    earth_radius = 6378137.0  # Radius of "spherical" earth
    # Coordinate offsets in radians
    dLat = dNorth/earth_radius
    dLon = dEast/(earth_radius*(math.cos(math.pi*original_location.lat/180)))

    # New position in decimal degrees
    newlat = original_location.lat + (dLat * 180/math.pi)
    newlon = original_location.lon + (dLon * 180/math.pi)
    return LocationGlobal(newlat, newlon, original_location.alt+alt)


# main

def fly_unrocker() -> None:
    res_fname = 'results/unrocker_emi.txt'
    MAX_COUNT = 1
    attack_amp_g = 0
    attack_freq_g = 0
    attack_freq_a = 0
    attack_freq_m = 0

    out = open(res_fname, 'w')
    for m_count in range(MAX_COUNT):
        refresh = subprocess.Popen("make clean", shell=True)
        res = subprocess.Popen("make px4_sitl gazebo_solo", shell=True)
        time.sleep(10)
        # Connect to the Vehicle

        print("INFO Connecting")
        connection_string = '127.0.0.1:14540'
        vehicle = connect(connection_string, wait_ready=True)

        ############################################################################################
        # Listeners
        ############################################################################################
        home_position_set = True
        vehicle.parameters['sitl_gyro_trg'] = 0
        # Display basic vehicle state
        print("INFO [Type] %s" % vehicle._vehicle_type)
        print("INFO [Armed] %s" % vehicle.armed)
        print("INFO [System status] %s" % vehicle.system_status.state)
        print("INFO [GPS] %s" % vehicle.gps_0)
        print("INFO [Alt] %s" % vehicle.location.global_relative_frame.alt)
        print("INFO [Wind] %s" % vehicle.airspeed)
        # vehicle.send_calibrate_barometer()
        # print("INFO [Callibration Done] %s" % vehicle.airspeed)

        # Change to AUTO mode
        vehicle._master.mav.command_long_send(vehicle._master.target_system,
                                              vehicle._master.target_component, mavutil.mavlink.MAV_CMD_DO_SET_MODE,
                                              0, MAV_MODE_AUTO, 0, 0, 0, 0, 0, 0)
        # mission_file = "mission.txt"
        # upload_mission(mission_file,vehicle)
        time.sleep(1)
        cmds = vehicle.commands
        cmds.clear()

        vehicle.home_location = LocationGlobal(47.3983612,8.5473099,50)
        home = vehicle.home_location
        #home = vehicle.location.global_relative_frame
        print("INFO [home] %s" % home)
        # save_mission(mission_file,vehicle)
        # normal flight
        # vehicle.airspeed = get_wind_speed(1, 10)
        # vehicle.airspeed = get_wind_speed(1, 10)
        # vehicle.airspeed = get_wind_speed(1, 10)
        vehicle.armed= True

        wp = get_location_offset_meters(home, 0, 0, 20)
        cmd = Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                      mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 1, 0, 0, 0, 0, wp.lat, wp.lon, wp.alt)
        time.sleep(1)
        #cmds.add(cmd)
        vehicle.parameters['sitl_emi_gyro_amp'] = attack_amp_g
        vehicle.parameters['sitl_gyro_freq'] = attack_freq_g
        vehicle.parameters['sitl_acc_freq'] = attack_freq_a
        vehicle.parameters['sitl_mag_freq'] = attack_freq_m

        # attack trigger switch
        vehicle.parameters['sitl_emi_trg'] = 0

        # vehicle.parameters['sitl_emi_trg'] = 1
        print("INFO [attack triggered] %s" %
              vehicle.parameters['sitl_emi_trg'])
        print("INFO [attack gyro freqeuncy] %s" %
              vehicle.parameters['sitl_gyr_freq'])
        print("INFO [attack accel freqeuncy] %s" %
              vehicle.parameters['sitl_acc_freq'])

        vehicle.airspeed = get_wind_speed(1, 10)
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
        # land
        wp = get_location_offset_meters(home, 0, 0, 10)
        cmd = Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                      mavutil.mavlink.MAV_CMD_NAV_LAND, 0, 1, 0, 0, 0, 0, wp.lat, wp.lon, wp.alt)
        cmds.add(cmd)
        vehicle.close()
        out.flush()
        out.close()
        print("INFO [Test Done]")


if __name__ == "__main__":

    fly_unrocker()
