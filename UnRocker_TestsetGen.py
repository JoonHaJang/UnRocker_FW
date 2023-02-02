#!/usr/bin/env python

import os, signal
import subprocess
import time
import sys, argparse, math
from pymavlink import mavutil
from dronekit import connect, Command, LocationGlobal
import commands

DIALECT = 'custom'
global vehicle 

MAV_MODE_AUTO   = 4
if (sys.version_info[0] >= 3):
    ENCODING = 'ascii'
else:
    ENCODING = None

    def check_falldown(vehicle):
        if (vehicle.location.global_relative_frame.alt < 15.0):
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

    # Checks if there exists an abnormal behavior during a roundtrip.
    # This function is based on wait_wapoint in common.py
    def check_roundtrip(vehicle,
                        timeout=600):
        tstart = time.time()#self.get_sim_time()
        # this message arrives after we set the current WP
        start_wp = vehicle.commands.next#_master.mav.waypoint_current()
        current_wp = start_wp
        mode = vehicle.mode#_master.mav.flightmode
        last_wp_msg = 0
	roundtrip_start = False
	roundtrip_check = False
        while time.time() < tstart + timeout:
            seq = vehicle.commands.next#self.mav.waypoint_current()
            if time.time() - last_wp_msg > 1:
                last_wp_msg = time.time()#self.get_sim_time_cached()
            if seq == current_wp+1: #or (seq > current_wp+1 and allow_skip):
                print("test: Starting new waypoint %u" % seq)
                tstart = time.time()#self.get_sim_time()
                current_wp = seq
            if current_wp == 7:#wpnum_end and wp_dist < max_dist:
                print("Reached final waypoint %u" % seq)
                return False
            if seq >= 255:
                print("Reached final waypoint %u" % seq)
                return False
            if seq > current_wp+1:
                print("WHAT? Skipped waypoint: %u -> %u" % (seq, current_wp+1))

        msg = 'Time-out occurred\n'
        print(msg)

        return False

def fly_unrocker():
    MAX_COUNT = 40000
    attack_amp = 0##temp
    gyro_attack_freq = 206
    accel_attack_freq = 1830
    sub_count = 0#temp

    for m_count in range(MAX_COUNT):
        res = subprocess.Popen("make px4_sitl gazebo_solo",shell=True)
        time.sleep(10)
    
        # Connect to the Vehicle
        print "Connecting"
        connection_string = '127.0.0.1:14540'
        vehicle = connect(connection_string, wait_ready=True)
        ############################################################################################
        # Listeners
        ############################################################################################
        vehicle.parameters['sitl_accel_trg'] = 0
        vehicle.parameters['sitl_gyro_trg'] = 0
        vehicle.parameters['sitl_accel_log'] = 1
        vehicle.parameters['sitl_gyro_log'] = 1
        vehicle.parameters['sitl_accel_freq'] = accel_attack_freq
        vehicle.parameters['sitl_gyro_freq'] = gyro_attack_freq

        if sub_count == 0:
            time.sleep(5)
            vehicle.parameters['sitl_accel_amp'] = attack_amp * 20
            vehicle.parameters['sitl_gyro_amp'] = attack_amp

        else :
            time.sleep(5)
            vehicle._master.mav.command_long_send(vehicle._master.target_system, 
                vehicle._master.target_component, mavutil.mavlink.MAV_CMD_DO_SET_MODE, 
                0, MAV_MODE_AUTO, 0, 0, 0, 0, 0, 0)

            time.sleep(3)

            vehicle.armed= True
            home = vehicle.location.global_relative_frame
            time.sleep(50)

            check_roundtrip(vehicle=vehicle)
            time.sleep(50)

            vehicle.parameters['sitl_accel_trg'] = 0
            vehicle.parameters['sitl_gyro_trg'] = 0

        time.sleep(1)
        subprocess.Popen("ps aux | grep gaze | awk '{print $2}' | xargs kill -9 ", shell=True)
        subprocess.Popen("ps aux | grep gzcli | awk '{print $2}' | xargs kill -9 ", shell=True)
        time.sleep(2)

        sub_count = sub_count + 1
        if sub_count == 5:
            sub_count = 0
            attack_amp = attack_amp + 1.0
        if attack_amp > 4.5:
            break

   
    self.progress("UnRocker test is done.")

def start_MAVProxy_SITL(master='udp:127.0.0.1:14550'):#:5760'):##udp
    """Launch mavproxy connected to a SITL instance."""
    import pexpect
    #global close_list
    MAVPROXY = os.getenv('MAVPROXY_CMD', 'mavproxy.py')
    cmd = MAVPROXY + ' --master=%s --out=127.0.0.1:18570' % master##14540
    ret = pexpect.spawn("mavproxy.py --master=udp:127.0.0.1:14550 --out=127.0.0.1:1857", encoding=ENCODING, timeout=60)#cmd, encoding=ENCODING, timeout=60)
    ret.delaybeforesend = 0
    return ret


def tests(self):
    return [
        ("UnRocker Dataset Generator",
             "Generate UnRocker Dataset (takeoff)",
         self.fly_unrocker),
    ]

def fly_test(mavproxy):
    print("test")
    mavproxy.send('commander takeoff\n')

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



if __name__ == "__main__":

    fly_unrocker()

