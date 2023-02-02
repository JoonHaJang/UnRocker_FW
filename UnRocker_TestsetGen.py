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
        tstart = time.time()
        # this message arrives after we set the current WP
        start_wp = vehicle.commands.next
        current_wp = start_wp
        mode = vehicle.mode
        last_wp_msg = 0
	roundtrip_start = False
	roundtrip_check = False
        while time.time() < tstart + timeout:
            seq = vehicle.commands.next
            if time.time() - last_wp_msg > 1:
                last_wp_msg = time.time()
            if seq == current_wp+1: 
                print("test: Starting new waypoint %u" % seq)
                tstart = time.time()
                current_wp = seq
            if current_wp == 7:
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
        if sub_count == 6:
            sub_count = 0
            attack_amp = attack_amp + 1.0
        if attack_amp > 4.5:
            break

   
    self.progress("UnRocker test is done.")

if __name__ == "__main__":

    fly_unrocker()

