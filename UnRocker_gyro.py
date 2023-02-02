#!/usr/bin/env python

import os, signal
import subprocess
import time
import sys, argparse, math
from pymavlink import mavutil
from dronekit import connect, Command, LocationGlobal



DIALECT = 'custom'
global vehicle 

MAV_MODE_AUTO   = 4
if (sys.version_info[0] >= 3):
    ENCODING = 'ascii'
else:
    ENCODING = None

    def check_falldown(vehicle):
        if (vehicle.location.global_relative_frame.alt < 10.0):
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
                        out_file,
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
            if is_abnormal(vehicle=vehicle):
                print("Found abnormal behavior!")
                return True

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
        out_file.write(msg)

        return False


def fly_unrocker():

    res_fname = 'results/unrocker_gyro.txt'
    out= open(res_fname, 'w')
    MAX_COUNT = 40000
    attack_amp = 0.5
    attack_freq = 0

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

        home_position_set = False
        vehicle.parameters['sitl_gyro_trg'] = 0
        vehicle.parameters['sitl_gyro_trg'] = 0

        # Display basic vehicle state
        print " Type: %s" % vehicle._vehicle_type
        print " Armed: %s" % vehicle.armed
        print " System status: %s" % vehicle.system_status.state
        print " GPS: %s" % vehicle.gps_0
        print " Alt: %s" % vehicle.location.global_relative_frame.alt
        time.sleep(10)
    
    
        # Change to AUTO mode
        vehicle._master.mav.command_long_send(vehicle._master.target_system, 
                vehicle._master.target_component, mavutil.mavlink.MAV_CMD_DO_SET_MODE, 
                0, MAV_MODE_AUTO, 0, 0, 0, 0, 0, 0)

        time.sleep(5)

        vehicle.armed= True
        home = vehicle.location.global_relative_frame

        time.sleep(30)
        vehicle.parameters['sitl_gyro_amp'] = attack_amp
        vehicle.parameters['sitl_gyro_freq'] = attack_freq
        vehicle.parameters['sitl_gyro_trg'] = 1

        print "attack : %s" %vehicle.parameters['sitl_gyro_trg']

       
        crashed = check_roundtrip(vehicle=vehicle, out_file = out)
        if crashed:
            msg = 'Crashed at GYRO AMP: %f, FREQ: %d\n' % (attack_amp, attack_freq)
            print(msg)
            out.write(msg)
            attack_amp = 0.5
            attack_freq += 5

        else:
            msg = 'Not crashed at GYRO AMP: %f, FREQ: %d\n' % (attack_amp, attack_freq)
            print(msg)
            out.write(msg)
            attack_amp += 0.5
        if attack_amp > 4.5:
            msg = 'Crashed at GYRO AMP: 4.5, FREQ: %d\n' % (attack_freq)
            print(msg)
            out.write(msg)
            attack_amp = 0.5
            attack_freq += 5

        vehicle.parameters['sitl_gyro_trg'] = 0
        vehicle.armed = False
        time.sleep(3)

        subprocess.Popen("ps aux | grep gaze | awk '{print $2}' | xargs kill -9 ", shell=True)
        subprocess.Popen("ps aux | grep gzcli | awk '{print $2}' | xargs kill -9 ", shell=True)

        if attack_freq > 245:
            break
    out.close()
    self.progress("Iterative test is done.")


if __name__ == "__main__":
    fly_unrocker()
