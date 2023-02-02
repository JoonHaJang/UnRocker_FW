#!/usr/bin/env python

import os, signal
import subprocess
import time
import sys, argparse, math
from pymavlink import mavutil
from dronekit import connect, Command, LocationGlobal
import fcntl

DIALECT = 'custom'
global vehicle , usb_bus, usb_device, f


usb_bus = subprocess.getoutput('lsusb | grep 26ac:0032 | awk \'{print $2}\' ')
usb_device = subprocess.getoutput('lsusb | grep 26ac:0032 | awk \'{print $4}\' ')

print(usb_bus)
print(usb_device)

subprocess.getoutput('echo password | sudo -S chmod 777 /dev/bus/usb/%s/%s' %(usb_bus,usb_device[:-1]))

time.sleep(1)

f = open('/dev/bus/usb/%s/%s'%(usb_bus,usb_device[:-1]), 'w', os.O_WRONLY)

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
                    timeout=1000):
    tstart = time.time()
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

def check_tty():
    tty_check = subprocess.getoutput('dmesg | grep tty | awk \'{print $5}\' ')
    tty_check_list = list(tty_check)
    A = (tty_check[-2] == '0')
    print( 'TTY Result : %s' % tty_check[-2] )
    print( 'results : %s' % A)

def check_usb():
    global usb_device, f
    usb_bus_new = subprocess.getoutput('lsusb | grep 26ac:0032 | awk \'{print $2}\' ')
    usb_device_new = subprocess.getoutput('lsusb | grep 26ac:0032 | awk \'{print $4}\' ')
    print(usb_bus_new)
    print(usb_device_new)

    if usb_device != usb_device_new:
        print('change USB device')
        subprocess.getoutput('echo password | sudo -S chmod 777 /dev/bus/usb/%s/%s' %(usb_bus_new,usb_device_new[:-1]))
        usb_device = usb_device_new
        time.sleep(1)
        f = open('/dev/bus/usb/%s/%s'%(usb_bus,usb_device[:-1]), 'w', os.O_WRONLY)
        time.sleep(10)

def fly_runrocker():
    global f
    res_fname = 'results/unrocker_acc.txt'
    out= open(res_fname, 'w')
    MAX_COUNT = 40000
    attack_amp = 0.5
    attack_freq = 5000
    for m_count in range(MAX_COUNT):
        time.sleep(10)
        while check_tty() == False:
            print('ttyACM0 false.. rebooting')
            fcntl.ioctl(f, 21780, 0)
            time.sleep(30)
        time.sleep(1)
        check_usb()

        res = subprocess.Popen("gazebo Tools/sitl_gazebo/worlds/solo.world",shell=True)
        time.sleep(20)
    
        # Connect to the Vehicle
        print ("Connecting")
        connection_string = '127.0.0.1:14540'

        vehicle = connect(connection_string, wait_ready=True)
        vehicle.wait_ready(True, timeout=300)

        #####
        vehicle.parameters['hitl_gyro_trg'] = 0
        time.sleep(1)
        vehicle.parameters['hitl_accel_trg'] = 0
        time.sleep(1)
        vehicle.reboot()
        time.sleep(3)
        print('Pixhawk rebooting again')

        subprocess.Popen("ps aux | grep gaze | awk '{print $2}' | xargs kill -9 ", shell=True)
        subprocess.Popen("ps aux | grep gzcli | awk '{print $2}' | xargs kill -9 ", shell=True)
        
        time.sleep(10)
        while check_tty() == False:
            print('ttyACM0 false.. rebooting')
            fcntl.ioctl(f, 21780, 0)
            time.sleep(30)
        time.sleep(1)
        check_usb()
       
        res = subprocess.Popen("gazebo Tools/sitl_gazebo/worlds/solo.world",shell=True)
        time.sleep(10)
    
        # Connect to the Vehicle
        print ("Connecting again")
        connection_string = '127.0.0.1:14540'
        vehicle = connect(connection_string, wait_ready=True)
        vehicle.wait_ready(True, timeout=300)

        #####
        home_position_set = False
        vehicle.parameters['hitl_accel_trg'] = 0
        time.sleep(1)
        vehicle.parameters['hitl_gyro_trg'] = 0
        time.sleep(15)
        # Display basic vehicle state
        print (" Type: %s" % vehicle._vehicle_type)
        print (" Armed: %s" % vehicle.armed)
        print (" System status: %s" % vehicle.system_status.state)
        print (" GPS: %s" % vehicle.gps_0)
        print (" Alt: %s" % vehicle.location.global_relative_frame.alt)
        time.sleep(10)
    
        vehicle._master.mav.command_long_send(vehicle._master.target_system, 
                vehicle._master.target_component, mavutil.mavlink.MAV_CMD_DO_SET_MODE, 
                0, MAV_MODE_AUTO, 0, 0, 0, 0, 0, 0)

        time.sleep(5)

        vehicle.armed= True
        print ("Arm commanded")

        home = vehicle.location.global_relative_frame

        time.sleep(30)
        vehicle.parameters['hitl_accel_amp'] = attack_amp*20
        vehicle.parameters['hitl_accel_freq'] = attack_freq
        vehicle.parameters['hitl_accel_trg'] = 1

        print ("attack : %s" %vehicle.parameters['hitl_accel_trg'])

        crashed = check_roundtrip(vehicle=vehicle, out_file = out)
        if crashed:
            msg = 'Crashed at AMP: %f, FREQ: %d\n' % (attack_amp*20, attack_freq)
            print(msg)
            out.write(msg)
            attack_amp = 0.5
            attack_freq += 5
        else:
            msg = 'Not crashed at AMP: %f, FREQ: %d\n' % (attack_amp*20, attack_freq)
            print(msg)
            out.write(msg)
            attack_amp += 0.5
        if attack_amp > 5.0:
            msg = 'Crashed at AMP: 100, FREQ: %d\n' % (attack_freq)
            print(msg)
            out.write(msg)
            attack_amp = 0.5
            attack_freq += 5
        vehicle.parameters['hitl_accel_trg'] = 0
        vehicle.armed = False
        time.sleep(2)

        vehicle.reboot()
        time.sleep(3)
        print('Pixhawk rebooting')

        subprocess.Popen("ps aux | grep gaze | awk '{print $2}' | xargs kill -9 ", shell=True)
        subprocess.Popen("ps aux | grep gzcli | awk '{print $2}' | xargs kill -9 ", shell=True)
        
        time.sleep(5)
        if attack_freq > 5245:
            break
    out.close()
    self.progress("UnRocker test is done.")

if __name__ == "__main__":
    fly_unrocker()

