#!/usr/bin/env python

import subprocess
import sys
import os
import time


################################################################################################
# Terminals
################################################################################################
def open_terminal(command):
    if sys.platform == 'win32':
        subprocess.Popen(['start', 'cmd.exe', '/k', command], shell=True)
    elif sys.platform == 'linux' or sys.platform == 'linux2':
        subprocess.Popen(['gnome-terminal', '--', 'bash', '-c', command])
    elif sys.platform == 'darwin':
        subprocess.Popen(['open', '-a', 'Terminal.app', 'bash', '-c', command])
    else:
        print("Unsupported platform:", sys.platform)
        return


commands1 = subprocess.Popen("make px4_sitl gazebo_solo", shell=True)
commands2 = subprocess.Popen("python3 take_off.py", shell= True)
