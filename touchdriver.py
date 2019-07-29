#!/usr/bin/env python

import os
import signal
import sys
import time


try:
    from evdev import uinput, UInput, AbsInfo, ecodes as e
except ImportError:
    exit("Install: sudo pip install evdev")

try:
    import RPi.GPIO as gpio
except ImportError:
    exit("Install: sudo pip install RPi.GPIO")

try:
    import smbus
except ImportError:
    exit("Install: sudo apt-get install python-smbus")


os.system("sudo modprobe uinput")

DAEMON = True

CAPABILITIES = {
    e.EV_ABS : (
        (e.ABS_X, AbsInfo(value=0, min=0, max=800, fuzz=0, flat=0, resolution=1)),
        (e.ABS_Y, AbsInfo(value=0, min=0, max=480, fuzz=0, flat=0, resolution=1)),
        (e.ABS_MT_SLOT, AbsInfo(value=0, min=0, max=1, fuzz=0, flat=0, resolution=0)),
        (e.ABS_MT_TRACKING_ID, AbsInfo(value=0, min=0, max=65535, fuzz=0, flat=0, resolution=0)),
        (e.ABS_MT_POSITION_X, AbsInfo(value=0, min=0, max=800, fuzz=0, flat=0, resolution=0)),
        (e.ABS_MT_POSITION_Y, AbsInfo(value=0, min=0, max=480, fuzz=0, flat=0, resolution=0)),
    ),
    e.EV_KEY : [
        e.BTN_TOUCH, 
    ]
}

PIDFILE = "/var/run/touch.pid"


if DAEMON:
    try:
        pid = os.fork()
        if pid > 0:
            sys.exit(0)

    except OSError, e:
        print("Fork #1 failed: {} ({})".format(e.errno, e.strerror))
        sys.exit(1)


    os.chdir("/")
    os.setsid()
    os.umask(0)

    try:
        pid = os.fork()
        if pid > 0:
            fpid = open(PIDFILE, 'w')
            fpid.write(str(pid))
            fpid.close()
            sys.exit(0)
    except OSError, e:
        print("Fork #2 failed: {} ({})".format(e.errno, e.strerror))
        sys.exit(1)


    si = file("/dev/null", 'r')
    so = file("/dev/null", 'a+', 0)
    se = file("/dev/null", 'a+', 0)

    os.dup2(si.fileno(), sys.stdin.fileno())
    os.dup2(so.fileno(), sys.stdout.fileno())
    os.dup2(se.fileno(), sys.stderr.fileno())

try:
    ui = UInput(CAPABILITIES, name="Touchscreen", bustype=e.BUS_USB)

except uinput.UInputError as e:
    sys.stdout.write(e.message)
    sys.stdout.write("Running as root? sudo ...".format(sys.argv[0]))
    sys.exit(0)

INT = 26
ADDR = 0x5c

gpio.setmode(gpio.BCM)
gpio.setwarnings(False)
gpio.setup(INT, gpio.IN)

bus = smbus.SMBus(2)

last_x1  = -1
last_y1  = -1
start = 0
stop = 0


def touchint(channel):
  global start,stop
  start = 1
  stop = 1  
      
   


gpio.add_event_detect(INT, gpio.RISING, callback=touchint, bouncetime=200)      #touch interrupt


def write_status(x, y):
            global start
            ui.write(e.EV_ABS, e.ABS_MT_SLOT, 0)
            ui.write(e.EV_ABS, e.ABS_X, x)
            ui.write(e.EV_ABS, e.ABS_Y, y)
            ui.write(e.EV_ABS, e.ABS_MT_POSITION_X, x)
            ui.write(e.EV_ABS, e.ABS_MT_POSITION_Y, y)
            if start:
              ui.write(e.EV_ABS, e.ABS_MT_TRACKING_ID, 0)
              ui.write(e.EV_KEY, e.BTN_TOUCH, 1)
              start = 0
            ui.write(e.EV_SYN, e.SYN_REPORT, 0)
            ui.syn()
            
              

def smbus_read_touch():
    

    try:
        data = bus.read_i2c_block_data(ADDR, 0x40, 8)

        x = data[0] | (data[4] << 8)
        y = data[1] | (data[5] << 8)
        
        if (0 < x < 800)  and (0 < y < 480):
         write_status(800-x, 480-y)
        
    except:
        pass

bus.write_byte_data(0x5c,0x6e,0b00001110)

while True:
        if gpio.input(INT):
            smbus_read_touch()
        elif stop:
                 stop = 0                 
                 ui.write(e.EV_ABS, e.ABS_MT_SLOT, 0)
                 ui.write(e.EV_ABS, e.ABS_MT_TRACKING_ID, -1)
                 ui.write(e.EV_KEY, e.BTN_TOUCH, 0)
                 ui.write(e.EV_SYN, e.SYN_REPORT, 0)
                 ui.syn()
        time.sleep(0.02)
ui.close()
