# SPDX-FileCopyrightText: 2021 ladyada for Adafruit Industries
# SPDX-License-Identifier: MIT
from os import getenv
import adafruit_connection_manager
import adafruit_requests
from digitalio import DigitalInOut
import neopixel


from adafruit_esp32spi.wifimanager import WiFiManager
from adafruit_esp32spi import adafruit_esp32spi
from adafruit_esp32spi import socketpool
import time

import board
import busio
import adafruit_gps

from adafruit_lsm6ds.lsm6ds3 import LSM6DS3 as LSM6DS
from adafruit_lis3mdl import LIS3MDL

import adafruit_sdcard
import digitalio
import microcontroller
import storage

import os
import circuitpython_csv as csv # If using the library


ssid = getenv("CIRCUITPY_WIFI_SSID")
password = getenv("CIRCUITPY_WIFI_PASSWORD")

aio_username = getenv("ADAFRUIT_AIO_USERNAME")
aio_key = getenv("ADAFRUIT_AIO_KEY")
useWifi = getenv("USE_WIFI")
speed = getenv("WIFI_UPDATE_SPEED")

counterSpeed = 0
if speed == "1":
    counterSpeed = 1
if speed == "2":
    counterSpeed = 2
if speed == "3":
    counterSpeed = 5
if speed == "4":
    counterSpeed = 10
if speed == "5":
    counterSpeed = 30


led = digitalio.DigitalInOut(board.LED)
led.direction = digitalio.Direction.OUTPUT

esp32_cs = DigitalInOut(board.GP13)
esp32_ready = DigitalInOut(board.GP14)
esp32_reset = DigitalInOut(board.GP15)

spi2 = busio.SPI(board.GP10, board.GP11, board.GP12)
esp = adafruit_esp32spi.ESP_SPIcontrol(spi2, esp32_cs, esp32_ready, esp32_reset)

"""Uncomment below for an externally defined RGB LED (including Arduino Nano Connect)
import adafruit_rgbled
from adafruit_esp32spi import PWMOut
RED_LED = PWMOut.PWMOut(esp, 1)
GREEN_LED = PWMOut.PWMOut(esp, 8)
BLUE_LED = PWMOut.PWMOut(esp, 9)
status_pixel = adafruit_rgbled.RGBLED(RED_LED, BLUE_LED, GREEN_LED)
"""
wifi = WiFiManager(esp, ssid, password) #, status_pixel=status_pixel


# Create a serial connection for the GPS connection using default speed and
# a slightly higher timeout (GPS modules typically update once a second).
# These are the defaults you should use for the GPS FeatherWing.
# For other boards set RX = GPS module TX, and TX = GPS module RX pins.
#rx = board.RX  # Change to board.GP4 for Raspberry Pi Pico boards
rx = board.GP16
#tx = board.TX  # Change to board.GP5 for Raspberry Pi Pico boards
tx = board.GP17
uart = busio.UART(rx, tx, baudrate=9600, timeout=10)

# for a computer, use the pyserial library for uart access
# import serial
# uart = serial.Serial("/dev/ttyUSB0", baudrate=9600, timeout=10)

# If using I2C, we'll create an I2C interface to talk to using default pins
# i2c = board.I2C()  # uses board.SCL and board.SDA
# i2c = board.STEMMA_I2C()  # For using the built-in STEMMA QT connector on a microcontroller

# Create a GPS module instance.
gps = adafruit_gps.GPS(uart, debug=False)  # Use UART/pyserial
# gps = adafruit_gps.GPS_GtopI2C(i2c, debug=False)  # Use I2C interface

# Initialize the GPS module by changing what data it sends and at what rate.
# These are NMEA extensions for PMTK_314_SET_NMEA_OUTPUT and
# PMTK_220_SET_NMEA_UPDATERATE but you can send anything from here to adjust
# the GPS module behavior:
#   https://cdn-shop.adafruit.com/datasheets/PMTK_A11.pdf

# Turn on the basic GGA and RMC info (what you typically want)
gps.send_command(b"PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0")
# Turn on the basic GGA and RMC info + VTG for speed in km/h
# gps.send_command(b"PMTK314,0,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0")
# Turn on just minimum info (RMC only, location):
# gps.send_command(b'PMTK314,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0')
# Turn off everything:
# gps.send_command(b'PMTK314,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0')
# Turn on everything (not all of it is parsed!)
# gps.send_command(b'PMTK314,1,1,1,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0')

# Set update rate to once a second (1hz) which is what you typically want.
gps.send_command(b"PMTK220,1000")
# Or decrease to once every two seconds by doubling the millisecond value.
# Be sure to also increase your UART timeout above!
# gps.send_command(b'PMTK220,2000')
# You can also speed up the rate, but don't go too fast or else you can lose
# data during parsing.  This would be twice a second (2hz, 500ms delay):
# gps.send_command(b'PMTK220,500')

# Main loop runs forever printing the location, etc. every second.
last_print = time.monotonic()






#i2c = board.I2C()  # uses board.SCL and board.SDA
i2c = busio.I2C(board.GP27, board.GP26)
accel_gyro = LSM6DS(i2c)
mag = LIS3MDL(i2c)




SD_CS = board.GP5


# Connect to the card and mount the filesystem.
spi = busio.SPI(board.GP2, board.GP3, board.GP4)
cs = digitalio.DigitalInOut(SD_CS)
sdcard = adafruit_sdcard.SDCard(spi, cs)
vfs = storage.VfsFat(sdcard)
storage.mount(vfs, "/sd")

# Use the filesystem as normal! Our files are under /sd

filename = "PreDemoTest.csv"
if filename not in os.listdir("/sd"): # Adjust path for internal storage
    with open("/sd/" + filename, mode="w", encoding="utf-8") as f:
        csv_writer = csv.writer(f)
        csv_writer.writerow(["Date", "Time", "Satellites", "Latitude", "Longitude", "Height (Geoid) (m)",
            "X Accel (m/s^2)", "Y Accel (m/s^2)", "Z Accel (m/s^2)",
            "X Mag (uT)", "Y Mag (uT)", "Z Mag (uT)",
            "X Gyro (rad/s)", "Y Gyro (rad/s)", "Z Gyro (rad/s)"]) #headers


count = 0
minCount = 0
maxCount = 120
while True:
    # Make sure to call gps.update() every loop iteration and at least twice
    # as fast as data comes from the GPS unit (usually every second).
    # This returns a bool that's true if it parsed new data (you can ignore it
    # though if you don't care and instead look at the has_fix property).
    gps.update()
    # Every second print out current location details if there's a fix.
    current = time.monotonic()
    if current - last_print >= 0.5:
        count = count + counterSpeed
        last_print = current
        acceleration = accel_gyro.acceleration
        gyro = accel_gyro.gyro
        magnetic = mag.magnetic
        print("=" * 40)  # Print a separator line.
        print("Acceleration: X:{0:7.2f}, Y:{1:7.2f}, Z:{2:7.2f} m/s^2".format(*acceleration))
        print("Gyro          X:{0:7.2f}, Y:{1:7.2f}, Z:{2:7.2f} rad/s".format(*gyro))
        print("Magnetic      X:{0:7.2f}, Y:{1:7.2f}, Z:{2:7.2f} uT".format(*magnetic))
        print("")
        if useWifi == "true":
            try:
                if gps.has_fix:
                    if count == (120):
                    #if count > maxCount:
                        payload = {"value": gps.latitude}
                        #response = wifi.post(
                        wifi.post(
                            "https://io.adafruit.com/api/v2/" + aio_username + "/feeds/" + "lat" + "/data",
                            json=payload,
                            headers={"X-AIO-KEY": aio_key},
                        )
                        #response.close()
                    if count == (120):
                        payload = {"value": gps.longitude}
                        wifi.post(
                        #response = wifi.post(
                            "https://io.adafruit.com/api/v2/" + aio_username + "/feeds/" + "long" + "/data",
                            json=payload,
                            headers={"X-AIO-KEY": aio_key},
                        )
                        #response.close()
                    if count == (180):
                        payload = {"value": gps.height_geoid}
                        #response = wifi.post(
                        wifi.post(
                            "https://io.adafruit.com/api/v2/" + aio_username + "/feeds/" + "height" + "/data",
                            json=payload,
                            headers={"X-AIO-KEY": aio_key},
                        )
                        #response.close()
                    if count == (240):
                        payload = {"value": gps.satellites}
                        #response = wifi.post(
                        wifi.post(
                            "https://io.adafruit.com/api/v2/" + aio_username + "/feeds/" + "sats" + "/data",
                            json=payload,
                            headers={"X-AIO-KEY": aio_key},
                        )
                        #response.close()
                if count == (300):
                #if count > maxCount:
                    payload = {"value": acceleration[0]}
                    #response = wifi.post(
                    wifi.post(
                        "https://io.adafruit.com/api/v2/" + aio_username + "/feeds/" + "accelx" + "/data",
                        json=payload,
                        headers={"X-AIO-KEY": aio_key},
                    )
                    #response.close()
                if count == (360):
                    payload = {"value": acceleration[1]}
                    #response = wifi.post(
                    wifi.post(
                        "https://io.adafruit.com/api/v2/" + aio_username + "/feeds/" + "accely" + "/data",
                        json=payload,
                        headers={"X-AIO-KEY": aio_key},
                    )
                    #response.close()
                if count == (420):
                    payload = {"value": acceleration[2]}
                    #response = wifi.post(
                    wifi.post(
                        "https://io.adafruit.com/api/v2/" + aio_username + "/feeds/" + "accelz" + "/data",
                        json=payload,
                        headers={"X-AIO-KEY": aio_key},
                    )
                    #response.close()
                if count == (480):
                    payload = {"value": magnetic[0]}
                    #response = wifi.post(
                    wifi.post(
                        "https://io.adafruit.com/api/v2/" + aio_username + "/feeds/" + "magx" + "/data",
                        json=payload,
                        headers={"X-AIO-KEY": aio_key},
                    )
                    #response.close()
                if count == (540):
                    payload = {"value": magnetic[1]}
                    #response = wifi.post(
                    wifi.post(
                        "https://io.adafruit.com/api/v2/" + aio_username + "/feeds/" + "magy" + "/data",
                        json=payload,
                        headers={"X-AIO-KEY": aio_key},
                    )
                    #response.close()
                if count == (600):
                    payload = {"value": magnetic[2]}
                    #response = wifi.post(
                    wifi.post(
                        "https://io.adafruit.com/api/v2/" + aio_username + "/feeds/" + "magz" + "/data",
                        json=payload,
                        headers={"X-AIO-KEY": aio_key},
                    )
                    #response.close()
                if count == (660):
                    payload = {"value": gyro[0]}
                    #response = wifi.post(
                    wifi.post(
                        "https://io.adafruit.com/api/v2/" + aio_username + "/feeds/" + "gyrox" + "/data",
                        json=payload,
                        headers={"X-AIO-KEY": aio_key},
                    )
                    #response.close()
                if count == (720):
                    payload = {"value": gyro[1]}
                    #response = wifi.post(
                    wifi.post(
                        "https://io.adafruit.com/api/v2/" + aio_username + "/feeds/" + "gyroy" + "/data",
                        json=payload,
                        headers={"X-AIO-KEY": aio_key},
                    )
                    #response.close()
                if count == (780):
                    payload = {"value": gyro[2]}
                    #response = wifi.post(
                    wifi.post(
                        "https://io.adafruit.com/api/v2/" + aio_username + "/feeds/" + "gyroz" + "/data",
                        json=payload,
                        headers={"X-AIO-KEY": aio_key},
                    )
                    #response.close()
                    count = 0
            except OSError as e:
                wifi.reset()
                continue
            response = None
        
        if not gps.has_fix:
            # Try again if we don't have a fix yet.
            led.value = False
            print("Waiting for fix...")
            with open("/sd/" + filename, mode="a", encoding="utf-8") as f:
                csv_writer = csv.writer(f)
                csv_writer.writerow(["?", "?", "0", "?", "?", "?",
                "{0:7.2f}".format(*acceleration), "{1:7.2f}".format(*acceleration), "{2:7.2f}".format(*acceleration),
                "{0:7.2f}".format(*magnetic), "{1:7.2f}".format(*magnetic), "{2:7.2f}".format(*magnetic),
                "{0:7.2f}".format(*gyro), "{1:7.2f}".format(*gyro), "{2:7.2f}".format(*gyro)])
            continue
        # We have a fix! (gps.has_fix is true)
        led.value = True
        # Print out details about the fix like location, date, etc.
        with open("/sd/" + filename, mode="a", encoding="utf-8") as f:
            csv_writer = csv.writer(f)
            csv_writer.writerow(["{}/{}/{}".format(gps.timestamp_utc.tm_mon, gps.timestamp_utc.tm_mday, gps.timestamp_utc.tm_year),
            "{:02}:{:02}:{:02}".format(gps.timestamp_utc.tm_hour, gps.timestamp_utc.tm_min, gps.timestamp_utc.tm_sec),
            gps.satellites, gps.latitude, gps.longitude, gps.height_geoid,
            "{0:7.2f}".format(*acceleration), "{1:7.2f}".format(*acceleration), "{2:7.2f}".format(*acceleration),
            "{0:7.2f}".format(*magnetic), "{1:7.2f}".format(*magnetic), "{2:7.2f}".format(*magnetic),
            "{0:7.2f}".format(*gyro), "{1:7.2f}".format(*gyro), "{2:7.2f}".format(*gyro)])
        print(
            "Fix timestamp: {}/{}/{} {:02}:{:02}:{:02}".format(  # noqa: UP032
                gps.timestamp_utc.tm_mon,  # Grab parts of the time from the
                gps.timestamp_utc.tm_mday,  # struct_time object that holds
                gps.timestamp_utc.tm_year,  # the fix time.  Note you might
                gps.timestamp_utc.tm_hour,  # not get all data like year, day,
                gps.timestamp_utc.tm_min,  # month!
                gps.timestamp_utc.tm_sec,
            )
        )
        print(f"Latitude: {gps.latitude:.6f} degrees")
        print(f"Longitude: {gps.longitude:.6f} degrees")
        #print(f"Precise Latitude: {gps.latitude_degrees} degs, {gps.latitude_minutes:2.4f} mins")
        #print(f"Precise Longitude: {gps.longitude_degrees} degs, {gps.longitude_minutes:2.4f} mins")
        #print(f"Fix quality: {gps.fix_quality}")
        # Some attributes beyond latitude, longitude and timestamp are optional
        # and might not be present.  Check if they're None before trying to use!
        if gps.satellites is not None:
            print(f"# satellites: {gps.satellites}")
        #if gps.altitude_m is not None:
            #print(f"Altitude: {gps.altitude_m} meters")
        #if gps.speed_knots is not None:
            #print(f"Speed: {gps.speed_knots} knots")
        #if gps.speed_kmh is not None:
            #print(f"Speed: {gps.speed_kmh} km/h")
        #if gps.track_angle_deg is not None:
            #print(f"Track angle: {gps.track_angle_deg} degrees")
        #if gps.horizontal_dilution is not None:
            #print(f"Horizontal dilution: {gps.horizontal_dilution}")
        if gps.height_geoid is not None:
            print(f"Height geoid: {gps.height_geoid} meters")
