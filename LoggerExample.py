# SPDX-FileCopyrightText: 2017 Limor Fried for Adafruit Industries
#
# SPDX-License-Identifier: MIT

import time

import adafruit_sdcard
import board
import busio
import digitalio
import microcontroller
import storage

import os
import circuitpython_csv as csv # If using the library

# Use any pin that is not taken by SPI
SD_CS = board.GP5

led = digitalio.DigitalInOut(board.LED)
led.direction = digitalio.Direction.OUTPUT

# Connect to the card and mount the filesystem.
spi = busio.SPI(board.GP2, board.GP3, board.GP4)
cs = digitalio.DigitalInOut(SD_CS)
sdcard = adafruit_sdcard.SDCard(spi, cs)
vfs = storage.VfsFat(sdcard)
storage.mount(vfs, "/sd")

# Use the filesystem as normal! Our files are under /sd

print("Logging temperature to filesystem")
filename = "Exampledatelog.csv"
if filename not in os.listdir("/sd"): # Adjust path for internal storage
    with open("/sd/" + filename, mode="w", encoding="utf-8") as f:
        csv_writer = csv.writer(f)
        csv_writer.writerow(["Timestamp", "Temperature_C", "Humidity_Percent"]) # Example headers
# append to the file!
while True:
    with open("/sd/" + filename, mode="a", encoding="utf-8") as f:
        csv_writer = csv.writer(f)
        csv_writer.writerow(["ExTime", "ExTemp", "ExHumididty"])
    # open file for append
    #with open("/sd/temperature.txt", "a") as f:
    #    led.value = True  # turn on LED to indicate we're writing to the file
    #    t = microcontroller.cpu.temperature
    #    print("Temperature = %0.1f" % t)
    #    f.write("%0.1f\n" % t)
    #    led.value = False  # turn off LED to indicate we're done
    # file is saved
    time.sleep(1)

