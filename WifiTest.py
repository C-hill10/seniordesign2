# SPDX-FileCopyrightText: 2019 ladyada for Adafruit Industries
# SPDX-License-Identifier: MIT
import time
from os import getenv

import adafruit_connection_manager
import adafruit_requests
import board
import busio
from digitalio import DigitalInOut
import neopixel


from adafruit_esp32spi.wifimanager import WiFiManager
from adafruit_esp32spi import adafruit_esp32spi
from adafruit_esp32spi import socketpool

print("ESP32 SPI webclient test")
# Get wifi details and more from a settings.toml file
# tokens used by this Demo: CIRCUITPY_WIFI_SSID, CIRCUITPY_WIFI_PASSWORD
ssid = getenv("CIRCUITPY_WIFI_SSID")
password = getenv("CIRCUITPY_WIFI_PASSWORD")

aio_username = getenv("ADAFRUIT_AIO_USERNAME")
aio_key = getenv("ADAFRUIT_AIO_KEY")



esp32_cs = DigitalInOut(board.GP13)
esp32_ready = DigitalInOut(board.GP14)
esp32_reset = DigitalInOut(board.GP15)

spi2 = busio.SPI(board.GP10, board.GP11, board.GP12)
esp = adafruit_esp32spi.ESP_SPIcontrol(spi2, esp32_cs, esp32_ready, esp32_reset)


"""Uncomment below for an externally defined RGB LED (including Arduino Nano Connect)"""
import adafruit_rgbled
from adafruit_esp32spi import PWMOut
RED_LED = PWMOut.PWMOut(esp, 21)
GREEN_LED = PWMOut.PWMOut(esp, 22)
BLUE_LED = PWMOut.PWMOut(esp, 27)
status_pixel = adafruit_rgbled.RGBLED(RED_LED, BLUE_LED, GREEN_LED)

wifi = WiFiManager(esp, ssid, password, status_pixel=status_pixel)



while True:
    try:
        print("Posting data...", end="")
        payload = {"value": 36.123}
        response = wifi.post(
            "https://io.adafruit.com/api/v2/" + aio_username + "/feeds/" + "lat" + "/data",
            json=payload,
            headers={"X-AIO-KEY": aio_key},
        )
        response.close()
        payload = {"value": -97.06}
        response = wifi.post(
            "https://io.adafruit.com/api/v2/" + aio_username + "/feeds/" + "long" + "/data",
            json=payload,
            headers={"X-AIO-KEY": aio_key},
        )
        payload = {"value": 24}
        response = wifi.post(
            "https://io.adafruit.com/api/v2/" + aio_username + "/feeds/" + "height" + "/data",
            json=payload,
            headers={"X-AIO-KEY": aio_key},
        )
        response.close()
        payload = {"value": 5}
        response = wifi.post(
            "https://io.adafruit.com/api/v2/" + aio_username + "/feeds/" + "sats" + "/data",
            json=payload,
            headers={"X-AIO-KEY": aio_key},
        )
        payload = {"value": 0.4}
        response = wifi.post(
            "https://io.adafruit.com/api/v2/" + aio_username + "/feeds/" + "accelx" + "/data",
            json=payload,
            headers={"X-AIO-KEY": aio_key},
        )
        response.close()
        payload = {"value": -0.6}
        response = wifi.post(
            "https://io.adafruit.com/api/v2/" + aio_username + "/feeds/" + "accely" + "/data",
            json=payload,
            headers={"X-AIO-KEY": aio_key},
        )
        payload = {"value": -9.8}
        response = wifi.post(
            "https://io.adafruit.com/api/v2/" + aio_username + "/feeds/" + "accelz" + "/data",
            json=payload,
            headers={"X-AIO-KEY": aio_key},
        )
        response.close()
        payload = {"value": 42.2}
        response = wifi.post(
            "https://io.adafruit.com/api/v2/" + aio_username + "/feeds/" + "magx" + "/data",
            json=payload,
            headers={"X-AIO-KEY": aio_key},
        )
        payload = {"value": 43.2}
        response = wifi.post(
            "https://io.adafruit.com/api/v2/" + aio_username + "/feeds/" + "magy" + "/data",
            json=payload,
            headers={"X-AIO-KEY": aio_key},
        )
        response.close()
        payload = {"value": 44.2}
        response = wifi.post(
            "https://io.adafruit.com/api/v2/" + aio_username + "/feeds/" + "magz" + "/data",
            json=payload,
            headers={"X-AIO-KEY": aio_key},
        )
        payload = {"value": -12.2}
        response = wifi.post(
            "https://io.adafruit.com/api/v2/" + aio_username + "/feeds/" + "gyrox" + "/data",
            json=payload,
            headers={"X-AIO-KEY": aio_key},
        )
        response.close()
        payload = {"value": -13.1}
        response = wifi.post(
            "https://io.adafruit.com/api/v2/" + aio_username + "/feeds/" + "gyroy" + "/data",
            json=payload,
            headers={"X-AIO-KEY": aio_key},
        )
        response.close()
        payload = {"value": 27.24}
        response = wifi.post(
            "https://io.adafruit.com/api/v2/" + aio_username + "/feeds/" + "gyroz" + "/data",
            json=payload,
            headers={"X-AIO-KEY": aio_key},
        )
        response.close()
        print("OK")
        time.sleep(15)
    except OSError as e:
        print("Failed to get data, retrying\n", e)
        wifi.reset()
        continue
    response = None
