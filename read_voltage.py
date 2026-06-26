#!/usr/bin/env python3
"""
pH Sensor Voltage Reader
Reads raw voltage from ADS1115 channel A0. Ctrl+C to stop.
"""
import time
import board
import busio
import adafruit_ads1x15.ads1115 as ADS
from adafruit_ads1x15.analog_in import AnalogIn

i2c  = busio.I2C(board.SCL, board.SDA)
ads  = ADS.ADS1115(i2c)
ads.gain = 2 / 3
chan = AnalogIn(ads, 0)

print("Reading pH sensor voltage -- Ctrl+C to stop.\n")
while True:
    print(f"  {chan.voltage:.5f} V")
    time.sleep(0.5)
