# -*- coding: utf-8 -*
'''!
  @file  read_data_spi.py
  @brief  Download this demo to test read data from bme280, connect sensor through
  @n  spi interface connect cs pin to io 2. Data will print on your serial monitor
  @copyright  Copyright (c) 2010 DFRobot Co.Ltd (http://www.dfrobot.com)
  @license  The MIT License (MIT)
  @author  [qsjhyy](yihuan.huang@dfrobot.com)
  @version  V1.0
  @date  2022-08-04
  @url  https://github.com/DFRobot/DFRobot_BME280
'''
from __future__ import print_function
import sys
import os
sys.path.append(os.path.dirname(os.path.dirname(os.path.realpath(__file__))))

from DFRobot_BME280 import *

'''
  # SPI communications
'''
sensor = DFRobot_BME280_SPI(cs=8, bus=0, dev=0, speed=8000000)

def setup():
  while not sensor.begin():
    print ('Please check that the device is properly connected')
    time.sleep(3)
  print("sensor begin successfully!!!")

def loop():
  # Read currently measured temperature date directly, unit: °C
  print("temperature : %.2f C" %(sensor.get_temperature))

  # Directly read the currently measured pressure data, unit: pa
  print("Pressure : %.2f Pa" %(sensor.get_pressure))

  # Read humidity, unit: %RH
  print("Humidity : %.2f %%RH" %(sensor.get_humidity))

  # Read altitude, unit: m
  print("Altitude : %.2f m" %(sensor.get_altitude))

  print()
  time.sleep(3)

if __name__ == "__main__":
  setup()
  while True:
    loop()
