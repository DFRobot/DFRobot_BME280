# -*- coding: utf-8 -*
'''!
  @file  read_data_i2c.py
  @brief  Download this demo to test read data from bme280, connect sensor through
  @n  IIC interface. Data will print on your serial monitor
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
  # i2c_addr = 0x76: pin SDO is low
  # i2c_addr = 0x77: pin SDO is high
'''
sensor = DFRobot_BME280_I2C(i2c_addr = 0x77, bus = 1)

def setup():
  while not sensor.begin():
    print ('Please check that the device is properly connected')
    time.sleep(3)
  print("sensor begin successfully!!!")

  '''
    @brief IIR filter coefficient setting(IIR filtering)
    @param - iir_config_coef Set IIR filter coefficient, configurable mode:
    @n         BME280_IIR_FILTER_SETTINGS[0], filter off
    @n         BME280_IIR_FILTER_SETTINGS[1], filter coefficient 2
    @n         BME280_IIR_FILTER_SETTINGS[2], filter coefficient 4
    @n         BME280_IIR_FILTER_SETTINGS[3], filter coefficient 8
    @n         BME280_IIR_FILTER_SETTINGS[4], filter coefficient 16
  '''
  sensor.set_config_filter(BME280_IIR_FILTER_SETTINGS[0])

  '''
    @brief Set output data rate in subdivision/sub-sampling mode (ODR:output data rates)
    @param odr_set The output data rate needs to be set, configurable mode:
    @n BME280_CONFIG_STANDBY_TIME_0P5, BME280_CONFIG_STANDBY_TIME_62P5, BME280_CONFIG_STANDBY_TIME_125, 
    @n BME280_CONFIG_STANDBY_TIME_250, BME280_CONFIG_STANDBY_TIME_500, BME280_CONFIG_STANDBY_TIME_1000, 
    @n BME280_CONFIG_STANDBY_TIME_10, BME280_CONFIG_STANDBY_TIME_20
    @return  return configuration results
    @retval True indicate configuration succeed
    @retval False indicate configuration failed and remains its original state
  '''
  sensor.set_config_T_standby(BME280_CONFIG_STANDBY_TIME_125)

  '''
    @brief Configure the oversampling when measuring temperature (OSR:over-sampling register)
    @details When the IIR filter is enabled, the temperature resolution is 20 bit. 
    @n       When the IIR filter is disabled, the temperature resolution is 16 + (osrs_t – 1) bit, 
    @n       e.g. 18 bit when osrs_t is set to ‘3’.
    @param - osrs_t 6 temp oversampling mode:
    @n         BME280_TEMP_OSR_SETTINGS[0], temperature sampling×0, Skipped (output set to 0x80000)
    @n         BME280_TEMP_OSR_SETTINGS[1], temperature sampling×2, 16 bit
    @n         BME280_TEMP_OSR_SETTINGS[2], temperature sampling×4, 17 bit
    @n         BME280_TEMP_OSR_SETTINGS[3], temperature sampling×8, 18 bit
    @n         BME280_TEMP_OSR_SETTINGS[4], temperature sampling×16, 19 bit
    @n         BME280_TEMP_OSR_SETTINGS[5], temperature sampling×32, 20 bit
  '''
  sensor.set_ctrl_meas_sampling_temp(BME280_TEMP_OSR_SETTINGS[3])

  '''
    @brief Configure the oversampling when measuring press (OSR:over-sampling register)
    @details When the IIR filter is enabled, the pressure resolution is 20 bit. 
    @n       When the IIR filter is disabled, the pressure resolution is 16 + (osrs_p – 1) bit, 
    @n       e.g. 18 bit when osrs_p is set to ‘3’.
    @param - osrs_t 6 temp oversampling mode:
    @n         BME280_PRESS_OSR_SETTINGS[0], pressure sampling×0, Skipped (output set to 0x80000)
    @n         BME280_PRESS_OSR_SETTINGS[1], pressure sampling×2, 16 bit
    @n         BME280_PRESS_OSR_SETTINGS[2], pressure sampling×4, 17 bit
    @n         BME280_PRESS_OSR_SETTINGS[3], pressure sampling×8, 18 bit
    @n         BME280_PRESS_OSR_SETTINGS[4], pressure sampling×16, 19 bit
    @n         BME280_PRESS_OSR_SETTINGS[5], pressure sampling×32, 20 bit
  '''
  sensor.set_ctrl_meas_sampling_press(BME280_PRESS_OSR_SETTINGS[3])

  '''
    @brief Configure the oversampling when measuring humidity (OSR:over-sampling register)
    @details For the humidity measurement, oversampling is possible to reduce the noise. 
    @n       The resolution of the humidity measurement is fixed at 16 bit ADC output.
    @param - osrs_t 6 temp oversampling mode:
    @n         BME280_HUMI_OSR_SETTINGS[0], humidity sampling×0, Skipped (output set to 0x80000)
    @n         BME280_HUMI_OSR_SETTINGS[1], humidity sampling×2, 16 bit
    @n         BME280_HUMI_OSR_SETTINGS[2], humidity sampling×4, 17 bit
    @n         BME280_HUMI_OSR_SETTINGS[3], humidity sampling×8, 18 bit
    @n         BME280_HUMI_OSR_SETTINGS[4], humidity sampling×16, 19 bit
    @n         BME280_HUMI_OSR_SETTINGS[5], humidity sampling×32, 20 bit
  '''
  sensor.set_ctrl_sampling_humi(BME280_HUMI_OSR_SETTINGS[3])

  '''
    # Configure measurement mode and power mode 
    # mode The measurement mode and power mode that need to set.
    #   SLEEP_MODE(Sleep mode): It will be in sleep mode by default after power-on reset. In this mode, no measurement is performed and power consumption is minimal. 
    #                           All registers are accessible for reading the chip ID and compensation coefficient.
    #   FORCED_MODE(Forced mode): In this mode, the sensor will take a single measurement according to the selected measurement and filtering options. After the measurement
    #                             is completed, the sensor will return to sleep mode, and the measurement result can be obtained in the register.
    #   NORMAL_MODE(Normal mode): Continuously loop between the measurement period and the standby period. The output data rates are related to the ODR mode setting.
  '''
  sensor.set_ctrl_meas_mode(NORMAL_MODE)

  time.sleep(2)   # Wait for configuration to complete
  '''
    # Calibrate the sensor according to the current altitude
    # In this example, we use an altitude of 540 meters in Wenjiang District of Chengdu (China). Please change to the local altitude when using it.
    # If this interface is not called, the measurement data will not eliminate the absolute difference
    # Notice: This interface is only valid for the first call
    # If you do not need to eliminate the absolute difference of measurement, please comment the following two lines
  '''
  if( sensor.calibrated_absolute_difference(540.0) == True ):
    print("Absolute difference base value set successfully!")


def loop():
  global flag
  if sensor.get_data_ready_status:
    # Read currently measured temperature date directly, unit: °C
    print("temperature : %.2f C" %(sensor.get_temperature))

    # Directly read the currently measured pressure data, unit: pa
    print("Pressure : %.2f Pa" %(sensor.get_pressure))

    # Read humidity, unit: %RH
    print("Humidity : %.2f %%RH" %(sensor.get_humidity))

    # Read altitude, unit: m
    print("Altitude : %.2f m" %(sensor.get_altitude))

    print()
    time.sleep(1)


if __name__ == "__main__":
  setup()
  while True:
    loop()
