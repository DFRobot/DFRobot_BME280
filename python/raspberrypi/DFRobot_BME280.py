# -*- coding: utf-8 -*
'''!
  @file  DFRobot_BME280.py
  @brief  Define the infrastructure of DFRobot_BME280 class
  @details  It provides both SPI and I2C interfaces, which make it easy to make fast prototypes.
  @n  The sensor is especially adept in air pressure measurement; it has an offset 
  @n  temperature coefficient of ±1.5 Pa/K, equiv. to ±12.6 cm at 1 °C temperature change.
  @copyright  Copyright (c) 2010 DFRobot Co.Ltd (http://www.dfrobot.com)
  @license  The MIT License (MIT)
  @author  [qsjhyy](yihuan.huang@dfrobot.com)
  @version  V1.0
  @date  2022-08-04
  @url  https://github.com/DFRobot/DFRobot_BME280
'''
from calendar import c
from re import S
import sys
import time

import smbus
import spidev
import RPi.GPIO as GPIO

import logging
from ctypes import *

logger = logging.getLogger()
# logger.setLevel(logging.INFO)   # Display all print information
logger.setLevel(logging.FATAL)   # If you don’t want to display too many prints, only print errors, please use this option
ph = logging.StreamHandler()
formatter = logging.Formatter("%(asctime)s - [%(filename)s %(funcName)s]:%(lineno)d - %(levelname)s: %(message)s")
ph.setFormatter(formatter) 
logger.addHandler(ph)

## I2C communication address when SDO is grounded
DFROBOT_BME280_I2C_ADDR_SDO_GND = 0x76
## I2C communication address when SDO is connected to power
DFROBOT_BME280_I2C_ADDR_SDO_VDD = 0x77
## BME280 chip version
BME280_REG_CHIP_ID_DEFAULT = 0x60

# BME280 register address
## 0x88-0xA1 is calibration data, calib00..calib25
BME280_CALIB_DATA_00_25 = 0x88
## The “CHIP_ID” register contains the chip identification code.
BME280_CHIP_ID = 0xD0
## Triggers a reset, all user configuration settings are overwritten with their default state.
BME280_CMD_RESET = 0xE0
## 0xE1-0xF0 is calibration data, calib26..calib41
BME280_CALIB_DATA_26_41 = 0xE1
## The “CTRL_HUM” register
BME280_CTRL_HUM = 0xF2
## The Sensor Status Flags are stored in the “STATUS” register.
BME280_STATUS = 0xF3
## The “CTRL_MEAS” register
BME280_CTRL_MEAS = 0xF4
## The “CONFIG” register
BME280_CONFIG = 0xF5
## The 24Bit pressure data is split and stored in three consecutive registers.
BME280_PRESS_DATA_MSB = 0xF7
## The 24Bit temperature data is split and stored in three consecutive registered.
BME280_TEMP_DATA_MSB = 0xFA
## The 16Bit temperature data is split and stored in two consecutive registered.
BME280_HUM_DATA_MSB = 0xFD

# Sensor configuration
## Sleep mode: It will be in sleep mode by default after power-on reset. In this mode, no measurement is performed and power consumption is minimal. 
##             All registers are accessible for reading the chip ID and compensation coefficient.
SLEEP_MODE  = 0x00
## Forced mode: In this mode, the sensor will take a single measurement according to the selected measurement and filtering options. After the
##              measurement is completed, the sensor will return to sleep mode, and the measurement result can be obtained in the register.
FORCED_MODE = 0x01
## Normal mode: Continuously loop between the measurement period and the standby period. The output data rates are related to the ODR mode setting.
NORMAL_MODE = 0x03

## temperature oversampling settings
BME280_TEMP_OSR_SETTINGS = (0x00, 0x20, 0x40, 0x60, 0x80, 0xA0)
## pressure oversampling settings
BME280_PRESS_OSR_SETTINGS = (0x00, 0x04, 0x08, 0x0C, 0x10, 0x14)
## humidity oversampling settings
BME280_HUMI_OSR_SETTINGS = (0x00, 0x01, 0x02, 0x03, 0x04, 0x05)

## IIR filter settings
BME280_IIR_FILTER_SETTINGS = (0x00, 0x04, 0x08, 0x0C, 0x10)

# Controls inactive duration tstandby in normal mode.
## Controls inactive duration tstandby; ODR 2000Hz; Sampling period:0.5 ms
BME280_CONFIG_STANDBY_TIME_0P5  = 0x00
## Sampling period:62.5 ms
BME280_CONFIG_STANDBY_TIME_62P5 = 0x20
## Sampling period:125 ms
BME280_CONFIG_STANDBY_TIME_125  = 0x40
## Sampling period:250 ms
BME280_CONFIG_STANDBY_TIME_250  = 0x60
## Sampling period:500 ms
BME280_CONFIG_STANDBY_TIME_500  = 0x80
## Sampling period:1000 ms
BME280_CONFIG_STANDBY_TIME_1000 = 0xA0
## Sampling period:10 ms
BME280_CONFIG_STANDBY_TIME_10   = 0xC0
## Sampling period:20 ms
BME280_CONFIG_STANDBY_TIME_20   = 0xE0

## Triggers a reset, all user configuration settings are overwritten with their default state.
BME280_CMD_RESET_VALUE = 0xB6

## Standard sea level pressure, unit: pa
STANDARD_SEA_LEVEL_PRESSURE_PA = 101325.00


class DFRobot_BME280(object):
    '''!
      @brief define DFRobot_BME280 base class
      @details for driving the pressure sensor
    '''

    def __init__(self):
        '''!
          @brief Module init
        '''
        # Sea level pressure in Pa.
        self.sea_level_pressure = STANDARD_SEA_LEVEL_PRESSURE_PA
        self._t_fine = 0

    def begin(self):
        '''!
          @brief Initialize sensor
          @return  return initialization status
          @retval True indicate initialization succeed
          @retval False indicate initialization failed
        '''
        ret = False
        chip_id = self._read_reg(BME280_CHIP_ID, 1)
        logger.info(chip_id[0])
        if chip_id[0] == BME280_REG_CHIP_ID_DEFAULT:
            self.reset()
            time.sleep(0.3)
            self._get_coefficients()
            self.set_config_filter(BME280_IIR_FILTER_SETTINGS[0])
            self.set_config_T_standby(BME280_CONFIG_STANDBY_TIME_125)
            self.set_ctrl_meas_sampling_temp(BME280_TEMP_OSR_SETTINGS[3])
            self.set_ctrl_meas_sampling_press(BME280_PRESS_OSR_SETTINGS[3])
            self.set_ctrl_sampling_humi(BME280_HUMI_OSR_SETTINGS[3])
            self.set_ctrl_meas_mode(NORMAL_MODE)
            time.sleep(2)   # warm-up
            ret = True
        return ret

    @property
    def get_temperature(self):
        '''!
          @brief Get pressure measurement value from register, working range (-40 ~ +85 °C)
          @return Return temperature measurements, unit: °C
        '''
        data = self._read_reg(BME280_TEMP_DATA_MSB, 3)
        raw = data[0] << 12 | data[1] << 4 | (data[2] & 0x0F)

        # datasheet, Trimming Coefficient listing in register map with size and sign attributes
        t1, t2, t3, p1, p2, p3, p4, p5, p6, p7, p8, p9, h1, h2, h3, h4, h5, h6 = self._data_calib

        v1 = ((((raw >> 3) - (t1 << 1))) * t2) >> 11
        v2 = (((((raw >> 4) - t1) * ((raw >> 4) - t1)) >> 12) * t3) >> 14
        self._t_fine = v1 + v2
        rslt = (self._t_fine * 5 + 128) >> 8
        return (float(rslt) / 100)   # round((rslt / 100), 2)

    @property
    def get_pressure(self):
        '''!
          @brief Get pressure measurement value from register, working range (300 ~ 1100 hPa)
          @return Return pressure measurements, unit: Pa
          @attention If the reference value is provided before, the absolute value of the current 
          @n         position pressure is calculated according to the calibrated sea level atmospheric pressure
        '''
        data = self._read_reg(BME280_PRESS_DATA_MSB, 3)
        raw = data[0] << 12 | data[1] << 4 | (data[2] & 0x0F)

        # datasheet, Trimming Coefficient listing in register map with size and sign attributes
        t1, t2, t3, p1, p2, p3, p4, p5, p6, p7, p8, p9, h1, h2, h3, h4, h5, h6 = self._data_calib

        self.get_temperature   # update _t_fine

        v1 = self._t_fine - 128000
        v2 = v1 * v1 * p6
        v2 = v2 + ((v1 * p5) << 17)
        v2 = v2 + (p4 << 35)
        v1 = ((v1 * v1 * p3) >> 8) + ((v1 * p2) << 12)
        v1 = (((1 << 47) + v1)) * p1 >> 33
        if(v1 == 0):
          return 0
        rslt = 1048576 - raw
        rslt = (((rslt << 31) - v2) * 3125) / v1
        v1 = (p9 * (int(rslt) >> 13) * (int(rslt) >> 13)) >> 25
        v2 = (p8 * int(rslt)) >> 19
        rslt = ((int(rslt) + v1 + v2) >> 8) + (p7 << 4)
        return (float(rslt) / 256)

    @property
    def get_humidity(self):
        '''!
          @brief Get humidity measurement value from register, working range (0 ~ 100 %RH)
          @return Return humidity measurements, unit: %RH
        '''
        data = self._read_reg(BME280_HUM_DATA_MSB, 2)
        raw = data[0] << 8 | data[1]

        # datasheet, Trimming Coefficient listing in register map with size and sign attributes
        t1, t2, t3, p1, p2, p3, p4, p5, p6, p7, p8, p9, h1, h2, h3, h4, h5, h6 = self._data_calib

        self.get_temperature   # update _t_fine

        v1 = self._t_fine - 76800
        v1 = (((((raw <<14) - (h4 << 20) - (h5 * v1)) + 16384) >> 15) * (((((((v1 * h6) >> 10) * 
             (((v1 * h3) >> 11) + 32768)) >> 10) + 2097152) * h2 + 8192) >> 14))
        v1 = (v1 - (((((v1 >> 15) * (v1 >> 15)) >> 7) * h1) >> 4))
        if v1 < 0:
            v1 = 0
        elif v1 > 419430400:
            v1 = 419430400
        return (v1 >> 12) / 1024.0

    @property
    def get_altitude(self):
        '''!
          @brief Calculate the altitude based on the atmospheric pressure measured by the sensor
          @return Return altitude, unit: m
          @attention If the reference value is provided before, the absolute value of the current 
          @n         position pressure is calculated according to the calibrated sea level atmospheric pressure
        '''
        # see https://www.weather.gov/media/epz/wxcalc/pressureAltitude.pdf
        return 44307.7 * (1 - (self.get_pressure / self.sea_level_pressure) ** 0.190284)

    @property
    def get_data_ready_status(self):
        '''!
          @brief get data ready status
          @return True is data ready
        '''
        temp = self._read_reg(BME280_STATUS, 1)[0]
        if temp & 0b00001000:   # measuring[3]
            return False
        else:
            return True

    def reset(self):
        '''!
          @brief Reset and restart the sensor, restoring the sensor configuration to the default configuration
        '''
        self._write_reg(BME280_CMD_RESET, BME280_CMD_RESET_VALUE)
        time.sleep(0.1)

    def calibrated_absolute_difference(self, altitude):
        '''!
          @brief Take the given current location altitude as the reference value 
          @n     to eliminate the absolute difference for subsequent pressure and altitude data
          @param altitude Altitude in current position
          @return Pass the benchmark value successfully will return ture, if failed it will return false
        '''
        # The altitude in meters based on the currently set sea level pressure.
        ret = False
        if STANDARD_SEA_LEVEL_PRESSURE_PA == self.sea_level_pressure:
            self.sea_level_pressure = (self.get_pressure / pow(1.0 - (altitude / 44307.7), 5.255302))
            ret = True
        return ret

    def set_ctrl_meas_mode(self, mode):
        '''!
          @brief Configure measurement mode and power mode 
          @param mode The measurement mode and power mode that need to be set:
          @n  SLEEP_MODE(Sleep mode): It will be in sleep mode by default after power-on reset. In this mode,no 
          @n                          measurement is performed and power consumption is minimal. All registers 
          @n                          are accessible for reading the chip ID and compensation coefficient.
          @n  FORCED_MODE(Forced mode): In this mode, the sensor will take a single measurement according to the selected 
          @n                            measurement and filtering options. After the measurement is completed, the sensor 
          @n                            will return to sleep mode, and the measurement result can be obtained in the register.
          @n  NORMAL_MODE(Normal mode): Continuously loop between the measurement period and the standby period. 
          @n                            The output data rates are related to the ODR mode setting.
        '''
        mask = 0b00000011   # mode[1:0]
        self._write_reg_bits(BME280_CTRL_MEAS, mask, mode)

    def set_ctrl_meas_sampling_temp(self, osrs_t):
        '''!
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
        mask = 0b11100000   # osrs_t[7:5]
        self._write_reg_bits(BME280_CTRL_MEAS, mask, osrs_t)

    def set_ctrl_meas_sampling_press(self, osrs_p):
        '''!
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
        mask = 0b00011100   # osrs_t[4:2]
        self._write_reg_bits(BME280_CTRL_MEAS, mask, osrs_p)

    def set_ctrl_sampling_humi(self, osrs_h):
        '''!
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
        mask = 0b00000111   # osrs_t[2:0]
        self._write_reg_bits(BME280_CTRL_HUM, mask, osrs_h)

    def set_config_filter(self, iir_config_coef):
        '''!
          @brief IIR filter coefficient setting(IIR filtering)
          @param - iir_config_coef Set IIR filter coefficient, configurable mode:
          @n         BME280_IIR_FILTER_SETTINGS[0], filter off
          @n         BME280_IIR_FILTER_SETTINGS[1], filter coefficient 2
          @n         BME280_IIR_FILTER_SETTINGS[2], filter coefficient 4
          @n         BME280_IIR_FILTER_SETTINGS[3], filter coefficient 8
          @n         BME280_IIR_FILTER_SETTINGS[4], filter coefficient 16
        '''
        mask = 0b00011100   # osrs_t[4:2]
        self._write_reg_bits(BME280_CONFIG, mask, iir_config_coef)

    def set_config_T_standby(self, odr_set):
        '''!
          @brief Set output data rate in subdivision/sub-sampling mode (ODR:output data rates)
          @param odr_set The output data rate needs to be set, configurable mode:
          @n BME280_CONFIG_STANDBY_TIME_0P5, BME280_CONFIG_STANDBY_TIME_62P5, BME280_CONFIG_STANDBY_TIME_125, 
          @n BME280_CONFIG_STANDBY_TIME_250, BME280_CONFIG_STANDBY_TIME_500, BME280_CONFIG_STANDBY_TIME_1000, 
          @n BME280_CONFIG_STANDBY_TIME_10, BME280_CONFIG_STANDBY_TIME_20
          @return  return configuration results
          @retval True indicate configuration succeed
          @retval False indicate configuration failed and remains its original state
        '''
        # The IIR filter coefficient.
        ret = True
        mask = 0b11100000   # osrs_t[7:5]
        self._write_reg_bits(BME280_CONFIG, mask, odr_set)
        if (self._read_reg(BME280_CONFIG, 1)[0] & 0xE0):
            logger.warning("Sensor configuration error detected!")
            ret = False
        return ret

    def _uint8_to_int(self,num):
        '''!
          @brief Convert the incoming uint8 type data to int type
          @param num Incoming uint8 type data
          @return data converted to int type
        '''
        if(num>127):
            num = num - 256
        return num

    def _uint16_to_int(self,num):
        '''!
          @brief Convert the incoming uint16 type data to int type
          @param num Incoming uint16 type data
          @return data converted to int type
        '''
        if(num>32767):
            num = num - 65536
        return num

    def _get_coefficients(self):
        '''!
          @brief Get the calibration data in the NVM register of the sensor
        '''
        calib = self._read_reg(BME280_CALIB_DATA_00_25, 26)
        calib1 = self._read_reg(BME280_CALIB_DATA_26_41, 7)
        self._data_calib = (
            (calib[1] << 8) | calib[0],  # T1
            self._uint16_to_int((calib[3] << 8) | calib[2]),  # T2
            self._uint16_to_int((calib[5] << 8) | (calib[4])),  # T3
            (calib[7] << 8) | calib[6],  # P1
            self._uint16_to_int((calib[9] << 8) | calib[8]),  # P2
            self._uint16_to_int((calib[11] << 8) | calib[10]),  # P3
            self._uint16_to_int((calib[13] << 8) | calib[12]),  # P4
            self._uint16_to_int((calib[15] << 8) | calib[14]),  # P5
            self._uint16_to_int((calib[17] << 8) | calib[16]),  # P6
            self._uint16_to_int((calib[19] << 8) | calib[18]),  # P7
            self._uint16_to_int((calib[21] << 8) | calib[20]),  # P8
            self._uint16_to_int((calib[23] << 8) | calib[22]),  # P9
            calib[25],  # H1
            self._uint16_to_int((calib1[1] << 8) | calib1[0]),  # H2
            calib1[2],  # H3
            self._uint16_to_int((calib1[3] << 4) | (calib1[4] & 0x0F)),  # H4
            self._uint16_to_int((calib1[5] << 4) | ((calib1[4] >> 4) & 0x0F)),  # H5
            self._uint8_to_int(calib1[6]),  # H6
        )

    def _write_reg_bits(self, reg, field, val):
        '''!
          @brief writes data to a register
          @param reg register address
          @param data written data
        '''
        temp = self._read_reg(reg, 1)[0]
        temp &= ~field
        temp |= val
        self._write_reg(reg, temp)

    def _write_reg(self, reg, data):
        '''!
          @brief writes data to a register
          @param reg register address
          @param data written data
        '''
        # Low level register writing, not implemented in base class
        raise NotImplementedError()

    def _read_reg(self, reg, length):
        '''!
          @brief read the data from the register
          @param reg register address
          @param length read data length
          @return read data list
        '''
        # Low level register writing, not implemented in base class
        raise NotImplementedError()


class DFRobot_BME280_I2C(DFRobot_BME280):
    '''!
      @brief define DFRobot_BME280_I2C base class
      @details for using I2C protocol to drive the pressure sensor
    '''

    def __init__(self, i2c_addr=0x77, bus=1):
        '''!
          @brief Module I2C communication init
          @param i2c_addr I2C communication address
          @param bus I2C bus
        '''
        self._addr = i2c_addr
        self._i2c = smbus.SMBus(bus)
        super(DFRobot_BME280_I2C, self).__init__()

    def _write_reg(self, reg, data):
        '''!
          @brief writes data to a register
          @param reg register address
          @param data written data
        '''
        if isinstance(data, int):
            data = [data]
            #logger.info(data)
        self._i2c.write_i2c_block_data(self._addr, reg, data)

    def _read_reg(self, reg, length):
        '''!
          @brief read the data from the register
          @param reg register address
          @param length read data length
          @return read data list
        '''
        return self._i2c.read_i2c_block_data(self._addr, reg, length)


class DFRobot_BME280_SPI(DFRobot_BME280):
    '''!
      @brief define DFRobot_BME280_SPI base class
      @details for using SPI protocol to drive the pressure sensor
    '''

    def __init__(self, cs=8, bus=0, dev=0, speed=500000):
        '''!
          @brief Module SPI communication init
          @param cs cs chip select pin
          @param bus SPI bus
          @param dev SPI device number
          @param speed SPI communication frequency
        '''
        self._cs = cs
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        GPIO.setup(self._cs, GPIO.OUT, initial=1)
        GPIO.output(self._cs, GPIO.LOW)
        self._spi = spidev.SpiDev()
        self._spi.open(bus, dev)
        self._spi.no_cs = True
        self._spi.max_speed_hz = speed
        super(DFRobot_BME280_SPI, self).__init__()

    def _write_reg(self, reg, data):
        '''!
          @brief writes data to a register
          @param reg register address
          @param data written data
        '''
        if isinstance(data, int):
            data = [data]
            #logger.info(data)
        reg_addr = [reg & 0x7f]
        GPIO.output(self._cs, GPIO.LOW)
        self._spi.xfer(reg_addr)
        self._spi.xfer(data)
        GPIO.output(self._cs, GPIO.HIGH)

    def _read_reg(self, reg, length):
        '''!
          @brief read the data from the register
          @param reg register address
          @param length read data length
          @return read data list
        '''
        reg_addr = [reg | 0x80]
        GPIO.output(self._cs, GPIO.LOW)
        #logger.info(reg_addr)
        self._spi.xfer(reg_addr)
        time.sleep(0.01)
        # self._spi.readbytes(1)
        rslt = self._spi.readbytes(length)
        GPIO.output(self._cs, GPIO.HIGH)
        return rslt
