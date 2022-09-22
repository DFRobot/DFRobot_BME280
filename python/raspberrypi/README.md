# DFRobot_BME280
* [中文版](./README_CN.md)

BME280 is an environmental sensor that integrates onboard temperature sensor, humidity sensor and barometer. The sensor has high precision, multiple functions, and a small form factor. It provides both SPI and I2C interfaces, which make it easy to make fast prototypes. It can be widely used in environmental monitoring, story height measurement, Internet of Things (IoT) control and other various enviroment related ideas! Tge Gravity I2C BME280 Environmental Sensor has based on BoSCH's newest MEMS sensor (Micro-Electro-Mechanical System). It is very stable when compared with similar sensors. The sensor is especially adept in air pressure measurement; it has an offset temperature coefficient of ±1.5 Pa/K, equiv. to ±12.6 cm at 1 °C temperature change. Therefore, the stable and multi-function form of the BME280 can be a perfect fit in many scenarios.

![产品实物图](../../resources/images/BME280.png)


## Product Link (https://www.dfrobot.com/product-1606.html)
    SKU: SEN0236


## Table of Contents

* [Summary](#summary)
* [Installation](#installation)
* [Methods](#methods)
* [Compatibility](#compatibility)
* [History](#history)
* [Credits](#credits)


## Summary

- Compatible with 3.3V/5V microcontrollers
- Environmental monitoring: temperature, humidity and barometer
- Gravity I2C interface and reserve XH2.54 SPI interface
- Small size, convenient to install


## Installation

To use the library, first download the library file, paste it into the directory you specified, then open the Examples folder and run the demo in that folder.


## Methods

```python

    '''!
      @brief Initialize sensor
      @return return initialization status
      @retval True indicate initialization succeed
      @retval False indicate initialization failed
    '''
    def begin(self):

    '''!
      @brief Get pressure measurement value from register, working range (-40 ‒ +85 °C)
      @return Return temperature measurements, unit: °C
    '''
    @property
    def get_temperature(self):

    '''!
      @brief Get pressure measurement value from register, working range (300‒1100 hPa)
      @return Return pressure measurements, unit: Pa
      @attention If the reference value is provided before, the absolute value of the current 
      @n         position pressure is calculated according to the calibrated sea level atmospheric pressure
        '''
    @property
    def get_pressure(self):

    '''!
      @brief Get humidity measurement value from register, working range (0 ~ 100 %RH)
      @return Return humidity measurements, unit: %RH
    '''
    @property
    def get_humidity(self):

    '''!
      @brief Calculate the altitude based on the atmospheric pressure measured by the sensor
      @return Return altitude, unit: m
      @attention If the reference value is provided before, the absolute value of the current 
      @n         position pressure is calculated according to the calibrated sea level atmospheric pressure
    '''
    @property
    def get_altitude(self):

    '''!
      @brief get data ready status
      @return True is data ready
    '''
    @property
    def get_data_ready_status(self):

    '''!
      @brief Take the given current location altitude as the reference value 
      @n     to eliminate the absolute difference for subsequent pressure and altitude data
      @param altitude Altitude in current position
      @return Pass the benchmark value successfully will return ture, if failed it will return false
    '''
    def calibrated_absolute_difference(self, altitude):

    '''!
      @brief Reset and restart the sensor, restoring the sensor configuration to the default configuration
    '''
    def reset(self):

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
    def set_ctrl_meas_mode(self, mode):

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
    def set_ctrl_meas_sampling_temp(self, osrs_t):

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
    def set_ctrl_meas_sampling_press(self, osrs_p):

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
    def set_ctrl_sampling_humi(self, osrs_h):

    '''!
      @brief IIR filter coefficient setting(IIR filtering)
      @param - iir_config_coef Set IIR filter coefficient, configurable mode:
      @n         BME280_IIR_FILTER_SETTINGS[0], filter off
      @n         BME280_IIR_FILTER_SETTINGS[1], filter coefficient 2
      @n         BME280_IIR_FILTER_SETTINGS[2], filter coefficient 4
      @n         BME280_IIR_FILTER_SETTINGS[3], filter coefficient 8
      @n         BME280_IIR_FILTER_SETTINGS[4], filter coefficient 16
    '''
    def set_config_filter(self, iir_config_coef):

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
    def set_config_T_standby(self, odr_set)

```


## Compatibility

* RaspberryPi Version

| Board        | Work Well | Work Wrong | Untested | Remarks |
| ------------ | :-------: | :--------: | :------: | ------- |
| RaspberryPi2 |           |            |    √     |         |
| RaspberryPi3 |           |            |    √     |         |
| RaspberryPi4 |     √     |            |          |         |

* Python Version

| Python  | Work Well | Work Wrong | Untested | Remarks |
| ------- | :-------: | :--------: | :------: | ------- |
| Python2 |     √     |            |          |         |
| Python3 |     √     |            |          |         |


## History

- 2022/06/16 - Version 1.0.0 released.
- 2022/09/22 - Version 1.0.1 released.

## Credits

Written by qsjhyy(yihuan.huang@dfrobot.com), 2022. (Welcome to our [website](https://www.dfrobot.com/))

