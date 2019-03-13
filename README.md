# BMP280
DFRobot's Temperature, Pressure and Approx altitude

## DFRobot_BMP280 Library for Arduino
---------------------------------------------------------
Provides an Arduino library for reading and interpreting Bosch BMP280 data over I2C. <br>
Used to read current temperature, air pressure and calculate altitude.

## Table of Contents

* [Installation](#installation)
* [Methods](#methods)
* [Compatibility](#compatibility)
* [History](#history)
* [Credits](#credits)

<snippet>
<content>

## Installation

To use this library download the zip file, uncompress it to a folder named DFRobot_BMP280. 
Download the zip file first to use this library and uncompress it to a folder named DFRobot_BMP280. 

## Methods

```C++

class DFRobot_BMP280 {
// defines
public:
  /**
   * @brief Enum global status
   */
  typedef enum {
    eStatusOK,
    eStatusErr,
    eStatusErrDeviceNotDetected,
    eStatusErrParameter
  } eStatus_t;

  /**
   * @brief Enum control measurement mode (power)
   */
  typedef enum {
    eCtrlMeasMode_sleep,
    eCtrlMeasMode_forced,
    eCtrlMeasMode_normal = 0x03
  } eCtrlMeasMode_t;

  /**
   * @brief Enum sampling
   */
  typedef enum {
    eSampling_no,
    eSampling_X1,
    eSampling_X2,
    eSampling_X4,
    eSampling_X8,
    eSampling_X16
  } eSampling_t;

  /**
   * @brief Enum config filter
   */
  typedef enum {
    eConfigFilter_off,
    eConfigFilter_X2,
    eConfigFilter_X4,
    eConfigFilter_X8,
    eConfigFilter_X16
  } eConfigFilter_t;

  /**
   * @brief Enum config standby time, unit ms
   */
  typedef enum {
    eConfigTStandby_0_5,    // 0.5 ms
    eConfigTStandby_62_5,
    eConfigTStandby_125,
    eConfigTStandby_250,
    eConfigTStandby_500,
    eConfigTStandby_1000,
    eConfigTStandby_2000,
    eConfigTStandby_4000
  } eConfigTStandby_t;

// functions
public:
  DFRobot_BMP280();

  /**
   * @brief begin Sensor begin
   * @return Enum of eStatus_t
   */
  eStatus_t   begin();

  /**
   * @brief getTemperature Get temperature
   * @return Temprature in Celsius
   */
  float       getTemperature();

  /**
   * @brief getPressure Get pressure
   * @return Pressure in pa
   */
  uint32_t    getPressure();

  /**
   * @brief calAltitude Calculate altitude
   * @param seaLevelPressure Sea level pressure
   * @param pressure Pressure in pa
   * @return Altitude in meter
   */
  float       calAltitude(float seaLevelPressure, uint32_t pressure);

  /**
   * @brief reset Reset sensor
   */
  void    reset();

  /**
   * @brief setCtrlMeasMode Set control measure mode
   * @param eMode One enum of eCtrlMeasMode_t
   */
  void    setCtrlMeasMode(eCtrlMeasMode_t eMode);

  /**
   * @brief setCtrlMeasSamplingTemp Set control measure temperature oversampling
   * @param eSampling One enum of eSampling_t
   */
  void    setCtrlMeasSamplingTemp(eSampling_t eSampling);

  /**
   * @brief setCtrlMeasSamplingPress Set control measure pressure oversampling
   * @param eSampling One enum of eSampling_t
   */
  void    setCtrlMeasSamplingPress(eSampling_t eSampling);

  /**
   * @brief setConfigFilter Set config filter
   * @param eFilter One enum of eConfigFilter_t
   */
  void    setConfigFilter(eConfigFilter_t eFilter);

  /**
   * @brief setConfigTStandby Set config standby time
   * @param eT One enum of eConfigTStandby_t
   */
  void    setConfigTStandby(eConfigTStandby_t eT);

public:
  /**
   * @brief lastOperateStatus Last operate status
   */
  eStatus_t   lastOperateStatus;
  
};

class DFRobot_BMP280_IIC : public DFRobot_BMP280 {
public:
  /**
   * @brief Enum pin sdo states
   */
  typedef enum {
    eSdo_low,
    eSdo_high
  } eSdo_t;

  /**
   * @brief DFRobot_BMP280_IIC
   * @param pWire Which TwoWire peripheral to operate
   * @param eSdo Pin sdo status
   */
  DFRobot_BMP280_IIC(TwoWire *pWire, eSdo_t eSdo);
};

```

## Compatibility

MCU                | Work Well | Work Wrong | Untested  | Remarks
------------------ | :----------: | :----------: | :---------: | -----
FireBeetle-ESP32  |      √       |             |            | 
FireBeetle-ESP8266  |      √       |             |            | 
Arduino uno |       √      |             |            | 

## History

- Nov 31, 2018 - Version 0.1 released.
- March 12, 2019 - Version 0.1 remake.

## Credits

Written by Frank(jiehan.guo@dfrobot.com), 2018. (Welcome to our [website](https://www.dfrobot.com/))
