/*!
 * @file  DFRobot_BME280.h
 * @brief  Define infrastructure of DFRobot_BME280 class
 * @details  It provides both SPI and I2C interfaces, which make it easy to make fast prototypes.
 * @n  The sensor is especially adept in air pressure measurement; it has an offset 
 * @n  temperature coefficient of ±1.5 Pa/K, equiv. to ±12.6 cm at 1 °C temperature change.
 * @copyright  Copyright (c) 2010 DFRobot Co.Ltd (http://www.dfrobot.com)
 * @license  The MIT License (MIT)
 * @author  [Frank](jiehan.guo@dfrobot.com)
 * @maintainer  [qsjhyy](yihuan.huang@dfrobot.com)
 * @version  V1.0
 * @date  2022-06-16
 * @url  https://github.com/DFRobot/DFRobot_BME280
 */
#ifndef DFROBOT_BME280_H
#define DFROBOT_BME280_H

#include "Arduino.h"
#include "Wire.h"
#include "SPI.h"

#ifndef PROGMEM
  #define PROGMEM
#endif

#define BME280_REG_START    0x88

class DFRobot_BME280
{

public:

  /**
   * @enum eStatus_t
   * @brief Enum global status
   */
  typedef enum {
    eStatusOK,
    eStatusErr,
    eStatusErrDeviceNotDetected,
    eStatusErrParameter
  } eStatus_t;

  /**
   * @struct sCalibrateDig_t
   * @brief Temperature and pressure compensation calibration coefficient
   */
  typedef struct {
    uint16_t    t1;
    int16_t     t2, t3;
    uint16_t    p1;
    int16_t     p2, p3, p4, p5, p6, p7, p8, p9;
    uint16_t    reserved0;
  } sCalibrateDig_t;

  /**
   * @struct sCalibrateDigHumi_t
   * @brief Humidity compensation calibration coefficient
   */
  typedef struct {
    uint8_t   h1;
    int16_t   h2;
    uint8_t   h3;
    int16_t   h4;
    int16_t   h5;
    int8_t    h6;
  } sCalibrateDigHumi_t;

  /**
   * @struct sRegCtrlHum_t
   * @brief Control register for humidity measurement parameters
   */
  typedef struct {
    uint8_t   osrs_h: 3;
  } sRegCtrlHum_t;

  /**
   * @struct sRegStatus_t
   * @brief Status Register
   */
  typedef struct {
    uint8_t   im_update: 1;
    uint8_t   reserved: 2;
    uint8_t   measuring: 1;
  } sRegStatus_t;

  /**
   * @enum eCtrlMeasMode_t
   * @brief Enum control measurement mode (power)
   */
  typedef enum {
    eCtrlMeasMode_sleep,
    eCtrlMeasMode_forced,
    eCtrlMeasMode_normal = 0x03
  } eCtrlMeasMode_t;

  /**
   * @enum eSampling_t
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
   * @struct sRegCtrlMeas_t
   * @brief Temperature and humidity measurement control register
   */
  typedef struct {
    uint8_t   mode: 2;
    uint8_t   osrs_p: 3;
    uint8_t   osrs_t: 3;
  } sRegCtrlMeas_t;

  /**
   * @enum eConfigSpi3w_en_t
   */
  typedef enum {
    eConfigSpi3w_en_disable,
    eConfigSpi3w_en_enable
  } eConfigSpi3w_en_t;

  /**
   * @enum eConfigFilter_t
   * @brief Enum config filter
   */
  typedef enum {
    eConfigFilter_off,
    eConfigFilter_X2,
    eConfigFilter_X4,
    eConfigFilter_X8,
    eConfigFilter_X16
  } eConfigFilter_t;    // unknow config, can't underestand datasheet, datasheet error like

  /**
   * @enum eConfigTStandby_t
   * @brief Enum config standby time, unit ms
   */
  typedef enum {
    eConfigTStandby_0_5,    // 0.5 ms
    eConfigTStandby_62_5,
    eConfigTStandby_125,
    eConfigTStandby_250,
    eConfigTStandby_500,
    eConfigTStandby_1000,
    eConfigTStandby_10,
    eConfigTStandby_20
  } eConfigTStandby_t;

  /**
   * @struct sRegConfig_t
   * @brief control register
   */
  typedef struct {
    uint8_t   spi3w_en: 1;
    uint8_t   reserved1: 1;
    uint8_t   filter: 3;
    uint8_t   t_sb: 3;
  } sRegConfig_t;

  /**
   * @struct sRegPress_t
   * @brief Pressure measurement data
   */
  typedef struct {
    uint8_t   msb, lsb;
    uint8_t   reserved: 4;
    uint8_t   xlsb: 4;
  } sRegPress_t;

  /**
   * @struct sRegTemp_t
   * @brief Temperature measurement data
   */
  typedef struct {
    uint8_t   msb, lsb;
    uint8_t   reserved: 4;
    uint8_t   xlsb: 4;
  } sRegTemp_t;

  /**
   * @struct sRegHumi_t
   * @brief Humidity measurement data
   */
  typedef struct {
    uint8_t   msb, lsb;
  } sRegHumi_t;

  /**
   * @struct sRegs_t
   * @brief Module register structure
   */
  typedef struct {
    sCalibrateDig_t   calib;
    uint8_t   reserved0[(0xd0 - 0xa1 - 1)];
    uint8_t   chip_id;
    #define BME280_REG_CHIP_ID_DEFAULT    0x60
    uint8_t   reserved1[(0xe0 - 0xd0 - 1)];
    uint8_t   reset;
    uint8_t   reserved2[(0xf2 - 0xe0 - 1)];
    sRegCtrlHum_t   ctrl_hum;
    sRegStatus_t    status;
    sRegCtrlMeas_t    ctrl_meas;
    sRegConfig_t      config;
    uint8_t   reserved3;
    sRegPress_t   press;
    sRegTemp_t    temp;
    sRegHumi_t    humi;
  } __attribute__ ((packed)) sRegs_t;

public:
  /**
   * @fn DFRobot_BME280
   * @brief the DFRobot_BME280 class' constructor.
   * @return None
   */
  DFRobot_BME280();

  /**
   * @fn begin
   * @brief begin Sensor begin
   * @return Enum of eStatus_t
   */
  virtual eStatus_t   begin();

  /**
   * @fn getTemperature
   * @brief getTemperature Get temperature
   * @return Temprature in Celsius
   */
  float       getTemperature();

  /**
   * @fn getPressure
   * @brief getPressure Get pressure
   * @return Pressure in pa
   */
  uint32_t    getPressure();

  /**
   * @fn getHumidity
   * @brief getHumidity Get humidity
   * @return Humidity in percent
   */
  float       getHumidity();

  /**
   * @fn calAltitude
   * @brief calAltitude Calculate altitude
   * @param seaLevelPressure Sea level pressure
   * @param pressure Pressure in pa
   * @return Altitude in meter
   */
  float       calAltitude(float seaLevelPressure, uint32_t pressure);

  /**
   * @fn reset
   * @brief reset Reset sensor
   */
  void    reset();

  /**
   * @fn setCtrlMeasMode
   * @brief setCtrlMeasMode Set control measure mode
   * @param eMode One enum of eCtrlMeasMode_t
   */
  void    setCtrlMeasMode(eCtrlMeasMode_t eMode);

  /**
   * @fn setCtrlMeasSamplingTemp
   * @brief setCtrlMeasSamplingTemp Set control measure temperature oversampling
   * @param eSampling One enum of eSampling_t
   */
  void    setCtrlMeasSamplingTemp(eSampling_t eSampling);

  /**
   * @fn setCtrlMeasSamplingPress
   * @brief setCtrlMeasSamplingPress Set control measure pressure oversampling
   * @param eSampling One enum of eSampling_t
   */
  void    setCtrlMeasSamplingPress(eSampling_t eSampling);

  /**
   * @fn setCtrlHumiSampling
   * @brief setCtrlHumiSampling Set control measure humidity oversampling
   * @param eSampling One enum of eSampling_t
   */
  void    setCtrlHumiSampling(eSampling_t eSampling);

  /**
   * @fn setConfigFilter
   * @brief setConfigFilter Set config filter
   * @param eFilter One enum of eConfigFilter_t
   */
  void    setConfigFilter(eConfigFilter_t eFilter);

  /**
   * @fn setConfigTStandby
   * @brief setConfigTStandby Set config standby time
   * @param eT One enum of eConfigTStandby_t
   */
  void    setConfigTStandby(eConfigTStandby_t eT);

protected:
  void    getCalibrate();
  int32_t   getTemperatureRaw();
  int32_t   getPressureRaw();
  int32_t   getHumidityRaw();
  uint8_t   getReg(uint8_t reg);
  void      writeRegBits(uint8_t reg, uint8_t field, uint8_t val);
  virtual void    writeReg(uint8_t reg, uint8_t *pBuf, uint16_t len) = 0;
  virtual void    readReg(uint8_t reg, uint8_t *pBuf, uint16_t len) = 0;

public:
  eStatus_t   lastOperateStatus;

protected:
  int32_t   _t_fine;
  sCalibrateDig_t   _sCalib;
  sCalibrateDigHumi_t   _sCalibHumi;
};

class DFRobot_BME280_IIC : public DFRobot_BME280
{
public:
  /**
   * @fn DFRobot_BME280_IIC
   * @brief DFRobot_BME280_IIC
   * @param pWire Which TwoWire peripheral to operate
   * @param addr Sensor addr
   */
  DFRobot_BME280_IIC(TwoWire *pWire, uint8_t addr);

  /**
   * @fn begin
   * @brief begin Sensor begin
   * @return Enum of eStatus_t
   */
  virtual eStatus_t   begin();

protected:
  void    writeReg(uint8_t reg, uint8_t *pBuf, uint16_t len);
  void    readReg(uint8_t reg, uint8_t *pBuf, uint16_t len);

protected:
  TwoWire   *_pWire;
  uint8_t   _addr;
};

class DFRobot_BME280_SPI : public DFRobot_BME280
{
public:
  /**
   * @fn DFRobot_BME280_SPI
   * @brief DFRobot_BME280_SPI
   * @param pSpi Which SPIClass peripheral to oprate
   * @param pin Sensor cs pin id
   */
  DFRobot_BME280_SPI(SPIClass *pSpi, uint16_t pin);

  /**
   * @fn begin
   * @brief begin Sensor begin
   * @return Enum of eStatus_t
   */
  virtual eStatus_t   begin();

protected:
  void    writeReg(uint8_t reg, uint8_t *pBuf, uint16_t len);
  void    readReg(uint8_t reg, uint8_t *pBuf, uint16_t len);

protected:
  SPIClass    *_pSpi;
  uint8_t     _pinCs;
};

#endif
