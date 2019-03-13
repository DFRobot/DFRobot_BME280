/*
 MIT License

 Copyright (C) <2019> <@DFRobot Frank>

　Permission is hereby granted, free of charge, to any person obtaining a copy of this
　software and associated documentation files (the "Software"), to deal in the Software
　without restriction, including without limitation the rights to use, copy, modify,
　merge, publish, distribute, sublicense, and/or sell copies of the Software, and to
　permit persons to whom the Software is furnished to do so.

　The above copyright notice and this permission notice shall be included in all copies or
　substantial portions of the Software.
 THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
 INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR
 PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE
 FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
 ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#ifndef DFROBOT_BME280_H
#define DFROBOT_BME280_H

#include "Arduino.h"
#include "Wire.h"

#ifndef PROGMEM
# define PROGMEM
#endif

class DFRobot_BME280 {
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

  typedef struct {
    uint16_t    t1;
    int16_t     t2, t3;
    uint16_t    p1;
    int16_t     p2, p3, p4, p5, p6, p7, p8, p9;
    uint16_t    reserved0;
  } sCalibrateDig_t;

  typedef struct {
    uint8_t   h1;
    int16_t   h2;
    uint8_t   h3;
    int16_t   h4;
    int16_t   h5;
    int8_t    h6;
  } sCalibrateDigHumi_t;

  typedef struct {
    uint8_t   osrs_h: 3;
  } sRegCtrlHum_t;

  typedef struct {
    uint8_t   im_update: 1;
    uint8_t   reserved: 2;
    uint8_t   measuring: 1;
  } sRegStatus_t;

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

  typedef struct {
    uint8_t   mode: 2;
    uint8_t   osrs_p: 3;
    uint8_t   osrs_t: 3;
  } sRegCtrlMeas_t;

  typedef enum {
    eConfigSpi3w_en_disable,
    eConfigSpi3w_en_enable
  } eConfigSpi3w_en_t;

  /**
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

  typedef struct {
    uint8_t   spi3w_en: 1;
    uint8_t   reserved1: 1;
    uint8_t   filter: 3;
    uint8_t   t_sb: 3;
  } sRegConfig_t;

  typedef struct {
    uint8_t   msb, lsb;
    uint8_t   reserved: 4;
    uint8_t   xlsb: 4;
  } sRegPress_t;

  typedef struct {
    uint8_t   msb, lsb;
    uint8_t   reserved: 4;
    uint8_t   xlsb: 4;
  } sRegTemp_t;

  #define BME280_REG_START    0x88
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
    uint16_t    humi;
  } sRegs_t;

// functions
public:
  DFRobot_BME280();

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
   * @brief getHumidity Get humidity
   * @return Humidity in percent
   */
  float       getHumidity();

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
   * @brief setCtrlHumiSampling Set control measure humidity oversampling
   * @param eSampling One enum of eSampling_t
   */
  void    setCtrlHumiSampling(eSampling_t eSampling);

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

protected:
  void    getCalibrate();

  int32_t   getTemperatureRaw();
  int32_t   getPressureRaw();
  int32_t   getHumidityRaw();

  uint8_t   getReg(uint8_t reg);
  void      writeRegBits(uint8_t reg, uint8_t flied, uint8_t val);

  virtual void    writeReg(uint8_t reg, uint8_t *pBuf, uint16_t len) = 0;
  virtual void    readReg(uint8_t reg, uint8_t *pBuf, uint16_t len) = 0;

// variables
public:
  eStatus_t   lastOperateStatus;

protected:
  int32_t   _t_fine;

  sCalibrateDig_t   _sCalib;
  sCalibrateDigHumi_t   _sCalibHumi;
};

class DFRobot_BME280_IIC : public DFRobot_BME280 {
public:
  /**
   * @brief Enum pin sdo states
   */
  typedef enum {
    eSdo_low,
    eSdo_high
  } eSdo_t;

  /**
   * @brief DFRobot_BME280_IIC
   * @param pWire Which TwoWire peripheral to operate
   * @param eSdo Pin sdo status
   */
  DFRobot_BME280_IIC(TwoWire *pWire, eSdo_t eSdo);

protected:
  void    writeReg(uint8_t reg, uint8_t *pBuf, uint16_t len);
  void    readReg(uint8_t reg, uint8_t *pBuf, uint16_t len);

protected:
  TwoWire   *_pWire;

  uint8_t   _addr;

};

#endif
