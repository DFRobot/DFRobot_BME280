/*!
 * @file  DFRobot_BME280.cpp
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
#include "DFRobot_BME280.h"

#define __DBG   0
#if __DBG
# define __DBG_CODE(x)   Serial.print("__DBG_CODE: "); Serial.print(__FUNCTION__); Serial.print(" "); Serial.print(__LINE__); Serial.print(" "); x; Serial.println()
#else
# define __DBG_CODE(x)
#endif

const DFRobot_BME280::sRegs_t PROGMEM   _sRegs = DFRobot_BME280::sRegs_t();
#ifdef __AVR__
typedef uint16_t    platformBitWidth_t;
#else
typedef uint32_t    platformBitWidth_t;
#endif

const platformBitWidth_t    _regsAddr = (platformBitWidth_t) &_sRegs;

#define writeRegBitsHelper(reg, flied, val)   writeRegBits(regOffset(&(reg)), *(uint8_t*) &(flied), *(uint8_t*) &(val))

DFRobot_BME280::DFRobot_BME280() {}

uint8_t regOffset(const void *pReg)
{
  return ((platformBitWidth_t) pReg - _regsAddr + BME280_REG_START);
}

DFRobot_BME280::eStatus_t DFRobot_BME280::begin()
{
  __DBG_CODE(Serial.print("temp register addr: "); Serial.print(regOffset(&_sRegs.temp), HEX));
  __DBG_CODE(Serial.print("first register addr: "); Serial.print(regOffset(&_sRegs.calib), HEX));
  __DBG_CODE(Serial.print("status register addr: "); Serial.print(regOffset(&_sRegs.status), HEX));
  __DBG_CODE(Serial.print("id register addr: "); Serial.print(regOffset(&_sRegs.chip_id), HEX));

  uint8_t temp = getReg(regOffset(&_sRegs.chip_id));
  if((temp == BME280_REG_CHIP_ID_DEFAULT) && (lastOperateStatus == eStatusOK)) {
    reset();
    delay(300);
    getCalibrate();
    setCtrlMeasSamplingPress(eSampling_X8);
    setCtrlMeasSamplingTemp(eSampling_X8);
    setCtrlHumiSampling(eSampling_X8);
    setConfigFilter(eConfigFilter_off);
    setConfigTStandby(eConfigTStandby_125);
    setCtrlMeasMode(eCtrlMeasMode_normal);   // set control measurement mode to make these settings effective
  } else
    lastOperateStatus = eStatusErrDeviceNotDetected;
  return lastOperateStatus;
}

float DFRobot_BME280::getTemperature()
{
  int32_t   raw = getTemperatureRaw();
  float     rslt = 0;
  int32_t   v1, v2;
  if(lastOperateStatus == eStatusOK) {
    v1 = ((((raw >> 3) - ((int32_t) _sCalib.t1 << 1))) * ((int32_t) _sCalib.t2)) >> 11;
    v2 = (((((raw >> 4) - ((int32_t) _sCalib.t1)) * ((raw >> 4) - ((int32_t) _sCalib.t1))) >> 12) * ((int32_t) _sCalib.t3)) >> 14;
    _t_fine = v1 + v2;
    rslt = (_t_fine * 5 + 128) >> 8;
    return (rslt / 100);
  }
  return 0;
}

uint32_t DFRobot_BME280::getPressure()
{
  getTemperature();   // update _t_fine
  int32_t   raw = getPressureRaw();
  int64_t   rslt = 0;
  int64_t   v1, v2;
  if(lastOperateStatus == eStatusOK) {
    v1 = ((int64_t) _t_fine) - 128000;
    v2 = v1 * v1 * (int64_t) _sCalib.p6;
    v2 = v2 + ((v1 * (int64_t) _sCalib.p5) << 17);
    v2 = v2 + (((int64_t) _sCalib.p4) << 35);
    v1 = ((v1 * v1 * (int64_t) _sCalib.p3) >> 8) + ((v1 * (int64_t) _sCalib.p2) << 12);
    v1 = (((((int64_t) 1) << 47) + v1)) * ((int64_t) _sCalib.p1) >> 33;
    if(v1 == 0)
      return 0;
    rslt = 1048576 - raw;
    rslt = (((rslt << 31) - v2) * 3125) / v1;
    v1 = (((int64_t) _sCalib.p9) * (rslt >> 13) * (rslt >> 13)) >> 25;
    v2 = (((int64_t) _sCalib.p8) * rslt) >> 19;
    rslt = ((rslt + v1 + v2) >> 8) + (((int64_t) _sCalib.p7) << 4);
    return (uint32_t) (rslt / 256);
  }
  return 0;
}

float DFRobot_BME280::getHumidity()
{
  getTemperature();   // update _t_fine
  int32_t   raw = getHumidityRaw();
  int32_t   v1;
  __DBG_CODE(Serial.print("raw: "); Serial.print(raw));
  if(lastOperateStatus == eStatusOK) {
    v1 = (_t_fine - ((int32_t) 76800));
    v1 = (((((raw <<14) - (((int32_t) _sCalibHumi.h4) << 20) - (((int32_t) _sCalibHumi.h5) * v1)) +
         ((int32_t) 16384)) >> 15) * (((((((v1 * ((int32_t) _sCalibHumi.h6)) >> 10) * (((v1 *
         ((int32_t) _sCalibHumi.h3)) >> 11) + ((int32_t) 32768))) >> 10) + ((int32_t) 2097152)) *
         ((int32_t) _sCalibHumi.h2) + 8192) >> 14));
    v1 = (v1 - (((((v1 >> 15) * (v1 >> 15)) >> 7) * ((int32_t) _sCalibHumi.h1)) >> 4));
    v1 = (v1 < 0 ? 0 : v1);
    v1 = (v1 > 419430400 ? 419430400 : v1);
    return ((float) (v1 >> 12)) / 1024.0f;
  }
  return 0;
}

float DFRobot_BME280::calAltitude(float seaLevelPressure, uint32_t pressure)
{
  return 44330 * (1.0f - pow(pressure / 100 / seaLevelPressure, 0.1903));
}

void DFRobot_BME280::reset()
{
  uint8_t   temp = 0xb6;
  writeReg(regOffset(&_sRegs.reset), (uint8_t*) &temp, sizeof(temp));
  delay(100);
}

void DFRobot_BME280::setCtrlMeasMode(eCtrlMeasMode_t eMode)
{
  sRegCtrlMeas_t    sRegFlied = {0}, sRegVal = {0};
  // sRegFlied.mode = 0xff;
  sRegVal.mode = eMode;
  writeRegBitsHelper(_sRegs.ctrl_meas, sRegFlied, sRegVal);
}

void DFRobot_BME280::setCtrlMeasSamplingTemp(eSampling_t eSampling)
{
  sRegCtrlMeas_t    sRegFlied = {0}, sRegVal = {0};
  // sRegFlied.osrs_t = 0xff;
  sRegVal.osrs_t = eSampling;
  writeRegBitsHelper(_sRegs.ctrl_meas, sRegFlied, sRegVal);
}

void DFRobot_BME280::setCtrlMeasSamplingPress(eSampling_t eSampling)
{
  sRegCtrlMeas_t    sRegFlied = {0}, sRegVal = {0};
  // sRegFlied.osrs_p = 0xff;
  sRegVal.osrs_p = eSampling;
  writeRegBitsHelper(_sRegs.ctrl_meas, sRegFlied, sRegVal);
}

void DFRobot_BME280::setCtrlHumiSampling(eSampling_t eSampling)
{
  sRegCtrlHum_t   sRegFlied = {0}, sRegVal = {0};
  // sRegFlied.osrs_h = 0xff;
  sRegVal.osrs_h = eSampling;
  writeRegBitsHelper(_sRegs.ctrl_hum, sRegFlied, sRegVal);
}

void DFRobot_BME280::setConfigFilter(eConfigFilter_t eFilter)
{
  sRegConfig_t    sRegFlied = {0}, sRegVal = {0};
  // sRegFlied.filter = 0xff;
  sRegVal.filter = eFilter;
  writeRegBitsHelper(_sRegs.config, sRegFlied, sRegVal);
}

void DFRobot_BME280::setConfigTStandby(eConfigTStandby_t eT)
{
  sRegConfig_t    sRegFlied = {0}, sRegVal = {0};
  // sRegFlied.t_sb = 0xff;
  sRegVal.t_sb = eT;
  writeRegBitsHelper(_sRegs.config, sRegFlied, sRegVal);
}

void DFRobot_BME280::getCalibrate()
{
  readReg(regOffset(&_sRegs.calib), (uint8_t*) &_sCalib, sizeof(_sCalib));

  readReg(regOffset(&_sRegs.calib) + sizeof(_sCalib) - 1, (uint8_t*) &_sCalibHumi.h1, sizeof(_sCalibHumi.h1));
  readReg(0xe1, (uint8_t*) &_sCalibHumi.h2, sizeof(_sCalibHumi.h2));    // fxxk discontinuous address
  readReg(0xe3, (uint8_t*) &_sCalibHumi.h3, sizeof(_sCalibHumi.h3));
  readReg(0xe4, (uint8_t*) &_sCalibHumi.h4, sizeof(_sCalibHumi.h4));
  readReg(0xe5, (uint8_t*) &_sCalibHumi.h5, sizeof(_sCalibHumi.h5));
  readReg(0xe7, (uint8_t*) &_sCalibHumi.h6, sizeof(_sCalibHumi.h6));

  // 0xe4 / 0xe5 [3: 0] = dig_h4 [11: 4] / [3: 0]
  _sCalibHumi.h4 = ((_sCalibHumi.h4 >> 8) & 0x0f) | ((_sCalibHumi.h4 & 0x00ff) << 4);
  // 0xe5 [7: 4] / 0xe6 = dig_h5 [3: 0] / [11: 4]
  _sCalibHumi.h5 = ((_sCalibHumi.h5 & 0xff00) >> 4) | ((_sCalibHumi.h5 & 0x00f0) >> 4);   // fxxk fxxk fxxk very strange arrangement
}

int32_t DFRobot_BME280::getTemperatureRaw()
{
  sRegTemp_t    sReg;
  readReg(regOffset(&_sRegs.temp), (uint8_t*) &sReg, sizeof(sReg));
  return (((uint32_t) sReg.msb << 12) | ((uint32_t) sReg.lsb << 4) | ((uint32_t) sReg.xlsb));
}

int32_t DFRobot_BME280::getPressureRaw()
{
  sRegPress_t   sReg;
  readReg(regOffset(&_sRegs.press), (uint8_t*) &sReg, sizeof(sReg));
  return (((uint32_t) sReg.msb << 12) | ((uint32_t) sReg.lsb << 4) | ((uint32_t) sReg.xlsb));
}

int32_t DFRobot_BME280::getHumidityRaw()
{
  sRegHumi_t   sReg;
  readReg(regOffset(&_sRegs.humi), (uint8_t*) &sReg, sizeof(sReg));
  return (((int32_t) sReg.msb << 8) | (int32_t) sReg.lsb);
}

uint8_t DFRobot_BME280::getReg(uint8_t reg)
{
  uint8_t   temp;
  readReg(reg, (uint8_t*) &temp, sizeof(temp));
  return temp;
}

void DFRobot_BME280::writeRegBits(uint8_t reg, uint8_t field, uint8_t val)
{
  uint8_t   temp;
  readReg(reg, (uint8_t*) &temp, sizeof(temp));
  temp &= ~field;
  temp |= val;
  writeReg(reg, (uint8_t*) &temp, sizeof(temp));
}

DFRobot_BME280_IIC::DFRobot_BME280_IIC(TwoWire *pWire, uint8_t addr)
{
  _pWire = pWire;
  _addr = addr;
}

DFRobot_BME280::eStatus_t DFRobot_BME280_IIC::begin(void)
{
  _pWire->begin();   // Wire.h(I2C)library function initialize wire library
  return DFRobot_BME280::begin();   // Use the initialization function of the parent class
}

void DFRobot_BME280_IIC::readReg(uint8_t reg, uint8_t *pBuf, uint16_t len)
{
  lastOperateStatus = eStatusErrDeviceNotDetected;
  // _pWire->begin();
  _pWire->beginTransmission(_addr);
  _pWire->write(reg);
  if(_pWire->endTransmission() != 0)
    return;

  _pWire->requestFrom(_addr, (uint8_t)len);
  for(uint16_t i = 0; i < len; i ++)
    pBuf[i] = _pWire->read();
  lastOperateStatus = eStatusOK;
}

void DFRobot_BME280_IIC::writeReg(uint8_t reg, uint8_t *pBuf, uint16_t len)
{
  lastOperateStatus = eStatusErrDeviceNotDetected;
  // _pWire->begin();
  _pWire->beginTransmission(_addr);
  _pWire->write(reg);
  for(uint16_t i = 0; i < len; i ++)
    _pWire->write(pBuf[i]);
  if(_pWire->endTransmission() != 0)
    return;
  lastOperateStatus = eStatusOK;
}

DFRobot_BME280_SPI::DFRobot_BME280_SPI(SPIClass *pSpi, uint16_t pin)
{
  _pSpi = pSpi;
  _pinCs = pin;
  lastOperateStatus = eStatusOK;
}

DFRobot_BME280::eStatus_t DFRobot_BME280_SPI::begin(void)
{
  pinMode(_pinCs, OUTPUT);
  digitalWrite(_pinCs, HIGH);
  _pSpi->begin();
  return DFRobot_BME280::begin();
}

void DFRobot_BME280_SPI::readReg(uint8_t reg, uint8_t *pBuf, uint16_t len)
{
  _pSpi->beginTransaction(SPISettings(500000, MSBFIRST, SPI_MODE0));
  digitalWrite(_pinCs, LOW);
  _pSpi->transfer(reg | 0x80);
  for(uint16_t i = 0; i < len; i ++) {
    pBuf[i] = _pSpi->transfer(0x00);
  }
  digitalWrite(_pinCs, HIGH);
  _pSpi->endTransaction();
}

void DFRobot_BME280_SPI::writeReg(uint8_t reg, uint8_t *pBuf, uint16_t len)
{
  _pSpi->beginTransaction(SPISettings(500000, MSBFIRST, SPI_MODE0));
  digitalWrite(_pinCs, LOW);
  _pSpi->transfer(reg & 0x7f);
  for(uint16_t i = 0; i < len; i ++) {
    _pSpi->transfer(pBuf[i]);
  }
  digitalWrite(_pinCs, HIGH);
  _pSpi->endTransaction();
}
