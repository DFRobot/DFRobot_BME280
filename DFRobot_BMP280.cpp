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

#include "DFRobot_BMP280.h"

const DFRobot_BMP280::sRegs_t PROGMEM   _sRegs = DFRobot_BMP280::sRegs_t();
#ifdef __AVR__
typedef uint16_t    platformBitWidth_t;
#else
typedef uint32_t    platformBitWidth_t;
#endif

const platformBitWidth_t    _regsAddr = (platformBitWidth_t) &_sRegs;

#define writeRegBitsHelper(reg, flied, val)   writeRegBits(regOffset(&(reg)), *(uint8_t*) &(flied), *(uint8_t*) &(val))

#define __DBG   0
#if __DBG
# define __DBG_CODE(x)   Serial.print("__DBG_CODE: "); Serial.print(__FUNCTION__); Serial.print(" "); Serial.print(__LINE__); Serial.print(" "); x; Serial.println()
#else
# define __DBG_CODE(x)
#endif

uint8_t regOffset(const void *pReg)
{
  return ((platformBitWidth_t) pReg - _regsAddr + BMP280_REG_START);
}

DFRobot_BMP280::DFRobot_BMP280() {}

DFRobot_BMP280::eStatus_t DFRobot_BMP280::begin()
{
  __DBG_CODE(Serial.print("last register addr: "); Serial.print(regOffset(&_sRegs.temp), HEX));
  __DBG_CODE(Serial.print("first register addr: "); Serial.print(regOffset(&_sRegs.calib), HEX));
  __DBG_CODE(Serial.print("status register addr: "); Serial.print(regOffset(&_sRegs.status), HEX));
  __DBG_CODE(Serial.print("id register addr: "); Serial.print(regOffset(&_sRegs.chip_id), HEX));
  __DBG_CODE(Serial.print("res0 register addr: "); Serial.print(regOffset(&_sRegs.reserved0), HEX));

  uint8_t   temp = getReg(regOffset(&_sRegs.chip_id));
  if((temp == BMP280_REG_CHIP_ID_DEFAULT) && (lastOperateStatus == eStatusOK)) {
    reset();
    delay(200);
    getCalibrate();
    setCtrlMeasSamplingPress(eSampling_X8);
    setCtrlMeasSamplingTemp(eSampling_X8);
    setConfigFilter(eConfigFilter_off);
    setConfigTStandby(eConfigTStandby_125);
    setCtrlMeasMode(eCtrlMeasMode_normal);    // set control measurement mode to make these settings effective
  } else
    lastOperateStatus = eStatusErrDeviceNotDetected;
  return lastOperateStatus;
}

float DFRobot_BMP280::getTemperature()
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

uint32_t DFRobot_BMP280::getPressure()
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

float DFRobot_BMP280::calAltitude(float seaLevelPressure, uint32_t pressure)
{
  return 44330 * (1.0f - pow(pressure / 100 / seaLevelPressure, 0.1903));
}

void DFRobot_BMP280::reset()
{
  uint8_t   temp = 0xb6;
  writeReg(regOffset(&_sRegs.reset), (uint8_t*) &temp, sizeof(temp));
  delay(100);
}

void DFRobot_BMP280::setCtrlMeasMode(eCtrlMeasMode_t eMode)
{
  sRegCtrlMeas_t    sRegFlied = {0}, sRegVal = {0};
  sRegFlied.mode = 0xff; sRegVal.mode = eMode;
  writeRegBitsHelper(_sRegs.ctrlMeas, sRegFlied, sRegVal);
}

void DFRobot_BMP280::setCtrlMeasSamplingTemp(eSampling_t eSampling)
{
  sRegCtrlMeas_t    sRegFlied = {0}, sRegVal = {0};
  sRegFlied.osrs_t = 0xff; sRegVal.osrs_t = eSampling;
  writeRegBitsHelper(_sRegs.ctrlMeas, sRegFlied, sRegVal);
}

void DFRobot_BMP280::setCtrlMeasSamplingPress(eSampling_t eSampling)
{
  sRegCtrlMeas_t    sRegFlied = {0}, sRegVal = {0};
  sRegFlied.osrs_p = 0xff; sRegVal.osrs_p = eSampling;
  writeRegBitsHelper(_sRegs.ctrlMeas, sRegFlied, sRegVal);
}

void DFRobot_BMP280::setConfigFilter(eConfigFilter_t eFilter)
{
  sRegConfig_t    sRegFlied = {0}, sRegVal = {0};
  sRegFlied.filter = 0xff; sRegVal.filter = eFilter;
  writeRegBitsHelper(_sRegs.config, sRegFlied, sRegVal);
}

void DFRobot_BMP280::setConfigTStandby(eConfigTStandby_t eT)
{
  sRegConfig_t    sRegFlied = {0}, sRegVal = {0};
  sRegFlied.t_sb = 0xff; sRegVal.t_sb = eT;
  writeRegBitsHelper(_sRegs.config, sRegFlied, sRegVal);
}

void DFRobot_BMP280::getCalibrate()
{
  readReg(regOffset(&_sRegs.calib), (uint8_t*) &_sCalib, sizeof(_sCalib));
}

int32_t DFRobot_BMP280::getTemperatureRaw()
{
  sRegTemp_t    sReg;
  readReg(regOffset(&_sRegs.temp), (uint8_t*) &sReg, sizeof(sReg));
  return (((uint32_t) sReg.msb << 12) | ((uint32_t) sReg.lsb << 4) | ((uint32_t) sReg.xlsb));
}

int32_t DFRobot_BMP280::getPressureRaw()
{
  sRegPress_t   sReg;
  readReg(regOffset(&_sRegs.press), (uint8_t*) &sReg, sizeof(sReg));
  return (((uint32_t) sReg.msb << 12) | ((uint32_t) sReg.lsb << 4) | ((uint32_t) sReg.xlsb));
}

uint8_t DFRobot_BMP280::getReg(uint8_t reg)
{
  uint8_t   temp;
  readReg(reg, (uint8_t*) &temp, sizeof(temp));
  return temp;
}

void DFRobot_BMP280::writeRegBits(uint8_t reg, uint8_t flied, uint8_t val)
{
  __DBG_CODE(Serial.print("reg: "); Serial.print(reg, HEX); Serial.print(" flied: "); Serial.print(flied, HEX); Serial.print(" val: "); Serial.print(val, HEX));

  uint8_t   temp;
  readReg(reg, (uint8_t*) &temp, sizeof(temp));
  temp &= ~flied;
  temp |= val;
  writeReg(reg, (uint8_t*) &temp, sizeof(temp));
}

DFRobot_BMP280_IIC::DFRobot_BMP280_IIC(TwoWire *pWire, eSdo_t eSdo)
{
  _pWire = pWire;
  if(eSdo == eSdo_low)
    _addr = 0x76;
  else
    _addr = 0x77;
}

void DFRobot_BMP280_IIC::readReg(uint8_t reg, uint8_t *pBuf, uint16_t len)
{
  lastOperateStatus = eStatusErrDeviceNotDetected;
  _pWire->begin();
  _pWire->beginTransmission(_addr);
  _pWire->write(reg);
  if(_pWire->endTransmission() != 0)
    return;

  _pWire->requestFrom(_addr, len);
  for(uint8_t i = 0; i < len; i ++)
    pBuf[i] = _pWire->read();
  lastOperateStatus = eStatusOK;
}

void DFRobot_BMP280_IIC::writeReg(uint8_t reg, uint8_t *pBuf, uint16_t len)
{
  lastOperateStatus = eStatusErrDeviceNotDetected;
  _pWire->begin();
  _pWire->beginTransmission(_addr);
  _pWire->write(reg);
  for(uint8_t i = 0; i < len; i ++)
    _pWire->write(pBuf[i]);
  if(_pWire->endTransmission() != 0)
    return;
  lastOperateStatus = eStatusOK;
}
