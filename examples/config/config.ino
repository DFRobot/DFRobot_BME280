/*!
 * @file  config.ino
 * @brief  Download this demo to test config to bme280, connect sensor through 
 * @n  IIC interface. Data will print on your serial monitor
 * @copyright  Copyright (c) 2010 DFRobot Co.Ltd (http://www.dfrobot.com)
 * @license  The MIT License (MIT)
 * @author  [Frank](jiehan.guo@dfrobot.com)
 * @maintainer  [qsjhyy](yihuan.huang@dfrobot.com)
 * @version  V1.0
 * @date  2022-06-16
 * @url  https://github.com/DFRobot/DFRobot_BME280
 */
#include "DFRobot_BME280.h"

typedef DFRobot_BME280_IIC    BME;    // ******** use abbreviations instead of full names ********

/**IIC address is 0x77 when pin SDO is high (BME280 sensor module)*/
/**IIC address is 0x76 when pin SDO is low  */
BME   bme(&Wire, 0x76);

#define SEA_LEVEL_PRESSURE    1015.0f   // sea level pressure

// show last sensor operate status
void printLastOperateStatus(BME::eStatus_t eStatus)
{
  switch(eStatus) {
  case BME::eStatusOK:    Serial.println("everything ok"); break;
  case BME::eStatusErr:   Serial.println("unknow error"); break;
  case BME::eStatusErrDeviceNotDetected:    Serial.println("device not detected"); break;
  case BME::eStatusErrParameter:    Serial.println("parameter error"); break;
  default: Serial.println("unknow status"); break;
  }
}

void setup()
{
  Serial.begin(115200);
  Serial.println("bme config test");
  while(bme.begin() != BME::eStatusOK) {
    Serial.println("bme begin faild");
    printLastOperateStatus(bme.lastOperateStatus);
    delay(2000);
  }
  Serial.println("bme begin success");

  bme.setConfigFilter(BME::eConfigFilter_off);        // set config filter
  bme.setConfigTStandby(BME::eConfigTStandby_125);    // set standby time
  bme.setCtrlMeasSamplingTemp(BME::eSampling_X8);     // set temperature over sampling
  bme.setCtrlMeasSamplingPress(BME::eSampling_X8);    // set pressure over sampling
  bme.setCtrlHumiSampling(BME::eSampling_X8);         // set humidity over sampling
  bme.setCtrlMeasMode(BME::eCtrlMeasMode_normal);     // set control measurement mode to make these settings effective

  delay(100);
}

void loop()
{
  float   temp = bme.getTemperature();
  uint32_t    press = bme.getPressure();
  float   alti = bme.calAltitude(SEA_LEVEL_PRESSURE, press);
  float   humi = bme.getHumidity();

  Serial.println();
  Serial.println("======== start print ========");
  Serial.print("temperature (unit Celsius): "); Serial.println(temp);
  Serial.print("pressure (unit pa):         "); Serial.println(press);
  Serial.print("altitude (unit meter):      "); Serial.println(alti);
  Serial.print("humidity (unit percent):    "); Serial.println(humi);
  Serial.println("========  end print  ========");

  delay(1000);
}
