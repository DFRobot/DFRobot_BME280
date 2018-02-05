/*!
 * @file DFRobot_BME280.cpp
 * @brief DFRobot's BME280
 * @n Integrated environmental sensor
 *
 * @copyright    [DFRobot](http://www.dfrobot.com), 2016
 * @copyright    GNU Lesser General Public License
 *
 * @author [yangyang]
 * @version  V1.0
 * @date  2017-7-5
 */
#include <DFRobot_BME280.h>

DFRobot_BME280::DFRobot_BME280(int8_t cspin)
    : cs(cspin)
{ }


bool DFRobot_BME280::begin(uint8_t addr)
{
    i2caddr = addr;

    if (cs == -1) {
        // I2C
        Wire.begin();
    } else {
        pinMode(cs, OUTPUT);
        SPI.begin();
        /*
        SPI.setBitOrder(MSBFIRST);
        SPI.setClockDivider(SPI_CLOCK_DIV32); // 500KHz
        SPI.setDataMode(SPI_MODE0);
        */
    }

    ///< check if sensor, i.e. the chip ID is correct
    if (read8(BME280_REGISTER_CHIPID) != 0x60)
        return false;

    ///< reset the device using soft-reset
    ///< this makes sure the IIR is off, etc.
    write8(BME280_REGISTER_SOFTRESET, 0xB6);

    ///< wait for chip to wake up.
    delay(300);

    ///< if chip is still reading calibration, delay
    while (isReadingCalibration())
        delay(100);

    readCoefficients(); // read trimming parameters, see DS 4.2.2

    setSampling(); // use defaults

    return true;
}

void DFRobot_BME280::setSampling(eSensorMode       mode,
                                 eSensorSampling   tempSampling,
                                 eSensorSampling   pressSampling,
                                 eSensorSampling   humSampling,
                                 eSensorFilter     filter,
                                 eStandbyDuration  duration)
{
    measReg.mode     = mode;
    measReg.osrsT   = tempSampling;
    measReg.osrsP   = pressSampling;


    humReg.osrsH    = humSampling;
    configReg.filter = filter;
    configReg.sb   = duration;


    ///< you must make sure to also set REGISTER_CONTROL after setting the
    ///< CONTROLHUMID register, otherwise the values won't be applied (see DS 5.4.3)
    write8(BME280_REGISTER_CONTROLHUMID, humReg.get());
    write8(BME280_REGISTER_CONFIG, configReg.get());
    write8(BME280_REGISTER_CONTROL, measReg.get());
}


/*!
*   @brief  Writes an 8 bit value over I2C or SPI
*/
void DFRobot_BME280::write8(byte reg, byte value)
{
    if (cs == -1) {
        Wire.beginTransmission(i2caddr);
    #if ARDUINO >= 100
        Wire.write((uint8_t)reg);
        Wire.write((uint8_t)value);
    #else
        Wire.send((uint8_t)reg);
        Wire.send((uint8_t)value);
    #endif        
        Wire.endTransmission();
    } else {
        SPI.beginTransaction(SPISettings(500000, MSBFIRST, SPI_MODE0));
        digitalWrite(cs, LOW);
        SPI.transfer(reg & ~0x80); // write, bit 7 low
        SPI.transfer(value);
        digitalWrite(cs, HIGH);
        SPI.endTransaction(); // release the SPI bus
    }
}


/*!
*   @brief  Reads an 8 bit value over I2C or SPI
*/
uint8_t DFRobot_BME280::read8(byte reg)
{
    uint8_t value;

    if (cs == -1) {
        Wire.beginTransmission(i2caddr);
    #if ARDUINO >= 100
        Wire.write((uint8_t)reg);
    #else
        Wire.send((uint8_t)reg);
    #endif
        Wire.endTransmission();
        Wire.requestFrom(i2caddr, (byte)1);
    #if ARDUINO >= 100
        value = Wire.read();
    #else
        value = Wire.receive();
    #endif
    } else {
        SPI.beginTransaction(SPISettings(500000, MSBFIRST, SPI_MODE0));
        digitalWrite(cs, LOW);
        SPI.transfer(reg | 0x80); // read, bit 7 high
        value = SPI.transfer(0);
        digitalWrite(cs, HIGH);
        SPI.endTransaction(); // release the SPI bus
    }

    return value;
}


/*!
*   @brief  Reads a 16 bit value over I2C or SPI
*/
uint16_t DFRobot_BME280::read16(byte reg)
{
    uint16_t value;

    if (cs == -1) {
        Wire.beginTransmission(i2caddr);
    #if ARDUINO >= 100
        Wire.write((uint8_t)reg);
    #else
        Wire.send((uint8_t)reg);
    #endif
        Wire.endTransmission();
        Wire.requestFrom(i2caddr, (byte)2);
    #if ARDUINO >= 100
        value = (Wire.read() << 8) | Wire.read();
    #else
        value = (Wire.receive() << 8) | Wire.receive();
    #endif
    } else {
        SPI.beginTransaction(SPISettings(500000, MSBFIRST, SPI_MODE0));
        digitalWrite(cs, LOW);
        SPI.transfer(reg | 0x80); // read, bit 7 high
        value = (SPI.transfer(0) << 8) | SPI.transfer(0);
        digitalWrite(cs, HIGH);
        SPI.endTransaction(); // release the SPI bus
    }

    return value;
}


uint16_t DFRobot_BME280::read16_LE(byte reg)
{
    uint16_t temp = read16(reg);
    return (temp >> 8) | (temp << 8);
}


/*!
*   @brief  Reads a signed 16 bit value over I2C or SPI
*/
int16_t DFRobot_BME280::readS16(byte reg)
{
    return (int16_t)read16(reg);
}


int16_t DFRobot_BME280::readS16_LE(byte reg)
{
    return (int16_t)read16_LE(reg);
}


/*!
*   @brief  Reads a 24 bit value over I2C
*/

uint32_t DFRobot_BME280::read24(byte reg)
{
    uint32_t value;

    if (cs == -1) {
        Wire.beginTransmission(i2caddr);
    #if ARDUINO >= 100
        Wire.write((uint8_t)reg);
    #else
        Wire.send((uint8_t)reg);
    #endif
        Wire.endTransmission();
        Wire.requestFrom(i2caddr, (byte)3);
        
    #if ARDUINO >= 100
        value = Wire.read();
        value <<= 8;
        value |= Wire.read();
        value <<= 8;
        value |= Wire.read();
    #else
        value = Wire.receive();
        value <<= 8;
        value |= Wire.receive();
        value <<= 8;
        value |= Wire.receive();
    #endif
    } else {
        SPI.beginTransaction(SPISettings(500000, MSBFIRST, SPI_MODE0));
        digitalWrite(cs, LOW);
        SPI.transfer(reg | 0x80); // read, bit 7 high

        value = SPI.transfer(0);
        value <<= 8;
        value |= SPI.transfer(0);
        value <<= 8;
        value |= SPI.transfer(0);

        digitalWrite(cs, HIGH);
        SPI.endTransaction(); // release the SPI bus
    }

    return value;
}


/*!
*    @brief  Take a new measurement (only possible in forced mode)
*/
void DFRobot_BME280::takeForcedMeasurement()
{
    /* If we are in forced mode, the BME sensor goes back to sleep after each
     * measurement and we need to set it to forced mode once at this point, so
     * it will take the next measurement and then return to sleep again.
     * In normal mode simply does new measurements periodically.
     */
    if (measReg.mode == MODE_FORCED) {
        ///< set to forced mode, i.e. "take next measurement"
        write8(BME280_REGISTER_CONTROL, measReg.get());
        ///< wait until measurement has been completed, otherwise we would read
        ///< the values from the last measurement
        while (read8(BME280_REGISTER_STATUS) & 0x08)
            delay(1);
    }
}


/*!
*    @brief  Reads the factory-set coefficients
*/
void DFRobot_BME280::readCoefficients(void)
{
    bme280Calib.digT1 = read16_LE(BME280_REGISTER_DIG_T1);
    bme280Calib.digT2 = readS16_LE(BME280_REGISTER_DIG_T2);
    bme280Calib.digT3 = readS16_LE(BME280_REGISTER_DIG_T3);

    bme280Calib.digP1 = read16_LE(BME280_REGISTER_DIG_P1);
    bme280Calib.digP2 = readS16_LE(BME280_REGISTER_DIG_P2);
    bme280Calib.digP3 = readS16_LE(BME280_REGISTER_DIG_P3);
    bme280Calib.digP4 = readS16_LE(BME280_REGISTER_DIG_P4);
    bme280Calib.digP5 = readS16_LE(BME280_REGISTER_DIG_P5);
    bme280Calib.digP6 = readS16_LE(BME280_REGISTER_DIG_P6);
    bme280Calib.digP7 = readS16_LE(BME280_REGISTER_DIG_P7);
    bme280Calib.digP8 = readS16_LE(BME280_REGISTER_DIG_P8);
    bme280Calib.digP9 = readS16_LE(BME280_REGISTER_DIG_P9);

    bme280Calib.digH1 = read8(BME280_REGISTER_DIG_H1);
    bme280Calib.digH2 = readS16_LE(BME280_REGISTER_DIG_H2);
    bme280Calib.digH3 = read8(BME280_REGISTER_DIG_H3);
    bme280Calib.digH4 = (read8(BME280_REGISTER_DIG_H4) << 4) | (read8(BME280_REGISTER_DIG_H4+1) & 0xF);
    bme280Calib.digH5 = (read8(BME280_REGISTER_DIG_H5+1) << 4) | (read8(BME280_REGISTER_DIG_H5) >> 4);
    bme280Calib.digH6 = (int8_t)read8(BME280_REGISTER_DIG_H6);
}


/*!
*    @brief return true if chip is busy reading cal data
*/
bool DFRobot_BME280::isReadingCalibration(void)
{
    uint8_t const rStatus = read8(BME280_REGISTER_STATUS);

    return (rStatus & (1 << 0)) != 0;
}


/*!
*    @brief  Returns the temperature from the sensor
*/
float DFRobot_BME280::temperatureValue(void)
{
    int32_t var1, var2;

    int32_t adc = read24(BME280_REGISTER_TEMPDATA);
    if (adc == 0x800000) // value in case temp measurement was disabled
        return NAN;
    adc >>= 4;

    var1 = ((((adc>>3) - ((int32_t)bme280Calib.digT1 <<1))) *
            ((int32_t)bme280Calib.digT2)) >> 11;

    var2 = (((((adc>>4) - ((int32_t)bme280Calib.digT1)) *
              ((adc>>4) - ((int32_t)bme280Calib.digT1))) >> 12) *
            ((int32_t)bme280Calib.digT3)) >> 14;

    fine = var1 + var2;

    float T = (fine * 5 + 128) >> 8;
    return T/100;
}


/*!
*    @brief  Returns the temperature from the sensor
*/
float DFRobot_BME280::pressureValue(void)
{
    int64_t var1, var2, p;

    temperatureValue(); // must be done first to get t_fine

    int32_t adc = read24(BME280_REGISTER_PRESSUREDATA);
    if (adc == 0x800000) // value in case pressure measurement was disabled
        return NAN;
    adc >>= 4;

    var1 = ((int64_t)fine) - 128000;
    var2 = var1 * var1 * (int64_t)bme280Calib.digP6;
    var2 = var2 + ((var1*(int64_t)bme280Calib.digP5)<<17);
    var2 = var2 + (((int64_t)bme280Calib.digP4)<<35);
    var1 = ((var1 * var1 * (int64_t)bme280Calib.digP3)>>8) +
           ((var1 * (int64_t)bme280Calib.digP2)<<12);
    var1 = (((((int64_t)1)<<47)+var1))*((int64_t)bme280Calib.digP1)>>33;

    if (var1 == 0) {
        return 0; // avoid exception caused by division by zero
    }
    p = 1048576 - adc;
    p = (((p<<31) - var2)*3125) / var1;
    var1 = (((int64_t)bme280Calib.digP9) * (p>>13) * (p>>13)) >> 25;
    var2 = (((int64_t)bme280Calib.digP8) * p) >> 19;

    p = ((p + var1 + var2) >> 8) + (((int64_t)bme280Calib.digP7)<<4);
    return (float)p/256;
}


/*!
*    @brief  Returns the humidity from the sensor
*/
float DFRobot_BME280::humidityValue(void)
{
    temperatureValue(); // must be done first to get t_fine

    int32_t adc_H = read16(BME280_REGISTER_HUMIDDATA);
    if (adc_H == 0x8000) // value in case humidity measurement was disabled
        return NAN;

    int32_t tmp;

    tmp = (fine - ((int32_t)76800));

    tmp = (((((adc_H << 14) - (((int32_t)bme280Calib.digH4) << 20) -
                    (((int32_t)bme280Calib.digH5) * tmp)) + ((int32_t)16384)) >> 15) *
                 (((((((tmp * ((int32_t)bme280Calib.digH6)) >> 10) *
                      (((tmp * ((int32_t)bme280Calib.digH3)) >> 11) + ((int32_t)32768))) >> 10) +
                    ((int32_t)2097152)) * ((int32_t)bme280Calib.digH2) + 8192) >> 14));

    tmp = (tmp - (((((tmp >> 15) * (tmp >> 15)) >> 7) *
                               ((int32_t)bme280Calib.digH1)) >> 4));

    tmp = (tmp < 0) ? 0 : tmp;
    tmp = (tmp > 419430400) ? 419430400 : tmp;
    float h = (tmp>>12);
    return  h / 1024.0;
}


/*!
*    Calculates the altitude (in meters) from the specified atmospheric
*    pressure (in hPa), and sea-level pressure (in hPa).
*
*    @param  seaLevel      Sea-level pressure in hPa
*    @param  atmospheric   Atmospheric pressure in hPa
*/
float DFRobot_BME280::altitudeValue(float seaLevel)
{
    float atmospheric = pressureValue() / 100.0F;
    return 44330.0 * (1.0 - pow(atmospheric / seaLevel, 0.1903));
}


/*!
*    Calculates the pressure at sea level (in hPa) from the specified altitude
*    (in meters), and atmospheric pressure (in hPa).
*    @param  altitude      Altitude in meters
*    @param  atmospheric   Atmospheric pressure in hPa
*/
float DFRobot_BME280::seaLevelForAltitude(float altitude, float atmospheric)
{
    return atmospheric / pow(1.0 - (altitude/44330.0), 5.255);
}
