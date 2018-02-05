/*!
 * @file DFRobot_BME280.h
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
#ifndef __BME280_H__
#define __BME280_H__


#include <SPI.h>
#include <Wire.h>
#include <Arduino.h>

/*I2C ADDRESS/BITS*/
#define BME280_ADDRESS                     0x76

/*REGISTERS*/
#define    BME280_REGISTER_DIG_T1          0x88
#define    BME280_REGISTER_DIG_T2          0x8A
#define    BME280_REGISTER_DIG_T3          0x8C

#define    BME280_REGISTER_DIG_P1          0x8E
#define    BME280_REGISTER_DIG_P2          0x90
#define    BME280_REGISTER_DIG_P3          0x92
#define    BME280_REGISTER_DIG_P4          0x94
#define    BME280_REGISTER_DIG_P5          0x96
#define    BME280_REGISTER_DIG_P6          0x98
#define    BME280_REGISTER_DIG_P7          0x9A
#define    BME280_REGISTER_DIG_P8          0x9C
#define    BME280_REGISTER_DIG_P9          0x9E

#define    BME280_REGISTER_DIG_H1          0xA1
#define    BME280_REGISTER_DIG_H2          0xE1
#define    BME280_REGISTER_DIG_H3          0xE3
#define    BME280_REGISTER_DIG_H4          0xE4
#define    BME280_REGISTER_DIG_H5          0xE5
#define    BME280_REGISTER_DIG_H6          0xE7

#define    BME280_REGISTER_CHIPID          0xD0
#define    BME280_REGISTER_VERSION         0xD1
#define    BME280_REGISTER_SOFTRESET       0xE0

#define    BME280_REGISTER_CAL26           0xE1  // R calibration stored in 0xE1-0xF0

#define    BME280_REGISTER_CONTROLHUMID    0xF2
#define    BME280_REGISTER_STATUS          0XF3
#define    BME280_REGISTER_CONTROL         0xF4
#define    BME280_REGISTER_CONFIG          0xF5
#define    BME280_REGISTER_PRESSUREDATA    0xF7
#define    BME280_REGISTER_TEMPDATA        0xFA
#define    BME280_REGISTER_HUMIDDATA       0xFD


enum eSensorSampling {
    SAMPLING_NONE = 0b000,
    SAMPLING_X1   = 0b001,
    SAMPLING_X2   = 0b010,
    SAMPLING_X4   = 0b011,
    SAMPLING_X8   = 0b100,
    SAMPLING_X16  = 0b101
};

enum eSensorMode {
    MODE_SLEEP  = 0b00,
    MODE_FORCED = 0b01,
    MODE_NORMAL = 0b11
};

enum eSensorFilter {
    FILTER_OFF = 0b000,
    FILTER_X2  = 0b001,
    FILTER_X4  = 0b010,
    FILTER_X8  = 0b011,
    FILTER_X16 = 0b100
};

/*!
*   @brief standby durations in ms
*/
enum eStandbyDuration {
    STANDBY_MS_0_5  = 0b000,
    STANDBY_MS_10   = 0b110,
    STANDBY_MS_20   = 0b111,
    STANDBY_MS_62_5 = 0b001,
    STANDBY_MS_125  = 0b010,
    STANDBY_MS_250  = 0b011,
    STANDBY_MS_500  = 0b100,
    STANDBY_MS_1000 = 0b101
};


/*CALIBRATION DATA*/
typedef struct {
    uint16_t digT1;
    int16_t  digT2;
    int16_t  digT3;

    uint16_t digP1;
    int16_t  digP2;
    int16_t  digP3;
    int16_t  digP4;
    int16_t  digP5;
    int16_t  digP6;
    int16_t  digP7;
    int16_t  digP8;
    int16_t  digP9;

    uint8_t  digH1;
    int16_t  digH2;
    uint8_t  digH3;
    int16_t  digH4;
    int16_t  digH5;
    int8_t   digH6;
} tBme280CalibData;

/*!
*   @brief The ctrl_hum register
*/
typedef struct {
    ///< unused - don't set
    unsigned int none : 5;

    ///< pressure oversampling
    unsigned int osrsH : 3;

    unsigned int get() {
        return (osrsH);
    }
}tCtrlHum;
/*!
*   @brief The ctrl_meas register
*/
typedef struct{
    ///< temperature oversampling
    unsigned int osrsT : 3;

    ///< pressure oversampling
    unsigned int osrsP : 3;

    ///< device mode
    unsigned int mode : 2;

    unsigned int get() {
        return (osrsT << 5) | (osrsP << 3) | mode;
    }
}tCtrlMeas;

    /*!
    *   @brief The config register
    */
typedef struct {
    ///< inactive duration (standby time) in normal mode
    unsigned int sb : 3;

    ///< filter settings
    unsigned int filter : 3;

    ///< unused - don't set
    unsigned int none : 1;
    unsigned int spi3wEn : 1;

    unsigned int get() {
        return (sb << 5) | (filter << 3) | spi3wEn;
    }
}tConfig;

class DFRobot_BME280 {
public:
    /*!
    *   @brief  constructors
    */
    DFRobot_BME280(int8_t cspin = -1);

    /*!
    *   @brief  Initialise
    */
    bool  begin(uint8_t addr = BME280_ADDRESS);

    void takeForcedMeasurement();

    /*!
    *   @brief  Reads the temperature
    */
    float temperatureValue(void);

    /*!
    *   @brief Reads the pressue
    */
    float pressureValue(void);

    /*!
    *   @brief Reads the humidity
    */
    float humidityValue(void);

    /*!
    *   @brief Reads the altitude
    */
    float altitudeValue(float seaLevel);

    float seaLevelForAltitude(float altitude, float pressure);


    /*!
    *    @brief  setup sensor
    *
    *    This is simply a overload to the normal begin()-function.
    */
    void setSampling(eSensorMode mode              = MODE_NORMAL,
                     eSensorSampling tempSampling  = SAMPLING_X16,
                     eSensorSampling pressSampling = SAMPLING_X16,
                     eSensorSampling humSampling   = SAMPLING_X16,
                     eSensorFilter filter          = FILTER_OFF,
                     eStandbyDuration duration     = STANDBY_MS_0_5
                    );


private:
    void readCoefficients(void);
    bool isReadingCalibration(void);
    uint8_t spixfer(uint8_t x);

    void      write8(byte reg, byte value);
    uint8_t   read8(byte reg);
    uint16_t  read16(byte reg);
    uint32_t  read24(byte reg);
    int16_t   readS16(byte reg);
    uint16_t  read16_LE(byte reg); // little endian
    int16_t   readS16_LE(byte reg); // little endian

    uint8_t   i2caddr;
    int32_t   sensorID;
    int32_t   fine;

    int8_t cs;

    tBme280CalibData bme280Calib;
    tConfig configReg;
    tCtrlMeas measReg;
    tCtrlHum humReg;
};

#endif
