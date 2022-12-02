# DFRobot_BME280
* [English Version](./README.md)

BME280是一款集成温度、湿度、气压，三位一体的环境传感器。具有高精度，多功能，小尺寸等特点。DFRobot Gravity I2C BME280环境传感器采用Gravity-I2C接口设计，同时预留SPI接口，可以方便快捷的搭建产品原型，应用于环境监测，楼层高度检测，物联网控制等各种应用场景。

Gravity I2C BME280环境传感器使用BOSCH最新MEMS微机电传感器，具备良好的稳定性。气压测量在整个温区非常稳定的，偏置温度系数±1.5 pa/k，当温度变化时，1摄氏度的温度变化导致的误差仅在12.6厘米。这种稳定性，连同其多功能的特点，使得BME280可以适用于各种应用场景。

![产品实物图](./resources/images/BME280.png)


## 产品链接 (https://www.dfrobot.com.cn/goods-1410.html)
    SKU: SEN0236


## 目录

* [概述](#概述)
* [库安装](#库安装)
* [方法](#方法)
* [兼容性](#兼容性)
* [历史](#历史)
* [创作者](#创作者)


## 概述

- 宽电压输入，兼容3.3V/5V控制器
- 温度，湿度和气压，三合一环境参数监测
- Gravity I2C接口，同时预留XH2.54 SPI接口，接口方便实用
- 小尺寸，方便安装


## 库安装

这里有2种安装方法：

1. 使用此库前，请首先下载库文件，将其粘贴到\Arduino\libraries目录中，然后打开examples文件夹并在该文件夹中运行演示。
2. 直接在Arduino软件库管理中搜索下载 DFRobot_BME280 库。


## 方法

```C++

  /**
   * @fn DFRobot_BME280
   * @brief DFRobot_BME280类的构造函数。
   * @return None
   */
  DFRobot_BME280();

  /**
   * @fn begin
   * @brief 初始化传感器
   * @return 枚举的eStatus_t
   */
  eStatus_t   begin();

  /**
   * @fn getTemperature
   * @brief 获取温度测量参数
   * @return 温度单位摄氏度
   */
  float       getTemperature();

  /**
   * @fn getPressure
   * @brief 获取压力测量参数
   * @return 压力单位帕
   */
  uint32_t    getPressure();

  /**
   * @fn getHumidity
   * @brief 获取湿度测量参数
   * @return 湿度的百分比
   */
  float       getHumidity();

  /**
   * @fn calAltitude
   * @brief 根据大气压计算海拔高度
   * @param seaLevelPressure - 海平面气压
   * @param pressure - 大气压
   * @return 海拔单位米
   */
  float       calAltitude(float seaLevelPressure, uint32_t pressure);

  /**
   * @fn reset
   * @brief 复位传感器
   */
  void    reset();

  /**
   * @fn setCtrlMeasMode
   * @brief 设置控制测量模式
   * @param eMode - 枚举量 eCtrlMeasMode_t
   */
  void    setCtrlMeasMode(eCtrlMeasMode_t eMode);

  /**
   * @fn setCtrlMeasSamplingTemp
   * @brief 设置控制测量温度过采样
   * @param eSampling - 枚举量 eSampling_t
   */
  void    setCtrlMeasSamplingTemp(eSampling_t eSampling);

  /**
   * @fn setCtrlMeasSamplingPress
   * @brief 设置控制测量压力过采样
   * @param eSampling - 枚举量 eSampling_t
   */
  void    setCtrlMeasSamplingPress(eSampling_t eSampling);

  /**
   * @fn setCtrlHumiSampling
   * @brief 设置控制测量湿度过采样
   * @param eSampling - 枚举量 eSampling_t
   */
  void    setCtrlHumiSampling(eSampling_t eSampling);

  /**
   * @fn setConfigFilter
   * @brief 设置配置过滤器
   * @param eFilter - 枚举量 eConfigFilter_t
   */
  void    setConfigFilter(eConfigFilter_t eFilter);

  /**
   * @fn setConfigTStandby
   * @brief 设置配置备用时间
   * @param eT - 枚举量 eConfigTStandby_t
   */
  void    setConfigTStandby(eConfigTStandby_t eT);

```


## 兼容性

MCU                | Work Well    | Work Wrong   | Untested    | Remarks
------------------ | :----------: | :----------: | :---------: | :----:
Arduino Uno        |      √       |              |             |
FireBeetle-ESP8266 |      √       |              |             |
FireBeetle-ESP32   |      √       |              |             |


## 历史

- 2022/06/16 - 1.0.0 版本
- 2022/09/22 - 1.0.1 版本
- 2022/12/02 - 1.0.2 版本


## 创作者

Written by Frank(jiehan.guo@dfrobot.com), 2022. (Welcome to our [website](https://www.dfrobot.com/))

