# DFRobot_LPS27HHW
DFRobot's lps27hhw

## DFRobot_lps27hhw Library for Arduino
---------------------------------------------------------
Arduino library is provided for wireless communication

## Table of Contents

* [Installation](#installation)
* [Methods](#methods)
* [Compatibility](#compatibility)
* [History](#history)
* [Credits](#credits)

<snippet>
<content>

## Installation

To use this library download the zip file, uncompress it to a folder named DFRobot_bmm150.
Download the zip file first to use this library and uncompress it to a folder named DFRobot_bmm150.

## Methods

```C++
/*!
 *  @brief Software reset. Default value: 0(0: normal mode; 1: software reset)
 *         .The bit is self-cleared when the reset is completed.
 *  @param  NULL
 *  @return PROPERTY_DISABLE 0
 *          PROPERTY_ENABLE 1
 */
    void setReset(uint8_t val);

/*!
 *  @brief 获取此时传感器是否正常进行了软件复位
 *  @param  NULL
 *  @return 0:没有成功复位
 *          1:已成功复位
 */
    uint8_t getReset();

/*!
 *  @brief The BDU bit is used to inhibit the update of the output registers 
 *         until both upper and lower (and XLOW) register parts are read. In 
 *         default mode (BDU = ‘0’) the output register values are updated 
 *         continuously. If for any reason it is not sure to read faster than 
 *         the output data rate, it is recommended to set the BDU bit to ‘1’. 
 *         In this way, the content of the output registers is not updated 
 *         until MSB, LSB and XLSB have been read which avoids reading values 
 *         related to different sample times. 
 *  @param  PROPERTY_DISABLE 0
 *          PROPERTY_ENABLE 1
 *  @return NULL
 */
    void setBlockDataUpdate(uint8_t val);

/*!
 *  @brief When the ODR bits are set to a value different than '000', the device
 *         is in Continuous mode and automatically acquires a set of data (pressure 
 *         and temperature) at the frequency selected through the ODR[2:0] bits.
 *  @param  LPS27HHW_POWER_DOWN
 *          LPS27HHW_1_Hz  
 *          LPS27HHW_10_Hz 
 *          LPS27HHW_25_Hz 
 *          LPS27HHW_50_Hz 
 *          LPS27HHW_75_Hz 
 *          LPS27HHW_100_Hz
 *          LPS27HHW_200_Hz    
 *  @return NULL
 */
    void setDataRate(eLps27hhwOdr_t val);

/*!
 *  @brief 获取此时的气压值，单位为hPA
 *  @param  NULL
 *  @return 返回此时气压值，以浮点数输出
 */
    float getPressureData_hPA();

/*!
 *  @brief 获取此时的温度值，单位为摄氏度
 *  @param  NULL
 *  @return 返回此时温度值，以浮点数输出
 */
    float getTemperature_C();
/**
  * @brief  Sensing chain FIFO stop values memorization at
  *         threshold level.[set]
  * @param  val  change the values of stop_on_wtm in reg FIFO_CTRL
  * @retval  NULL
  *
  */
    void setFifoStopOnWtm(uint8_t val);

/**
  * @brief   Sensing chain FIFO stop values memorization at threshold
  *          level.[get]
  * @param   NULL
  * @retval  stop_on_wtm
  */
    uint8_t getFifoStopOnWtm();

/**
  * @brief  Fifo Mode selection.[set]
  * @param  val change the values of f_mode in reg FIFO_CTRL
  * @retval NULL
  */
    void setFifoMode(eLps27hhwFMode_t val);

/**
  * @brief  Fifo Mode selection.[get]
  * @param   NULL
  * @retval reg.f_mode;
  */
    uint8_t getFifoMode();

/**
  * @brief  Select the signal that need to route on int pad.[set]
  * @param  val registers CTRL_REG3
  * @retval NULL
  */
    void setPinIntRoute(sLps27hhwCtrlReg3_t *val);

    /**
  * @brief  Select the signal that need to route on int pad.[get]
  * @retval sLps27hhwCtrlReg3_t
  */
    void getPinIntRoute(sLps27hhwCtrlReg3_t *reg);

    /**
  * @brief  FIFO full flag on INT_DRDY pin.[set]
  * @param  uint8_t val: change the values of f_fss5 in reg CTRL_REG3
  * @retval NULL
  */
    void setFifoFullOnInt(uint8_t val);

/**
  * @brief  FIFO full flag on INT_DRDY pin.[get]
  * @retval reg.int_f_full;
  */
    uint8_t getFifoFullOnInt();

/**
  * @brief  FIFO watermark status on INT_DRDY pin.[set]
  * @param  uint8_t val: change the values of f_fth in reg CTRL_REG3
  * @retval NULL
  */
    void setFifoThresholdOnInt(uint8_t val);

/**
  * @brief  FIFO watermark status on INT_DRDY pin.[get]
  * @param NULL
  * @retval  uint8_t: change the values of f_fth in reg CTRL_REG3
  */
    uint8_t getFifoThresholdOnInt();

/**
  * @brief  FIFO overrun interrupt on INT_DRDY pin.[set]
  * @param  uint8_t val: change the values of f_ovr in reg CTRL_REG3
  * @retval NULL
  */
    void setFifoOvrOnInt(uint8_t val);

/**
  * @brief  FIFO overrun interrupt on INT_DRDY pin.[get]
  * @retval reg.int_f_ovr
  */
    uint8_t getFifoOvrOnInt();

/**
  * @brief  Temperature output from FIFO value.[get]
  * @retval 传感器气压值
  */
    float getFifoPressure_hPA();

/**
  * @brief  Temperature output from FIFO value.[get]
  * @retval 传感器温度值
  */
    float getFifoTemperature_C();

/**
  * @brief  FIFO stored data level.[get]
  * @retval fifo中缓存的数据的个数
  */
    uint8_t getFifoDataLevel();

/**
  * @brief   Enable interrupt generation on pressure low/high event.[set]
  * @param   val  change the values of pe in reg INTERRUPT_CFG
  * @retval  NULL
  */
    void setIntOnThreshold(eLps27hhwPe_t val);

    /**
  * @brief  Enable interrupt generation on pressure low/high event.[get]
  * @retval reg.pe
  */
    uint8_t getIntOnThreshold();

/**
  * @brief  User-defined threshold value for pressure interrupt event.[set]
  * @param  buff  buffer that contains data to write
  * @retval NULL
  */
    void setIntTreshold(uint16_t buff);

/**
* @brief   User-defined threshold value for pressure interrupt event.[get]
* @retval  buff     buffer that stores data read
*/
    uint16_t getIntTreshold();

/**
* @brief   User-defined threshold value for pressure interrupt event.[get]
* @param   threshold :气压阈值 trigger_mode：
                      触发方式:  LPS27HHW_NO_THRESHOLD 
                                 LPS27HHW_POSITIVE     
                                 LPS27HHW_NEGATIVE     
                                 LPS27HHW_BOTH         
* @retval  NULL
*/
    void setInterupt(uint16_t threshold, eLps27hhwPe_t trigger_mode);

/**
* @brief   配置传感器从fifo中获取数据
* @retval  NULL
*/
    void cfgGainDataByFifo();

```
## Compatibility

MCU                | Work Well | Work Wrong | Untested  | Remarks
------------------ | :----------: | :----------: | :---------: | -----
Arduino Uno  |      √       |             |            | 
Leonardo  |      √       |             |            | 
Meag2560 |      √       |             |            | 
M0 |      √       |             |            | 
ESP32 |      √       |             |            | 
ESP8266 |      √       |             |            | 
## History

- date 2021-4-29
- version V1.0

## Credits

Written by Pengkaixing(kaixing.peng@dfrobot.com), 2021. (Welcome to our [website](https://www.dfrobot.com/))