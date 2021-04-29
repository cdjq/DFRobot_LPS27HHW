#include "Arduino.h"
#include <stdlib.h>
#include <SPI.h>
#include <Wire.h>

#define ENABLE_DBG // Open this macro to see the program running in detail

#ifdef ENABLE_DBG
#define DBG(...)                      \
  {                                   \
    Serial.print("[");                \
    Serial.print(__FUNCTION__);       \
    Serial.print("(): ");             \
    Serial.print(__LINE__);           \
    Serial.print(" ] :0x");           \
    Serial.println(__VA_ARGS__, HEX); \
  }
#else
#define DBG(...)
#endif

#define LPS27HHW_CHIP_ID_ADDR 0x0F
#define LPS27HHW_ID 0xB3

#define LPS27HHW_I2C_ADD_H 0x5D
#define LPS27HHW_I2C_ADD_L 0x5C

#define LPS27HHW_PRESS_OUT_XL 0x28
#define LPS27HHW_PRESS_OUT_L 0x29
#define LPS27HHW_PRESS_OUT_H 0x2A
#define LPS27HHW_TEMP_OUT_L 0x2B
#define LPS27HHW_TEMP_OUT_H 0x2C
#define LPS27HHW_FIFO_DATA_OUT_PRESS_XL 0x78
#define LPS27HHW_FIFO_DATA_OUT_PRESS_L 0x79
#define LPS27HHW_FIFO_DATA_OUT_PRESS_H 0x7A
#define LPS27HHW_FIFO_DATA_OUT_TEMP_L 0x7B
#define LPS27HHW_FIFO_DATA_OUT_TEMP_H 0x7C

#define SENSITIVITY_hPA 4096

#define LPS27HHW_CTRL_REG1 0x10
#define LPS27HHW_CTRL_REG2 0x11
#define LPS27HHW_CTRL_REG3 0x12
#define LPS27HHW_FIFO_CTRL 0x13

#define PROPERTY_DISABLE 0
#define PROPERTY_ENABLE 1

#define LPS27HHW_INTERRUPT_CFG 0x0B

#define LPS27HHW_STATUS 0x27

#define RESOLUTION_PRESSURE 4096.0
#define RESOLUTION_TEMPERATURE 100.0

#define LPS27HHW_FIFO_STATUS1 0x25

#define LPS27HHW_THS_P_L 0x0C
#define LPS27HHW_THS_P_H 0x0D

class DFRobot_LPS27HHW
{
  public:
/*
 * FIFO control register
 * ---------------------------------------------------------------------
 * | b7 |  b6  |  b5  |  b4  |     b3       |     b2     |    b1    |    b0    |
 * ---------------------------------------------------------------------
 * | 0  |   0  |  0   |  0   | STOP_ON_WTM  | TRIG_MODES |  F_MODE1 |  F_MODE0 | 
 * ---------------------------------------------------------------------
 */  
    struct sLps27hhwFIFOCtrl_t
    {
      uint8_t f_mode : 3; /* f_mode + trig_modes */
      uint8_t stop_on_wtm : 1;
      uint8_t not_used_01 : 4;
    };

/*
 Control register 1
 * ---------------------------------------------------------------------
 * | b7 |  b6  |  b5  |  b4  |   b3    |    b2    |  b1  |  b0  |
 * ---------------------------------------------------------------------
 * | 0  | ODR2 | ODR1 | ODR0 | EN_LPFP | LPFP_CFG |  BDU |  SIM | 
 * ---------------------------------------------------------------------
 */
    struct sLps27hhwCtrlReg1_t
    {
      uint8_t sim : 1;
      uint8_t bdu : 1;
      uint8_t lpfp_cfg : 2; /* en_lpfp + lpfp_cfg */
      uint8_t odr : 3;
      uint8_t not_used_01 : 1;
    };

/*
 Control register 2
 * ---------------------------------------------------------------------
 * | b7 |   b6    |  b5   |    b4      | b3 |    b2    |     b1       |   b0     |
 * ---------------------------------------------------------------------
 * |BOOT| INT_H_L | PP_OD | IF_ADD_INC | 0  |  SWRESET | LOW_NOISE_EN | ONE_SHOT | 
 * ---------------------------------------------------------------------
 */
    struct sLps27hhwCtrlReg2_t
    {
      uint8_t one_shot : 1; 
      uint8_t low_noise_en : 1;
      uint8_t swreset : 1;
      uint8_t not_used_01 : 1;
      uint8_t if_add_inc : 1;
      uint8_t pp_od : 1;
      uint8_t int_h_l : 1;
      uint8_t boot : 1;
    };

    enum eLps27hhwPe_t
    { 
      LPS27HHW_NO_THRESHOLD  = 0,
      LPS27HHW_POSITIVE      = 1,
      LPS27HHW_NEGATIVE      = 2,
      LPS27HHW_BOTH          = 3,
    };

/*
 Control register 3
 * ---------------------------------------------------------------------
 * | b7 | b6 |    b5      |    b4      |    b3      |  b2  |   b1   |   b0   |
 * ---------------------------------------------------------------------
 * | 0  | 0  | INT_F_FULL | INT_F_WTM  | INT_F_OVR  | DRDY | INT_S1 | INT_S0 | 
 * ---------------------------------------------------------------------
 */
    struct sLps27hhwCtrlReg3_t
    {
      uint8_t int_s : 2;
      uint8_t drdy : 1;
      uint8_t int_f_ovr : 1;
      uint8_t int_f_wtm : 1;
      uint8_t int_f_full: 1;
      uint8_t not_used_01 : 2;
    };

    enum eLps27hhwOdr_t{
      LPS27HHW_POWER_DOWN          = 0x00,
      LPS27HHW_ONE_SHOOT           = 0x08,
      LPS27HHW_1_Hz                = 0x01,
      LPS27HHW_10_Hz               = 0x02,
      LPS27HHW_25_Hz               = 0x03,
      LPS27HHW_50_Hz               = 0x04,
      LPS27HHW_75_Hz               = 0x05,
      LPS27HHW_1_Hz_LOW_NOISE      = 0x11,
      LPS27HHW_10_Hz_LOW_NOISE     = 0x12,
      LPS27HHW_25_Hz_LOW_NOISE     = 0x13,
      LPS27HHW_50_Hz_LOW_NOISE     = 0x14,
      LPS27HHW_75_Hz_LOW_NOISE     = 0x15,
      LPS27HHW_100_Hz              = 0x06,
      LPS27HHW_200_Hz              = 0x07
    };

/*
 Interrupt mode for pressure acquisition configuration
 * ---------------------------------------------------------------------
 * |   b7      |     b6    |    b5    |    b4    |   b3    | b2  |  b1  |  b0  |
 * ---------------------------------------------------------------------
 * | AUTOREFP  | RESET_ARP | AUTOZERO | RESET_AZ | DIFF_EN | LIR |  PLE |  PHE | 
 * ---------------------------------------------------------------------
 */
    struct sLps27hhwInterruptCfg_t
    {
      uint8_t pe : 2; /* ple + phe */
      uint8_t lir : 1;
      uint8_t diff_en : 1;
      uint8_t reset_az : 1;
      uint8_t autozero : 1;
      uint8_t reset_arp : 1;
      uint8_t autorefp : 1;
    };

/*
 Status register (read only)
 * ---------------------------------------------------------------------
 * |   b7   |  b6  |  b5  |  b4  |  b3  |  b2  |  b1  |  b0  |
 * ---------------------------------------------------------------------
 * | -----  | ---- | T_OR | P_OR | ---- | ---- | T_DA | P_DA | 
 * ---------------------------------------------------------------------
 */
    struct sLps27hhwStatus_t
    {
      uint8_t p_da : 1;
      uint8_t t_da : 1;
      uint8_t not_used_01 : 2;
      uint8_t p_or : 1;
      uint8_t t_or : 1;
      uint8_t not_used_02 : 2;
    };

    enum eLps27hhwFMode_t
    {
      LPS27HHW_BYPASS_MODE            = 0,
      LPS27HHW_FIFO_MODE              = 1,
      LPS27HHW_STREAM_MODE            = 2,
      LPS27HHW_DYNAMIC_STREAM_MODE    = 3,
      LPS27HHW_BYPASS_TO_FIFO_MODE    = 5,
      LPS27HHW_BYPASS_TO_STREAM_MODE  = 6,
      LPS27HHW_STREAM_TO_FIFO_MODE    = 7
    };

    union uLps27hhwReg_t
    {
      sLps27hhwInterruptCfg_t interrupt_cfg;
      sLps27hhwStatus_t status;
      sLps27hhwFIFOCtrl_t fifo_ctrl;
      sLps27hhwCtrlReg3_t ctrl_reg3;
    };

    union uAxis1bit32_t
    {
      int32_t i32bit;
      uint8_t u8bit[4];
    };

    struct sLps27hhwThsPL_t
    {
      uint8_t ths : 8;
    };

    struct sLps27hhwThsPH_t
    {  
      uint8_t ths                             : 7;
      uint8_t not_used_01                     : 1;
    };

    DFRobot_LPS27HHW(){};
    ~DFRobot_LPS27HHW(){};
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

  protected:
    virtual void writeReg(uint8_t Reg, uint8_t *Data, uint8_t len)=0;
    virtual int16_t readReg(uint8_t Reg, uint8_t *Data, uint8_t len)=0;  
  private:
};

class DFRobot_LPS27HHW_I2C : public DFRobot_LPS27HHW
{
  public:
    DFRobot_LPS27HHW_I2C(TwoWire *pWire = &Wire, uint8_t addr=LPS27HHW_I2C_ADD_H);
    int8_t begin(void);
  protected:
    void writeReg(uint8_t Reg, uint8_t *Data, uint8_t len);
    int16_t readReg(uint8_t Reg, uint8_t *Data, uint8_t len);
  private:
    TwoWire *_pWire;
    uint8_t _I2C_addr;
};

class DFRobot_LPS27HHW_SPI : public DFRobot_LPS27HHW
{
  public:
    DFRobot_LPS27HHW_SPI(uint8_t csPin = 10, SPIClass *spi = &SPI);
    int8_t begin(void);
  protected:
    void writeReg(uint8_t Reg, uint8_t *Data, uint8_t len);
    int16_t readReg(uint8_t Reg, uint8_t *Data, uint8_t len);
  private:
    SPIClass *_pSpi;
    uint8_t _csPin;
};