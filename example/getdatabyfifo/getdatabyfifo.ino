/*!
  * @file  getdatabyfifo.ino
  * @brief MCU get data in FIFO of the sensor and print them out by serial port.
  * @n     Experiment phenomena: The configuration of the sensor and the self-test information will be printed on the serial port.
  * @copyright   Copyright (c) 2010 DFRobot Co.Ltd (http://www.dfrobot.com)
  * @licence     The MIT License (MIT)
  * @author      PengKaixing(kaixing.peng@dfrobot.com)
  * @version     V0.1
  * @date        2021-04-28
  * @get         from https://www.dfrobot.com
  * @url         https://github.com/dfrobot/DFRobot_LPS27HHW
  */
#include "DFRobot_LPS27HHW.h"

//Enable by default, use IIC communication at this time, use SPI communication after being shielded.
#define I2C_COMMUNICATION

#ifdef  I2C_COMMUNICATION
/*!
 * @brief When using I2C communication, use the following program to 
 *        construct an object by DFRobot_LPS27HHW_I2C
 * @param pWire I2C controller
 * @param I2C address
 *        LPS27HHW_I2C_ADD_L 0x5C  (SDO:0)
 *        LPS27HHW_I2C_ADD_H 0x5D  (SDO:1)
 */
#define I2C_ADDRESS    LPS27HHW_I2C_ADD_H
  DFRobot_LPS27HHW_I2C LPS27HHW(&Wire,I2C_ADDRESS);
/*!
 * @brief Constructor 
 * @param cs Chip selection pinChip selection pin
 *        spi connection method
 *        (SDO<-->MISO)    (SDI<-->MOSI)
 *        (SCK<-->SCK)     (CS<-->CS Customizable pin)
 */  
#else
/*!
 * @brief When using SPI communication, the pin value needs to be changed according to different MCUs.
 * This value can be any digital IO port
 * LPS27HHW_CS : D3(ESP32)
 * LPS27HHW_CS : 10(UNO)
 */
#define LPS27HHW_CS D3
  DFRobot_LPS27HHW_SPI LPS27HHW(/*cs = */LPS27HHW_CS);
#endif

void setup() {
  Serial.begin(115200);
  while(1)
  {
    /**
     *传感器初始化，用作初始化SPI或者初始化I2C，由此时使用的通信方式来决定
     */    
    uint8_t status = LPS27HHW.begin();
    if(status == 0)
    {
      Serial.println("sensor inint success !");
      break;  
    }else
    {
      Serial.print("sensor inint ,error code is:");
      Serial.println(status);
      delay(1000);
    }
  }
  /**
    *传感器软件复位
    */
  while (LPS27HHW.setReset())
  {
    Serial.println("Unsuccessful reset!");
    delay(1000);
  }
  Serial.println("reset success!");

  /*!
   * @brief  设置传感器以设置的频率的进行气压值采集
   *         默认使用 LPS27HHW_75_Hz_LOW_NOISE
   * @param 
   *         LPS27HHW_POWER_DOWN
   *         LPS27HHW_1_Hz  
   *         LPS27HHW_10_Hz 
   *         LPS27HHW_25_Hz 
   *         LPS27HHW_50_Hz 
   *         LPS27HHW_75_Hz 
   *         LPS27HHW_100_Hz
   *         LPS27HHW_200_Hz 
   *         LPS27HHW_1_Hz_LOW_NOISE 
   *         LPS27HHW_10_Hz_LOW_NOISE
   *         LPS27HHW_25_Hz_LOW_NOISE
   *         LPS27HHW_50_Hz_LOW_NOISE
   *         LPS27HHW_75_Hz_LOW_NOISE
   */
  LPS27HHW.setDataRate();

  /*!
   * @brief  配置MCU从传感器的fifo中获取数据
   */
  LPS27HHW.cfgGainDataByFifo();
}

void loop() {
  float press = LPS27HHW.getFifoPressure_hPA();
  float temp = LPS27HHW.getFifoTemperature_C();
  float alti = LPS27HHW.calAltitude(press);

  Serial.println("===================");
  Serial.print("Pressure : ");
  Serial.print(press);
  Serial.println(" hPA");

  Serial.print("Temperature : ");
  Serial.print(temp);
  Serial.println(" ℃");

  Serial.print("Altitude : ");
  Serial.print(alti);
  Serial.println(" m");

  Serial.println("===================");
  Serial.println("");
  delay(1000);
}
