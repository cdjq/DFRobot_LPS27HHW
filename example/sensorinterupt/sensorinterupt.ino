/*!
  * @file  sensorinterupt.ino
  * @brief 先设置传感器产生中断的条件：阈值和触发方式
  * @copyright   Copyright (c) 2010 DFRobot Co.Ltd (http://www.dfrobot.com)
  * @licence     The MIT License (MIT)
  * @author      PengKaixing(kaixing.peng@dfrobot.com)
  * @version     V0.1
  * @date        2021-04-28
  * @get         from https://www.dfrobot.com
  * @url         https://github.com/dfrobot/DFRobot_LPS27HHW
  */
#include "DFRobot_LPS27HHW.h"
/*!
 * @brief 需要根据不同的MCU改成不同的引脚值
 * MCU        |    intPin       
 * UNO        |     2,3
 * Mega2560   | 2, 3, 18, 19, 20, 21
 * Leonardo   | 0, 1, 2, 3, 7
 * ESP32      |  所有数字口
 * ESP8266    |  所有数字口
 */
#define intPin D2

//默认打开，此时使用IIC通信，屏蔽之后使用SPI通信
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
 *        spi连接方法
 *        (SDO<-->MISO)    (SDI<-->MOSI)
 *        (SCK<-->SCK)     (CS<-->CS 可自定义引脚)
 */  
#else
/*!
 * @brief 在使用SPI通信的时候需要根据不同的MCU改成不同的引脚值
 * 这个值可以是任意一个数字IO口
 * LPS27HHW_CS : D3(ESP32)
 * LPS27HHW_CS : 10(UNO)
 */
#define LPS27HHW_CS D3
  DFRobot_LPS27HHW_SPI LPS27HHW(/*cs = */LPS27HHW_CS);
#endif

bool intflag;

void funcCallback()
{
  if(digitalRead(intPin))
    intflag=1;
  else
    intflag=0;
}

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
   * @brief  为了保证读取的气压值的准确性，需要把传感器的
   *         持续更新关闭
   */
  LPS27HHW.closeBlockDataUpdate();

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

  LPS27HHW.setInterupt(/*阈值/hPA*/500);

  /*！
   * 配置MCU中断
   */
  pinMode(intPin,INPUT);
  attachInterrupt(digitalPinToInterrupt(intPin), funcCallback, CHANGE);
}

void loop() {
  if(intflag)
    Serial.println("The pressure is above the threshold!");
  else
    Serial.println("The pressure is below the threshold!");
  delay(1000);
}
