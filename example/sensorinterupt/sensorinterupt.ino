/*!
  * @file  sensorinterupt.ino
  * @brief First set the conditions for the sensor to generate an interrupt: threshold and trigger mode
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
 * @brief The pin value needs to be changed according to different MCU.
 * MCU        |    intPin       
 * UNO        |     2,3
 * Mega2560   | 2, 3, 18, 19, 20, 21
 * Leonardo   | 0, 1, 2, 3, 7
 * ESP32      |  All digital ports
 * ESP8266    |  All digital ports
 */
#define intPin D2

//Enable by default, use IIC communication at this time, use SPI communication after shielding.
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
 *        spi communication method
 *        (SDO<-->MISO)    (SDI<-->MOSI)
 *        (SCK<-->SCK)     (CS<-->CS Customizable pins)
 */  
#else
/*!
 * @brief When using SPI communication, the pin value needs to be changed according to different MCU.
 * This value can be any digital IO port
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
     *Iniatialize the sensor, whether be used to initialize SPI or I2C is up to the current communication way.
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
    *Sensor software reset
    */  
  while (LPS27HHW.setReset())
  {
    Serial.println("Unsuccessful reset!");
    delay(1000);
  }
  Serial.println("reset success!");

  /*!
   * @brief  For the accuracy of the pressure value, the continuous update of the sensor needs to be turned off.
   */
  LPS27HHW.closeBlockDataUpdate();

  /*!
   * @brief  Set the sensor to collect the pressure value at the frequency we set before
   *         Use LPS27HHW_75_Hz_LOW_NOISE by default
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

  LPS27HHW.setInterupt(/*Threshold/hPA*/500);

  /*！
   * Configure MCU interrupt
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
