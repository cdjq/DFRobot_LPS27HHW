/*!
  * @file  generalAccess.ino
  * @brief Get the barometric pressure (hPA), temperature (°C) and altitude (m) measured by the sensor.
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

//Enable by default, use IIC communication at this time, use SPI communication after shielding it.
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
 *        (SCK<-->SCK)     (CS<-->CS Customizable pins)
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
     *Iniatialize the sensor, whether be used to initialize SPI or I2C is up to the current communication way.
     */    
    uint8_t status = LPS27HHW.begin();
    if(status == 0)
    {
      Serial.println("sensor inint success !");
      break;  
    }else{
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
}

void loop() {
  float press = LPS27HHW.getPressureData_hPA();
  float temp  = LPS27HHW.getTemperature_C();
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
