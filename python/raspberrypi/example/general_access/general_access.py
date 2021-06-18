# -*- coding:utf-8 -*-
"""
  @file general_access.py
  @brief general access data
  @copyright   Copyright (c) 2010 DFRobot Co.Ltd (http://www.dfrobot.com)
  @licence     The MIT License (MIT)
  @author      [PengKaixing](kaixing.peng@dfrobot.com)
  version  V1.0
  date  2021-04-30
  @get from https://www.dfrobot.com
  @url https://github.com/DFRobot/DFRobot_LPS27HHW
"""
import sys
import os
import time

sys.path.append(os.path.dirname(os.path.dirname(os.path.dirname(os.path.realpath(__file__)))))
from dfrobot_lps27hhw import *

#lps27hhw = dfrobot_lps27hhw_I2C (0x01 ,0x5d)         # bus default use I2C1 , iic address is 0x13
lps27hhw = dfrobot_lps27hhw_SPI (27)                # default use spi0 ,cs pin is 27 

def setup():
  while(1):
    #Iniatialize the sensor, whether be used to initialize serial port or IIC is up to the current communication way.
    status = lps27hhw.begin()
    if status == 0:
      print("sensor inint success !")
      break
    else:
      print("sensor inint ,error code is:")
      print(status)
      time.sleep(1)

  #Sensor software reset
  while(lps27hhw.set_reset()):
    print "Unsuccessful reset!"
    time.sleep(1)
  print "reset success!"
      
  '''
  * @brief  For the accuracy of pressure value, the continuous update of the sensor needs to be turned off
  '''
  lps27hhw.set_block_data_update()
  
  '''
   * @brief  Set the sensor to collect the pressure value at the frequency we set and store it in the specified register
   * @param 
            LPS27HHW_POWER_DOWN          
            LPS27HHW_ONE_SHOOT           
            LPS27HHW_1_Hz               
            LPS27HHW_10_Hz               
            LPS27HHW_25_Hz               
            LPS27HHW_50_Hz               
            LPS27HHW_75_Hz               
            LPS27HHW_1_Hz_LOW_NOISE      
            LPS27HHW_10_Hz_LOW_NOISE     
            LPS27HHW_25_Hz_LOW_NOISE     
            LPS27HHW_50_Hz_LOW_NOISE    
            LPS27HHW_75_Hz_LOW_NOISE    
            LPS27HHW_100_Hz              
            LPS27HHW_200_Hz              
  '''  
  lps27hhw.set_data_rate(LPS27HHW_75_Hz_LOW_NOISE)

  
def loop():
  press = lps27hhw.get_pressure_data_hPA()
  temp  = lps27hhw.get_temperature_C()
  alti = lps27hhw.cal_altitude(SEA_LEVEL_PRESSURE, press)
  print "==================="
  print "Pressure : " + str(press) + " hPA"
  print "Temperature : " + str(temp) + " C"
  print "Altitude : " + str(alti) + " m"
  print("===================")
  time.sleep(1)  
  
if __name__ == "__main__":
  setup()
  while True:
    loop()
