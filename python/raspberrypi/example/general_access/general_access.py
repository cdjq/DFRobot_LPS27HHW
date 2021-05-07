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
    #传感器初始化，用作初始化串口或者初始化IIC，由此时使用的通信方式来决定   
    status = lps27hhw.begin()
    if status == 0:
      print("sensor inint success !")
      break
    else:
      print("sensor inint ,error code is:")
      print(status)
      time.sleep(1)

  #传感器软件复位
  while(lps27hhw.set_reset()):
    print "Unsuccessful reset!"
    time.sleep(1)
  print "reset success!"
      
  '''
  * @brief  为了保证读取的气压值的准确性，需要把传感器的
  *         持续更新关闭
  '''
  lps27hhw.set_block_data_update()
  
  '''
   * @brief  设置传感器以设置的频率的进行气压值采集并且存入
   *         指定寄存器
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