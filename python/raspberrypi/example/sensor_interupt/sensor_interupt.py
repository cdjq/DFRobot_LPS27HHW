# -*- coding:utf-8 -*-
"""
  @file sensor_interupt.py
  @brief 先设置传感器产生中断的条件：阈值和触发方式，并将传感器的
  *        int中断引脚与MCU的中断引脚连接在一起，当此时环境中的气压
  *        达到条件的时候，MCU会产生中断并且打印此时的气压和温度值
  * @n     实验现象 传感器的配置信息打印在串口上,自测信息打印在串口上
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
import pigpio

sys.path.append(os.path.dirname(os.path.dirname(os.path.dirname(os.path.realpath(__file__)))))
from dfrobot_lps27hhw import *

#lps27hhw = dfrobot_lps27hhw_I2C (0x01 ,0x5d)         # bus default use I2C1 , iic address is 0x13
lps27hhw = dfrobot_lps27hhw_SPI (27)                # default use spi0 ,cs pin is 27 

intflag = 0
intPin = 17

def test_callback(void):
  global intflag 
  global intPin
  if(GPIO.input(intPin)):
    intflag=1
  else:
    intflag=0

def setup():
  GPIO.setup(intPin, GPIO.IN)
  GPIO.add_event_detect(intPin, GPIO.RISING, callback=test_callback, bouncetime=200)
    
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
  print "close success!"
     
  '''
   * @brief  设置传感器产生中断的条件： 
   *         LPS27HHW_NO_THRESHOLD 
   *         LPS27HHW_POSITIVE     
   *         LPS27HHW_NEGATIVE     
   *         LPS27HHW_BOTH    
   *'''
  lps27hhw.set_interupt(500,LPS27HHW_POSITIVE)
  print "set interupt success!"

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