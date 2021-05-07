# -*- coding:utf-8 -*-
"""
  @file get_data_by_fifo.py
  @brief get data by fifo
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
    #��������ʼ����������ʼ�����ڻ��߳�ʼ��IIC���ɴ�ʱʹ�õ�ͨ�ŷ�ʽ������   
    status = lps27hhw.begin()
    if status == 0:
      print("sensor inint success !")
      break
    else:
      print("sensor inint ,error code is:")
      print(status)
      time.sleep(1)  
      
  #�����������λ
  while(lps27hhw.set_reset()):
    print "Unsuccessful reset!"
    time.sleep(1)
  print "reset success!"
      
  '''
  * @brief  Ϊ�˱�֤��ȡ����ѹֵ��׼ȷ�ԣ���Ҫ�Ѵ�������
  *         �������¹ر�
  '''
  lps27hhw.set_block_data_update()
  
  '''
   * @brief  ���ô����������õ�Ƶ�ʵĽ�����ѹֵ�ɼ����Ҵ���
   *         ָ���Ĵ���
   * @param 
   *         LPS27HHW_POWER_DOWN
   *         LPS27HHW_1_Hz  
   *         LPS27HHW_10_Hz 
   *         LPS27HHW_25_Hz 
   *         LPS27HHW_50_Hz 
   *         LPS27HHW_75_Hz 
   *         LPS27HHW_100_Hz
   *         LPS27HHW_200_Hz 
  '''  
  lps27hhw.set_data_rate(LPS27HHW_75_Hz_LOW_NOISE)  
  
  '''
   * @brief  ����MCU�Ӵ�������fifo�л�ȡ����
   '''
  lps27hhw.cfg_gain_data_by_fifo()
      
def loop():
  press = lps27hhw.get_fifo_pressure_hPA()
  temp  = lps27hhw.get_fifo_temperature_C()
  alti = lps27hhw.cal_altitude(SEA_LEVEL_PRESSURE, press)
  print "==================="
  print "Pressure : " + str(press) + " hPA"
  print "Temperature : " + str(temp) + " C" 
  print "Altitude : " + str(alti) + " m"
  print "===================" 
  print "" 
  time.sleep(1)  
  
  
if __name__ == "__main__":
  setup()
  while True:
    loop()