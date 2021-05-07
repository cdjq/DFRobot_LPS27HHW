# -*- coding:utf-8 -*-
"""
  @file sensor_interupt.py
  @brief �����ô����������жϵ���������ֵ�ʹ�����ʽ��������������
  *        int�ж�������MCU���ж�����������һ�𣬵���ʱ�����е���ѹ
  *        �ﵽ������ʱ��MCU������жϲ��Ҵ�ӡ��ʱ����ѹ���¶�ֵ
  * @n     ʵ������ ��������������Ϣ��ӡ�ڴ�����,�Բ���Ϣ��ӡ�ڴ�����
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
  print "close success!"
     
  '''
   * @brief  ���ô����������жϵ������� 
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