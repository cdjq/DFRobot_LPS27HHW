# -*- coding:utf-8 -*-
"""
  @file sensor_interupt.py
  @brief  First set the conditions for the sensor to generate an interrupt: threshold and trigger mode. Then connect the sensor interrupt pin 
  *       with the MCU interrupt pin. When the air pressure reaches the condition at this time, the MCU will generate an interrupt
  *       and print the air pressure and temperature value at this time.
  * @n     Experiment phenomena: The configuration of the sensor and the self-test information is printed on the serial port.
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
    #Initialize the sensor, whether be used to initialize serial port or I2C is up to the current communication way.
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
  * @brief For the accuracy of pressure value, the continuous update of the sensor needs to be disabled.
  '''
  lps27hhw.set_block_data_update()
  print "close success!"
     
  lps27hhw.set_interupt(500)
  print "set interupt success!"

def loop():
  if intflag == 1：
    print "The pressure is above the threshold!"
  else
    print "The pressure is below the threshold!"
  time.sleep(1)

if __name__ == "__main__":
  setup()
  while True:
    loop()
