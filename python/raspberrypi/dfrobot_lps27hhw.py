# -*- coding: utf-8 -*
""" 
  @file DFRobot_LPS27HHW.py
  @note DFRobot_LPS27HHW Class infrastructure, implementation of underlying methods
  @copyright   Copyright (c) 2010 DFRobot Co.Ltd (http://www.dfrobot.com)
  @licence     The MIT License (MIT)
  @author      [Pengkaixing](kaixing.peng@dfrobot.com)
  version  V0.1
  date  2021-04-29
  @get from https://www.dfrobot.com
  @url https://github.com/DFRobot/DFRobot_LPS27HHW
"""
import time
import smbus
import spidev
import os
import RPi.GPIO as GPIO
import math
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

I2C_MODE   = 1
SPI_MODE   = 0

LPS27HHW_CHIP_ID_ADDR = 0x0F
LPS27HHW_ID = 0xB3
LPS27HHW_I2C_ADD_H = 0x5D
LPS27HHW_I2C_ADD_L = 0x5C
LPS27HHW_PRESS_OUT_L = 0x29
LPS27HHW_PRESS_OUT_H = 0x2A
LPS27HHW_TEMP_OUT_L = 0x2B
LPS27HHW_TEMP_OUT_H = 0x2C
LPS27HHW_FIFO_DATA_OUT_PRESS_XL = 0x78
LPS27HHW_FIFO_DATA_OUT_PRESS_L = 0x79
LPS27HHW_FIFO_DATA_OUT_PRESS_H = 0x7A
LPS27HHW_FIFO_DATA_OUT_TEMP_L = 0x7B
LPS27HHW_FIFO_DATA_OUT_TEMP_H = 0x7C
SENSITIVITY_hPA = 4096
LPS27HHW_CTRL_REG1 = 0x10
LPS27HHW_CTRL_REG2 = 0x11
LPS27HHW_CTRL_REG3 = 0x12
LPS27HHW_FIFO_CTRL = 0x13
PROPERTY_DISABLE = 0
PROPERTY_ENABLE = 1
LPS27HHW_INTERRUPT_CFG = 0x0B
LPS27HHW_STATUS = 0x27
RESOLUTION_PRESSURE = 4096.0
RESOLUTION_TEMPERATURE = 100.0
LPS27HHW_FIFO_STATUS1 = 0x25
LPS27HHW_THS_P_L = 0x0C
LPS27HHW_THS_P_H = 0x0D

LPS27HHW_POWER_DOWN          = 0x00
LPS27HHW_ONE_SHOOT           = 0x08
LPS27HHW_1_Hz                = 0x01
LPS27HHW_10_Hz               = 0x02
LPS27HHW_25_Hz               = 0x03
LPS27HHW_50_Hz               = 0x04
LPS27HHW_75_Hz               = 0x05
LPS27HHW_1_Hz_LOW_NOISE      = 0x11
LPS27HHW_10_Hz_LOW_NOISE     = 0x12
LPS27HHW_25_Hz_LOW_NOISE     = 0x13
LPS27HHW_50_Hz_LOW_NOISE     = 0x14
LPS27HHW_75_Hz_LOW_NOISE     = 0x15
LPS27HHW_100_Hz              = 0x06
LPS27HHW_200_Hz              = 0x07

LPS27HHW_PRESS_OUT_XL =0x28
LPS27HHW_PRESS_OUT_L =0x29
LPS27HHW_PRESS_OUT_H =0x2A
LPS27HHW_TEMP_OUT_L =0x2B
LPS27HHW_TEMP_OUT_H =0x2C
LPS27HHW_FIFO_DATA_OUT_PRESS_XL =0x78
LPS27HHW_FIFO_DATA_OUT_PRESS_L =0x79
LPS27HHW_FIFO_DATA_OUT_PRESS_H =0x7A
LPS27HHW_FIFO_DATA_OUT_TEMP_L =0x7B
LPS27HHW_FIFO_DATA_OUT_TEMP_H =0x7C

LPS27HHW_BYPASS_MODE            = 0
LPS27HHW_FIFO_MODE              = 1
LPS27HHW_STREAM_MODE            = 2
LPS27HHW_DYNAMIC_STREAM_MODE    = 3
LPS27HHW_BYPASS_TO_FIFO_MODE    = 5
LPS27HHW_BYPASS_TO_STREAM_MODE  = 6
LPS27HHW_STREAM_TO_FIFO_MODE    = 7

LPS27HHW_NO_THRESHOLD  = 0
LPS27HHW_POSITIVE      = 1
LPS27HHW_NEGATIVE      = 2
LPS27HHW_BOTH          = 3

SEA_LEVEL_PRESSURE  =  1015.0

class dfrobot_lps27hhw(object):
  def __init__(self ,bus):
    if bus != 0:
      self.i2cbus = smbus.SMBus(bus)
      self.__i2c_spi = I2C_MODE
    else:
      self.__i2c_spi = SPI_MODE

  #Initialize the sensor, whether be used to initialize serial port or I2C is up to the current communication way.
  def begin(self):
    dummy_read=self.read_reg(LPS27HHW_CHIP_ID_ADDR ,1)
    if dummy_read[0] != 0xB3:
      return -1
    else:
      return 0

  def set_block_data_update(self):
    val = PROPERTY_ENABLE
    reg = self.read_reg(LPS27HHW_CTRL_REG1,1)
    reg[0] = (reg[0] & 0xFD) | (val <<1)
    self.write_reg(LPS27HHW_CTRL_REG1, reg)

  def set_data_rate(self,val):
    ctrl_reg1=self.read_reg(LPS27HHW_CTRL_REG1,1)
    ctrl_reg2=self.read_reg(LPS27HHW_CTRL_REG2,1)
    ctrl_reg1[0] = ((val & 0x07)<<6) | (ctrl_reg1[0] & 0x8f)
    self.write_reg(LPS27HHW_CTRL_REG1, ctrl_reg1)
    ctrl_reg2[0] = (ctrl_reg2[0] & 0xFD) | ((val & 0x10) >> 4)
    ctrl_reg2[0] = ((val & 0x08) >> 3) | (ctrl_reg2[0] & 0xFE)
    self.write_reg(LPS27HHW_CTRL_REG2,ctrl_reg2)

  def set_reset(self):
    val = PROPERTY_ENABLE
    reg = self.read_reg(LPS27HHW_CTRL_REG2, 1)
    reg[0] = (reg[0] & 0xFB) | val;
    self.write_reg(LPS27HHW_CTRL_REG2,reg)  
    reg1=self.read_reg(LPS27HHW_CTRL_REG2,1)
    return (reg1[0] & 0x04) 

  def cfg_gain_data_by_fifo(self):
    self.set_fifo_mode(LPS27HHW_FIFO_MODE)

  def set_fifo_mode(self,val):
    reg = self.read_reg(LPS27HHW_FIFO_CTRL,1)
    reg[0] = (reg[0] & 0xf8) | val
    self.write_reg(LPS27HHW_FIFO_CTRL,reg)

  def get_fifo_pressure_hPA(self):
    buff = self.read_reg(LPS27HHW_FIFO_DATA_OUT_PRESS_XL,3)
    Pressure32 = buff[0] + (buff[1] << 8) + (buff[2] << 16);
    return round((Pressure32 / RESOLUTION_PRESSURE),2)
    
  def get_fifo_temperature_C(self):
    buff = self.read_reg(LPS27HHW_FIFO_DATA_OUT_TEMP_L , 2)
    Temperature16 = buff[0] + (buff[1] << 8)
    return (Temperature16 / RESOLUTION_TEMPERATURE)  

  def get_pressure_data_hPA(self):
    reg = self.read_reg(LPS27HHW_STATUS,1)
    if  (reg[0] & 0x01):
      buff=self.read_reg(LPS27HHW_PRESS_OUT_XL,3)
      Pressure32 = buff[0] + (buff[1] << 8) + (buff[2] << 16)
      return round((Pressure32 / RESOLUTION_PRESSURE),2)
    else:
      return 0.0

  def get_temperature_C(self):
    reg=self.read_reg(LPS27HHW_STATUS,1)
    if (reg[0] & 0x02):
      buff = self.read_reg(LPS27HHW_TEMP_OUT_L,2)
      Temperature16 = buff[0] + (buff[1] << 8)
      return Temperature16 / RESOLUTION_TEMPERATURE
    else:
      return 0.0

  def set_pin_int_route(self,val):
    self.write_reg(LPS27HHW_CTRL_REG3,val)

  def set_int_treshold(self,buff):
    ths_p_l=[0]*1
    ths_p_h=[0]*1
    ths_p_l[0] = buff & 0x00FF
    ths_p_h[0] = (buff & 0x7F00) >> 8
    self.write_reg(LPS27HHW_THS_P_L,ths_p_l)
    self.write_reg(LPS27HHW_THS_P_H,ths_p_h)

  def set_int_on_threshold(self,val):
    reg = self.read_reg(LPS27HHW_INTERRUPT_CFG,1)
    reg[0] = (reg[0] & 0xfc)|val
    if (val == LPS27HHW_NO_THRESHOLD):
      reg[0] = reg[0] & 0xf7
    else:
      reg[0] = reg[0] | 0x08
    self.write_reg(LPS27HHW_INTERRUPT_CFG, reg)

  def set_interupt(self, threshold):
    reg=[0]*1
    reg[0] = 0x01
    self.set_pin_int_route(reg)
    self.set_int_treshold(threshold * 16)
    self.set_int_on_threshold(LPS27HHW_BOTH)

  def cal_altitude(self,seaLevelPressure,pressure):
    return round(44330 * (1.0 - pow(pressure / seaLevelPressure, 0.1903)),2)

#brief An example of an i2c interface module
class dfrobot_lps27hhw_I2C(dfrobot_lps27hhw):
  def __init__(self ,bus ,addr):
    self.__addr = addr
    super(dfrobot_lps27hhw_I2C, self).__init__(bus)

  '''
    @brief writes data to a register
    @param reg register address
    @param value written data
  '''
  def write_reg(self, reg, data):
    while 1:
      try:
        self.i2cbus.write_i2c_block_data(self.__addr ,reg ,data)
        return
      except:
        print("please check connect!")
        #os.system('i2cdetect -y 1')
        time.sleep(1)
        return
  '''
    @brief read the data from the register
    @param reg register address
    @param value read data
  '''
  def read_reg(self, reg ,len):
    try:
      rslt = self.i2cbus.read_i2c_block_data(self.__addr ,reg ,len)
      #print rslt
    except:
      rslt = -1
    return rslt
      
class dfrobot_lps27hhw_SPI(dfrobot_lps27hhw): 
  def __init__(self ,cs, bus = 0, dev = 0,speed = 1000000):
    self.__cs = cs
    GPIO.setup(self.__cs, GPIO.OUT)
    GPIO.output(self.__cs, GPIO.LOW)
    self.__spi = spidev.SpiDev()
    self.__spi.open(bus, dev)
    self.__spi.no_cs = True
    self.__spi.max_speed_hz = speed
    super(dfrobot_lps27hhw_SPI, self).__init__(0)

  '''
    @brief writes data to a register
    @param reg register address
    @param value written data
  '''
  def write_reg(self, reg, data):
    GPIO.output(self.__cs, GPIO.LOW)
    reg = reg&0x7F
    self.__spi.writebytes([reg,data[0]])
    GPIO.output(self.__cs, GPIO.HIGH)

  '''
    @brief read the data from the register
    @param reg register address
    @param value read data
  '''
  def read_reg(self, reg ,len):
    reg = reg | 0x80
    GPIO.output(self.__cs, GPIO.LOW)
    self.__spi.writebytes([reg])
    rslt = self.__spi.readbytes(len)
    GPIO.output(self.__cs, GPIO.HIGH)
    return rslt
