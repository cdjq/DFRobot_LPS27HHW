#include "DFRobot_LPS27HHW.h"

float DFRobot_LPS27HHW::getPressureData_hPA()
{
  uLps27hhwReg_t reg;
  readReg(LPS27HHW_STATUS, (uint8_t *)&reg, 1);
  if(reg.status.p_da){
    uint8_t buff[3]={0};
    readReg(LPS27HHW_PRESS_OUT_XL, buff, 3);
    uint32_t Pressure32 = (uint32_t)buff[0] | ((uint32_t)buff[1] << 8) | ((uint32_t)buff[2] << 16);
    return (Pressure32 / RESOLUTION_PRESSURE);
  }else
  {
    return 0.0;
  }
}

float DFRobot_LPS27HHW::getTemperature_C()
{
  uLps27hhwReg_t reg;
  readReg(LPS27HHW_STATUS, (uint8_t *)&reg, 1);
  if(reg.status.t_da)
  {
    uint8_t buff[2]={0};
    readReg(LPS27HHW_TEMP_OUT_L, buff, 2);
    uint16_t Temperature16 = (uint16_t)buff[0] + ((uint16_t)buff[1] << 8);
    return (Temperature16 / RESOLUTION_TEMPERATURE);
  }else
  {
    return 0.0;
  }
}

bool DFRobot_LPS27HHW::setReset()
{
  uint8_t val = PROPERTY_ENABLE;
  sLps27hhwCtrlReg2_t reg;
  readReg(LPS27HHW_CTRL_REG2, (uint8_t *)&reg, 1);
  reg.swreset = val;
  writeReg(LPS27HHW_CTRL_REG2, (uint8_t *)&reg, 1);
  readReg(LPS27HHW_CTRL_REG2, (uint8_t *)&reg, 1);
  return reg.swreset;
}

void DFRobot_LPS27HHW::setBlockDataUpdate(uint8_t val)
{
  sLps27hhwCtrlReg1_t reg;
  readReg(LPS27HHW_CTRL_REG1, (uint8_t *)&reg, 1);
  reg.bdu = val;
  writeReg(LPS27HHW_CTRL_REG1, (uint8_t *)&reg, 1);
}

void DFRobot_LPS27HHW::setDataRate(eLps27hhwOdr_t val)
{
  sLps27hhwCtrlReg1_t ctrl_reg1;
  sLps27hhwCtrlReg2_t ctrl_reg2;
  readReg(LPS27HHW_CTRL_REG1, (uint8_t *)&ctrl_reg1, 1);
  readReg(LPS27HHW_CTRL_REG2, (uint8_t *)&ctrl_reg2, 1);
  ctrl_reg1.odr = (uint8_t)val & 0x07;
  writeReg(LPS27HHW_CTRL_REG1, (uint8_t *)&ctrl_reg1, 1);
  ctrl_reg2.low_noise_en = ((uint8_t)val & 0x10) >> 4;
  ctrl_reg2.one_shot = ((uint8_t)val & 0x08) >> 3;
  writeReg(LPS27HHW_CTRL_REG2, (uint8_t *)&ctrl_reg2, 1);
}

void DFRobot_LPS27HHW::setFifoStopOnWtm(uint8_t val)
{
  sLps27hhwFIFOCtrl_t reg;
  readReg(LPS27HHW_FIFO_CTRL, (uint8_t *)&reg, 1);
  reg.stop_on_wtm = val;
  writeReg(LPS27HHW_FIFO_CTRL, (uint8_t *)&reg, 1);
}

uint8_t DFRobot_LPS27HHW::getFifoStopOnWtm()
{
  sLps27hhwFIFOCtrl_t reg;
  readReg(LPS27HHW_FIFO_CTRL, (uint8_t *)&reg, 1);
  return reg.stop_on_wtm;
}

void DFRobot_LPS27HHW::setFifoMode(eLps27hhwFMode_t val)
{
  sLps27hhwFIFOCtrl_t reg;
  readReg(LPS27HHW_FIFO_CTRL, (uint8_t *)&reg, 1);
  reg.f_mode = (uint8_t)val;
  writeReg(LPS27HHW_FIFO_CTRL, (uint8_t *)&reg, 1);
}

uint8_t DFRobot_LPS27HHW::getFifoMode()
{
  sLps27hhwFIFOCtrl_t reg;
  readReg(LPS27HHW_FIFO_CTRL, (uint8_t *)&reg, 1);
  return reg.f_mode;
}

void DFRobot_LPS27HHW::setPinIntRoute(sLps27hhwCtrlReg3_t* val)
{
  writeReg(LPS27HHW_CTRL_REG3,(uint8_t* )&val, 1);
}

void DFRobot_LPS27HHW::getPinIntRoute(sLps27hhwCtrlReg3_t* reg)
{
  readReg(LPS27HHW_CTRL_REG3, (uint8_t *)reg, 1);
}

void DFRobot_LPS27HHW::setFifoFullOnInt(uint8_t val)
{
  sLps27hhwCtrlReg3_t reg;
  readReg(LPS27HHW_CTRL_REG3, (uint8_t *)&reg, 1);
  reg.int_f_full = val;
  writeReg(LPS27HHW_CTRL_REG3, (uint8_t *)&reg, 1);
}

uint8_t DFRobot_LPS27HHW::getFifoFullOnInt()
{
  sLps27hhwCtrlReg3_t reg;
  readReg(LPS27HHW_CTRL_REG3, (uint8_t *)&reg, 1);
  return reg.int_f_full;
}

void DFRobot_LPS27HHW::setFifoThresholdOnInt(uint8_t val)
{
  sLps27hhwCtrlReg3_t reg;
  readReg(LPS27HHW_CTRL_REG3, (uint8_t *)&reg, 1);
  reg.int_f_wtm = val;
  writeReg(LPS27HHW_CTRL_REG3, (uint8_t *)&reg, 1);
}

uint8_t DFRobot_LPS27HHW::getFifoThresholdOnInt()
{
  sLps27hhwCtrlReg3_t reg;
  readReg(LPS27HHW_CTRL_REG3, (uint8_t *)&reg, 1);
  return reg.int_f_wtm;
}

void DFRobot_LPS27HHW::setFifoOvrOnInt(uint8_t val)
{
  sLps27hhwCtrlReg3_t reg;
  readReg(LPS27HHW_CTRL_REG3, (uint8_t *)&reg, 1);
  reg.int_f_ovr = val;
  writeReg(LPS27HHW_CTRL_REG3, (uint8_t *)&reg, 1);
}

uint8_t DFRobot_LPS27HHW::getFifoOvrOnInt()
{
  sLps27hhwCtrlReg3_t reg;
  readReg(LPS27HHW_CTRL_REG3, (uint8_t *)&reg, 1);
  return reg.int_f_ovr;
}

float DFRobot_LPS27HHW::getFifoPressure_hPA()
{
  uint8_t buff[3];
  readReg(LPS27HHW_FIFO_DATA_OUT_PRESS_XL, buff, 3);
  uint32_t Pressure32 = (uint32_t)buff[0] + ((uint32_t)buff[1] << 8) + ((uint32_t)buff[2] << 16);
  return (Pressure32 / RESOLUTION_PRESSURE);
}

float DFRobot_LPS27HHW::getFifoTemperature_C()
{
  uint8_t buff[2];
  readReg(LPS27HHW_FIFO_DATA_OUT_TEMP_L, buff, 2);
  uint16_t Temperature16 = (uint16_t)buff[0] + ((uint16_t)buff[1] << 8);
  return (Temperature16 / RESOLUTION_TEMPERATURE);
}

uint8_t DFRobot_LPS27HHW::getFifoDataLevel()
{
  uint8_t buff;
  readReg(LPS27HHW_FIFO_STATUS1, &buff, 1);
  return buff;
}

void DFRobot_LPS27HHW::setIntOnThreshold(eLps27hhwPe_t val)
{
  sLps27hhwInterruptCfg_t reg;
  readReg(LPS27HHW_INTERRUPT_CFG, (uint8_t *)&reg, 1);
  reg.pe = (uint8_t)val;
  if (val == LPS27HHW_NO_THRESHOLD){
    reg.diff_en = PROPERTY_DISABLE;
  }
  else{
    reg.diff_en = PROPERTY_ENABLE;
  }
  writeReg(LPS27HHW_INTERRUPT_CFG, (uint8_t *)&reg, 1);
}

uint8_t DFRobot_LPS27HHW::getIntOnThreshold()
{
  sLps27hhwInterruptCfg_t reg;
  readReg(LPS27HHW_INTERRUPT_CFG, (uint8_t *)&reg, 1);
  return reg.pe;
}

void DFRobot_LPS27HHW::setIntTreshold(uint16_t buff)
{
  sLps27hhwThsPL_t ths_p_l;
  sLps27hhwThsPH_t ths_p_h;
  ths_p_l.ths = (uint8_t)(buff & 0x00FFU);
  ths_p_h.ths = (uint8_t)((buff & 0x7F00U) >> 8);
  writeReg(LPS27HHW_THS_P_L,(uint8_t *)&ths_p_l, 1);
  writeReg(LPS27HHW_THS_P_H,(uint8_t *)&ths_p_h, 1);
}

uint16_t DFRobot_LPS27HHW::getIntTreshold()
{
  uint16_t buff;
  sLps27hhwThsPL_t ths_p_l;
  sLps27hhwThsPH_t ths_p_h;
  readReg(LPS27HHW_THS_P_L,(uint8_t *)&ths_p_l, 1);
  readReg(LPS27HHW_THS_P_H,(uint8_t *)&ths_p_h, 1);
  buff = (uint16_t)ths_p_h.ths << 8;
  buff |= (uint16_t)ths_p_l.ths;
  return buff;
}

void DFRobot_LPS27HHW::setInterupt(uint16_t threshold, eLps27hhwPe_t trigger_mode)
{
  uLps27hhwReg_t reg;
  reg.ctrl_reg3.int_s = 1;
  setPinIntRoute(&reg.ctrl_reg3);
  setIntTreshold(threshold * 16);
  setIntOnThreshold(trigger_mode);
}

void DFRobot_LPS27HHW::cfgGainDataByFifo()
{
  setFifoMode(LPS27HHW_FIFO_MODE);
}

float DFRobot_LPS27HHW::calAltitude(float seaLevelPressure, float pressure)
{
  return (44330 * (1.0 - pow(pressure / seaLevelPressure, 0.1903)));
}

//iic通信
DFRobot_LPS27HHW_I2C::DFRobot_LPS27HHW_I2C(TwoWire *pWire, uint8_t addr)
{
  this->_pWire = pWire;
  this->_I2C_addr = addr;
}

int8_t DFRobot_LPS27HHW_I2C::begin(void)
{
  uint8_t dummy_read = 0;
  this->_pWire->begin();
  this->_pWire->beginTransmission(_I2C_addr);
  if (this->_pWire->endTransmission() == 0)
  {
    readReg(LPS27HHW_CHIP_ID_ADDR, &dummy_read, 1);
    if (dummy_read != 0xB3)
    {
      return -1;
    }
    else
      return 0;
  }
  else
  {
    return -2;
  }  
}

void DFRobot_LPS27HHW_I2C::writeReg(uint8_t Reg, uint8_t *Data, uint8_t len)
{
  this->_pWire->beginTransmission(this->_I2C_addr);
  this->_pWire->write(Reg);
  for (uint8_t i = 0; i < len; i++)
  {
    _pWire->write(Data[i]);
  }
  _pWire->endTransmission();
}

int16_t DFRobot_LPS27HHW_I2C::readReg(uint8_t Reg, uint8_t *Data, uint8_t len)
{
  int i = 0;
  _pWire->beginTransmission(this->_I2C_addr);
  _pWire->write(Reg);
  if (_pWire->endTransmission() != 0)
  {
    return -1;
  }
  _pWire->requestFrom((uint8_t)this->_I2C_addr, (uint8_t)len);
  while (_pWire->available())
  {
    Data[i++] = _pWire->read();
  }
  return 0;
}

//SPI通信
DFRobot_LPS27HHW_SPI::DFRobot_LPS27HHW_SPI(uint8_t csPin, SPIClass *pSpi)
{
  this->_pSpi = pSpi;
  this->_csPin = csPin;
}

int8_t DFRobot_LPS27HHW_SPI::begin(void)
{
  uint8_t dummy_read = 0;
  pinMode(_csPin, OUTPUT);
  digitalWrite(_csPin, 1);
  this->_pSpi->begin();
  readReg(LPS27HHW_CHIP_ID_ADDR, &dummy_read, 1);
  if (dummy_read != 0xB3)
  {
    return -1;
  }
  else
    return 0;
}

void DFRobot_LPS27HHW_SPI::writeReg(uint8_t Reg, uint8_t *Data, uint8_t len)
{
  digitalWrite(_csPin, 0);
  this->_pSpi->transfer(Reg & 0x7F);
  this->_pSpi->transfer(Data[0]);
  digitalWrite(_csPin, 1);
  for (uint8_t i = 1; i < len; i++)
  {
    this->_pSpi->transfer(Data[i]);
  }
}

int16_t DFRobot_LPS27HHW_SPI::readReg(uint8_t Reg, uint8_t *Data, uint8_t len)
{
  digitalWrite(_csPin, 0);
  this->_pSpi->transfer(Reg | 0x80);
  Data[0] = this->_pSpi->transfer(0xFF);
  for (uint8_t i = 1; i < len; i++)
  {
    Data[i] = this->_pSpi->transfer(0xFF);
  }
  digitalWrite(_csPin, 1);
  return 0;
}