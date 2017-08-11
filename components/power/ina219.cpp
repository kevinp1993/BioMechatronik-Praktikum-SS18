#include <amiro/power/ina219.hpp>

#include <ch.hpp>
#include <chprintf.h>
#include <global.hpp>

using namespace chibios_rt;
using namespace amiro;
using namespace INA219;

extern Global global;

Driver::Driver(I2CDriver &i2c_driver, const uint8_t i2c_address) :
  BaseSensor<InitData,CalibData>(), i2c_driver(&i2c_driver), tx_params({i2c_address, NULL, 0, NULL, 0}), current_lsb_uA(0)
{
  this->config.content.brng = Configuration::BRNG_DEFAULT;
  this->config.content.pg = Configuration::PGA_DEFAULT;
  this->config.content.badc = Configuration::ADC_DEFAULT;
  this->config.content.sadc = Configuration::ADC_DEFAULT;
  this->config.content.mode = Configuration::MODE_DEFAULT;

  return;
}

Driver::~Driver()
{}

msg_t
Driver::init(InitData* initialization_data)
{
  if(!initialization_data)
  {
    return ERROR;
  }

  // write configuration
  if (this->writeRegister(REG_CONFIGURATION, initialization_data->configuration.value & MASK_CONFIGURATION)) {
    return ERROR;
  }
  this->config.value = initialization_data->configuration.value & MASK_CONFIGURATION;

  if (this->writeRegister(REG_CALIBRATION, initialization_data->calibration & MASK_CALIBRATION)) {
    return ERROR;
  }

  this->current_lsb_uA = initialization_data->current_lsb_uA;

  return this->update();
}

msg_t
Driver::update()
{
  this->status.power = 0;
  msg_t res = this->readRegister(REG_BUS_VOLTAGE, this->status.bus_voltage.value);
  res |= this->readRegister(REG_POWER, this->status.power);

  // if the power register was not updated yet, try again
  while (!this->status.bus_voltage.content.conversion_ready || this->status.power == 0 || res != 0)
  {
    BaseThread::sleep(MS2ST(10));
    res |= this->readRegister(REG_BUS_VOLTAGE, this->status.bus_voltage.value);
    res |= this->readRegister(REG_POWER, this->status.power);
  }

  return res ? ERROR : SUCCESS;
}

msg_t
Driver::wakeup()
{
  if (this->writeRegister(REG_CONFIGURATION, this->config.value)) {
    return ERROR;
  } else {
    return this->update();
  }
}

msg_t
Driver::hibernate()
{
  Configuration::Register tmp_config = this->config;
  tmp_config.content.mode = Configuration::MODE_PowerDown;
  if (this->writeRegister(REG_CONFIGURATION, tmp_config.value)) {
    return ERROR;
  } else {
    return SUCCESS;
  }

}

#ifndef AMIRO_NCALIBRATION
msg_t
Driver::calibration(CalibData* calibration_data)
{
  if (!calibration_data) {
    return ERROR;
  }

  uint16_t current_lsb_uA = calibration_data->input.current_lsb_uA;
  if (current_lsb_uA < calibration_data->input.max_expected_current_A / 0.032767f) {
    current_lsb_uA = calibration_data->input.max_expected_current_A / 0.032767f;
  } else if (current_lsb_uA > calibration_data->input.max_expected_current_A / 0.004096f) {
    current_lsb_uA = calibration_data->input.max_expected_current_A / 0.004096f;
  }

  const uint16_t calibration_value = uint16_t(40960 / (current_lsb_uA * calibration_data->input.shunt_resistance_O));

  float V_shunt_max;
  switch (calibration_data->input.configuration.content.pg)
  {
    case Configuration::PGA_40mV:
      V_shunt_max = 0.04f;
      break;
    case Configuration::PGA_80mV:
      V_shunt_max = 0.08f;
      break;
    case Configuration::PGA_160mV:
      V_shunt_max = 0.16f;
      break;
    case Configuration::PGA_320mV:
      V_shunt_max = 0.32f;
      break;
  }

  const float max_current_before_overflow = ( (current_lsb_uA * 0.032767f >= V_shunt_max) / (calibration_data->input.shunt_resistance_O) )?
                                            V_shunt_max / calibration_data->input.shunt_resistance_O :
                                            current_lsb_uA * 0.032767f;

  const float max_shuntvoltage_before_overflow = ( (max_current_before_overflow * calibration_data->input.shunt_resistance_O) >= V_shunt_max )?
                                                 V_shunt_max :
                                                 max_current_before_overflow * calibration_data->input.shunt_resistance_O;

  calibration_data->output.max_current_before_overflow_A = max_current_before_overflow;
  calibration_data->output.max_shuntvoltage_before_overflow_V = max_shuntvoltage_before_overflow;
  calibration_data->output.current_lsb_uA = current_lsb_uA;
  calibration_data->output.calibration_value = calibration_value;

  return SUCCESS;
}
#endif

#ifndef AMIRO_NSELFTEST
msg_t
Driver::selftest()
{
  struct RegisterContent {
    Configuration::Register configuration;
    uint16_t shunt_voltage = 0;
    BusVoltage bus_voltage;
    uint16_t power = 0;
    uint16_t current = 0;
    uint16_t calibration = 0;
  };

  // backup the current status
  RegisterContent backup;
  msg_t res = this->readRegister(REG_CONFIGURATION, backup.configuration.value);
  res |= this->readRegister(REG_SHUNT_VOLTAGE, backup.shunt_voltage);
  res |= this->readRegister(REG_BUS_VOLTAGE, backup.bus_voltage.value);
  res |= this->readRegister(REG_POWER, backup.power);
  res |= this->readRegister(REG_CURRENT, backup.current);
  res |= this->readRegister(REG_CALIBRATION, backup.calibration);
  if (res ||
      !(backup.configuration.value ||
        backup.shunt_voltage ||
        backup.bus_voltage.value ||
        backup.power ||
        backup.current ||
        backup.calibration))
  {
    return ST_FAIL_BACKUP;
  }

  // reset the INA219
  if (this->reset())
  {
    return ST_FAIL_RESET;
  }

  // read the configuration
  Configuration::Register config = this->readConfiguration();

  // check for the default configuration
  Configuration::Register config_default;
  config_default.content.brng = Configuration::BRNG_DEFAULT;
  config_default.content.pg = Configuration::PGA_DEFAULT;
  config_default.content.badc = Configuration::ADC_DEFAULT;
  config_default.content.sadc = Configuration::ADC_DEFAULT;
  config_default.content.mode = Configuration::MODE_DEFAULT;
  if (config.value != config_default.value)
  {
    return ST_FAIL_IS_DEFAULT;
  }

  // revert to the previous configuration
  if (this->writeRegister(REG_CONFIGURATION, backup.configuration.value))
  {
    return ST_FAIL_WRITE_CONFIG;
  }

  // revert to the previous calibration
  if (this->writeRegister(REG_CALIBRATION, backup.calibration))
  {
    return ST_FAIL_WRITE_CALIB;
  }

  // read the current configuration
  if (this->readConfiguration().value != backup.configuration.value)
  {
    return ST_FAIL_CHECK_CONFIG;
  }

  // read the current calibration
  if (this->readCalibration() != backup.calibration)
  {
    return ST_FAIL_CHECK_CALIB;
  }

  // read and print the current status
  INA219::BusVoltage bus_voltage = this->readBusVoltage();
  uint16_t power;
  this->readRegister(REG_POWER, power);
  // wait until the bus voltage and the power register are valid
  while (!bus_voltage.conversion_ready || power == 0)
  {
    BaseThread::sleep(MS2ST(10));
    bus_voltage = this->readBusVoltage();
    this->readRegister(REG_POWER, power);
  }
  chprintf((BaseSequentialStream*) &global.sercanmux1, "shunt voltage : %fV\n", this->readShuntVoltage_uV() / 1000000.f);
  chprintf((BaseSequentialStream*) &global.sercanmux1, "bus voltage   : %fV\n", bus_voltage.voltage_uV / 1000000.f);
  chprintf((BaseSequentialStream*) &global.sercanmux1, "power         : %fW\n", this->readPower_uW() / 1000000.f);
  chprintf((BaseSequentialStream*) &global.sercanmux1, "current       : %fA\n", this->readCurrent_uA() / 1000000.f);

  return ST_OK;
}
#endif

Configuration::Register
Driver::readConfiguration()
{
  Configuration::Register reg;
  this->readRegister(REG_CONFIGURATION, reg.value);
  return reg;
}

int32_t
Driver::readShuntVoltage_uV()
{
  uint16_t val = 0;
  this->readRegister(REG_SHUNT_VOLTAGE, val);
  /*
   * Depending on the configuration either one, two, three or four most significant bits are used for coding the sign (no two's complement!)
   * -> Masking bits in order to separate sign bits and value bits.
   * Furthermore, the least significant bit represents 10uV.
   * -> Multiplication by 10 or -10 respectively.
   */
  switch (this->config.content.pg)
  {
    case Configuration::PGA_320mV:
      return ( uint32_t(val & 0x7FFFu) * ((val & 0x8000u)? -10 : 10) );
    case Configuration::PGA_160mV:
      return ( uint32_t(val & 0x3FFFu) * ((val & 0xC000u)? -10 : 10) );
    case Configuration::PGA_80mV:
      return ( uint32_t(val & 0x1FFFu) * ((val & 0xE000u)? -10 : 10) );
    case Configuration::PGA_40mV:
      return ( uint32_t(val & 0x0FFFu) * ((val & 0xF000u)? -10 : 10) );
  }
}

INA219::BusVoltage
Driver::readBusVoltage()
{
  Driver::BusVoltage reg;
  this->readRegister(REG_BUS_VOLTAGE, reg.value);
  return this->busVoltageReg2uV(reg);
}

uint32_t
Driver::readPower_uW()
{
  uint16_t val = 0;
  this->readRegister(REG_POWER, val);
  return this->powerReg2uW(val);
}

int32_t
Driver::readCurrent_uA()
{
  uint16_t val = 0;
  this->readRegister(REG_CURRENT, val);

  /*
   * Reinterpret register value as a signed integer (two's complement).
   * Multiply with the value of the least significant bit.
   */
  return int32_t(*reinterpret_cast<int16_t*>(&val)) * this->current_lsb_uA;
}

uint16_t
Driver::readCalibration()
{
  uint16_t val = 0;
  this->readRegister(REG_CALIBRATION, val);
  return (val & MASK_CALIBRATION);
}

uint8_t
Driver::reset()
{
  const msg_t res = this->writeRegister(REG_CONFIGURATION, MASK_RESET);
  return (res? ERROR : SUCCESS);
}

msg_t
Driver::main(void)
{
  while (!this->shouldTerminate())
  {
    this->update();
    //this->eventSource.broadcastFlags(0);
    this->sleep(MS2ST(1000));
    //this->waitAnyEventTimeout(ALL_EVENTS, MS2ST(1000));
  }

  return RDY_OK;
}

msg_t
Driver::readRegister(const RegisterAddress reg, uint16_t& dst)
{
  uint8_t buffer[2];
  this->tx_params.txbuf = reinterpret_cast<const uint8_t*>(&reg);
  this->tx_params.txbytes = 1;
  this->tx_params.rxbuf = buffer;
  this->tx_params.rxbytes = 2;

  this->i2c_driver->acquireBus();
  const msg_t res = this->i2c_driver->masterTransmit(&this->tx_params);
  this->i2c_driver->releaseBus();

  if (!res) {
    dst = (buffer[0] << 8) | buffer[1];
    if (reg == REG_CALIBRATION)
      dst &= MASK_CALIBRATION;
#ifndef NDEBUG
  } else {
    chprintf((BaseSequentialStream*) &global.sercanmux1, "%s(%d): ERROR: i2c transmit failed (%d | 0x%08X)\n", __FILE__ , __LINE__ , res, this->i2c_driver->getErrors());
#endif
  }

  return res;
}

msg_t
Driver::writeRegister(const RegisterAddress reg, const uint16_t& val)
{
  const uint8_t buffer[3] = {reg,
                             static_cast<uint8_t>((val & 0xFF00u) >> 8),
                             static_cast<uint8_t>(val & 0x00FFu)};
  this->tx_params.txbuf = buffer;
  this->tx_params.txbytes = 3;
  this->tx_params.rxbytes = 0;

  this->i2c_driver->acquireBus();
  const msg_t res = this->i2c_driver->masterTransmit(&this->tx_params);
  this->i2c_driver->releaseBus();

#ifndef NDEBUG
  if (res) {
    chprintf((BaseSequentialStream*) &global.sercanmux1, "%s(%d): ERROR: i2c transmit failed (%d | 0x%08X)\n", __FILE__ , __LINE__ , res, this->i2c_driver->getErrors());
  }
#endif

  return res;
}

