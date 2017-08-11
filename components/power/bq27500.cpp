#include <amiro/power/bq27500.hpp>

#include <ch.hpp>
#include <chprintf.h>
#include <global.hpp>

#include <cstring>

using namespace chibios_rt;
using namespace amiro;
using namespace BQ27500;

extern Global global;

Driver::Driver(I2CDriver &i2c_driver, const GPIO_TypeDef &batgd_pingrp, const uint8_t batgd_pin, const GPIO_TypeDef &batlow_pingrp, const uint8_t batlow_pin) :
  BaseSensor<BQ27500::InitData,BQ27500::CalibData>(), i2c_driver(&i2c_driver), tx_params({I2C_ADDR, NULL, 0, NULL, 0}),
  batgd_pingrp(&batgd_pingrp), batgd_pin(batgd_pin), batlow_pingrp(&batlow_pingrp), batlow_pin(batlow_pin)
{}

Driver::~Driver()
{}

chibios_rt::EvtSource*
Driver::getEventSource()
{
  return &this->eventSource;
}

msg_t
Driver::init(InitData* initialization_data)
{
  chprintf((BaseSequentialStream*) &global.sercanmux1, "%s(%d): TODO\n", __FILE__, __LINE__);
  return NOT_IMPLEMENTED;
}

msg_t
Driver::update()
{
  msg_t res = SUCCESS;
  res |= this->stdCommand(STD_CMD_TimeToEmpty, this->status.minutes_to_empty);
  res |= this->stdCommand(STD_CMD_TimeToFull, this->status.minutes_to_full);
  res |= this->stdCommand(STD_CMD_AveragePower, this->status.average_power_mW);
  uint16_t tmp;
  res |= this->stdCommand(STD_CMD_StateOfCharge, tmp);
  this->status.state_of_charge = tmp;
  return (res == SUCCESS)? SUCCESS : ERROR;
}

msg_t
Driver::wakeup()
{
  const msg_t res = this->subCommand(SUB_CMD_CLEAR_HIBERNATE);
  return (res? ERROR : SUCCESS);
}

msg_t
Driver::hibernate()
{
  const msg_t res = this->subCommand(SUB_CMD_SET_HIBERNATE);
  return (res? ERROR : SUCCESS);
}

#ifndef AMIRO_NCALIBRATION
msg_t
Driver::calibration(CalibData* calibration_data)
{
  DataFlashBlock block;
  this->readDataFlashBlock(block, CONFIGURATION_Safety);

  chprintf((BaseSequentialStream*) &global.sercanmux1, "%s(%d):\n", __FILE__, __LINE__);
  for(uint8_t i = 0; i < 32; ++i) {
    chprintf((BaseSequentialStream*) &global.sercanmux1, "%02X ", block.content.data[i]);
  }
  chprintf((BaseSequentialStream*) &global.sercanmux1, "\n");
  chprintf((BaseSequentialStream*) &global.sercanmux1, "%02X\n", block.content.checksum);

  chprintf((BaseSequentialStream*) &global.sercanmux1, "%s(%d): TODO\n", __FILE__, __LINE__);
  return NOT_IMPLEMENTED;
}
#endif

#ifndef AMIRO_NSELFTEST
msg_t
Driver::selftest()
{
  uint16_t val = 0;
  Version version;

  // read hardware version
  version.value = 0;
  if (this->subCommand(SUB_CMD_HW_VERSION, &version.value))
  {
    return ST_FAIL_READ_HW_VERSION;
  }
  chprintf((BaseSequentialStream*) &global.sercanmux1, "hardware version: %X%X-%X%X (0x%04X)\n", version.content.major_high, version.content.major_low, version.content.minor_high, version.content.minor_low, version.value);

  // read firmware version
  version.value = 0;
  if (this->subCommand(SUB_CMD_FW_VERSION, &version.value))
  {
    return ST_FAIL_READ_FW_VERSION;
  }
  chprintf((BaseSequentialStream*) &global.sercanmux1, "firmware version: %X%X-%X%X (0x%04X)\n", version.content.major_high, version.content.major_low, version.content.minor_high, version.content.minor_low, version.value);

  // read device name
  uint8_t name_length = 0;
  if (this->extCommand(EXT_CMD_DNAMELEN, EXT_CMD_READ, &name_length))
  {
    return ST_FAIL_READ_DEVICENAMELENGTH;
  }
  char name_buffer[9]; // maximum name length is 8
  if (this->extCommand(EXT_CMD_DNAME, EXT_CMD_READ, (uint8_t*)name_buffer, name_length))
  {
    return ST_FAIL_READ_DEVICENAME;
  }
  name_buffer[name_length] = '\0';
  chprintf((BaseSequentialStream*) &global.sercanmux1, "device name: %s (%u characters)\n", name_buffer, name_length);

  // read the current flags
  Flags flags;
  if (this->stdCommand(STD_CMD_FLAGS, flags.value))
  {
    return ST_FAIL_READ_FLAGS;
  }
  chprintf((BaseSequentialStream*) &global.sercanmux1, "flags: 0x%04X\n", flags.value);
  chprintf((BaseSequentialStream*) &global.sercanmux1, "  OTC     : %u\n", flags.content.otc);
  chprintf((BaseSequentialStream*) &global.sercanmux1, "  OTD     : %u\n", flags.content.otd);
  chprintf((BaseSequentialStream*) &global.sercanmux1, "  CHG_INH : %u\n", flags.content.chg_inh);
  chprintf((BaseSequentialStream*) &global.sercanmux1, "  XCHG    : %u\n", flags.content.xchg);
  chprintf((BaseSequentialStream*) &global.sercanmux1, "  FC      : %u\n", flags.content.fc);
  chprintf((BaseSequentialStream*) &global.sercanmux1, "  CHG     : %u\n", flags.content.chg);
  chprintf((BaseSequentialStream*) &global.sercanmux1, "  OCV_GD  : %u\n", flags.content.ocv_gd);
  chprintf((BaseSequentialStream*) &global.sercanmux1, "  WAIT_ID : %u\n", flags.content.wait_id);
  chprintf((BaseSequentialStream*) &global.sercanmux1, "  BAT_DET : %u\n", flags.content.bat_det);
  chprintf((BaseSequentialStream*) &global.sercanmux1, "  SOC1    : %u\n", flags.content.soc1);
  chprintf((BaseSequentialStream*) &global.sercanmux1, "  SOCF    : %u\n", flags.content.socf);
  chprintf((BaseSequentialStream*) &global.sercanmux1, "  DSG     : %u\n", flags.content.dsg);

  // read the current controller status
  ControlStatus ctrl_status;
  if (this->subCommand(SUB_CMD_CONTROL_STATUS, &ctrl_status.value))
  {
    return ST_FAIL_READ_STATUS;
  }
  chprintf((BaseSequentialStream*) &global.sercanmux1, "control status: 0x%04X\n", ctrl_status.value);
  chprintf((BaseSequentialStream*) &global.sercanmux1, "  FAS       : %u\n", ctrl_status.content.fas);
  chprintf((BaseSequentialStream*) &global.sercanmux1, "  SS        : %u\n", ctrl_status.content.ss);
  chprintf((BaseSequentialStream*) &global.sercanmux1, "  CSV       : %u\n", ctrl_status.content.csv);
  chprintf((BaseSequentialStream*) &global.sercanmux1, "  CSA       : %u\n", ctrl_status.content.cca);
  chprintf((BaseSequentialStream*) &global.sercanmux1, "  BCA       : %u\n", ctrl_status.content.bca);
  chprintf((BaseSequentialStream*) &global.sercanmux1, "  HIBERNATE : %u\n", ctrl_status.content.hibernate);
  chprintf((BaseSequentialStream*) &global.sercanmux1, "  SNOOZE    : %u\n", ctrl_status.content.snooze);
  chprintf((BaseSequentialStream*) &global.sercanmux1, "  SLEEP     : %u\n", ctrl_status.content.sleep);
  chprintf((BaseSequentialStream*) &global.sercanmux1, "  LDMD      : %u\n", ctrl_status.content.ldmd);
  chprintf((BaseSequentialStream*) &global.sercanmux1, "  RUP_DIS   : %u\n", ctrl_status.content.rup_dis);
  chprintf((BaseSequentialStream*) &global.sercanmux1, "  VOK       : %u\n", ctrl_status.content.vok);
  chprintf((BaseSequentialStream*) &global.sercanmux1, "  QEN       : %u\n", ctrl_status.content.qen);

  // if no battery was detected, abort
  if (!flags.content.bat_det)
  {
    return ST_ABORT_NO_BAT;
  }

  // read the BATGD_N pin
  chprintf((BaseSequentialStream*) &global.sercanmux1, "battery good: %s\n", (this->isBatteryGood()? "yes" : "no"));

  // read temperature
  if (this->stdCommand(STD_CMD_TEMP, val))
  {
    return ST_FAIL_READ_TEMP;
  }
  chprintf((BaseSequentialStream*) &global.sercanmux1, "temperature: %fK (%fC)\n", float(val)/10.0f, float(val)/10.0f - 273.15f);

  // read the full available capacity
  if (this->stdCommand(STD_CMD_FAC, val))
  {
    return ST_FAIL_READ_FAC;
  }
  chprintf((BaseSequentialStream*) &global.sercanmux1, "full available capacity: %umAh\n", val);

  // read the full charge capacity
  if (this->stdCommand(STD_CMD_FCC, val))
  {
    return ST_FAIL_READ_FCC;
  }
  chprintf((BaseSequentialStream*) &global.sercanmux1, "full charge capacity: %umAh\n", val);

  // read the remaining capacity
  if (this->stdCommand(STD_CMD_RM, val))
  {
    return ST_FAIL_READ_RM;
  }
  chprintf((BaseSequentialStream*) &global.sercanmux1, "remaining capacity capacity: %umAh\n", val);

  // read the state of charge
  if (this->stdCommand(STD_CMD_SOC, val))
  {
    return ST_FAIL_READ_SOC;
  }
  chprintf((BaseSequentialStream*) &global.sercanmux1, "state of charge: %3u%%\n", val);

  // read voltage
  if (this->stdCommand(STD_CMD_VOLT, val))
  {
    return ST_FAIL_READ_VOLT;
  }
  chprintf((BaseSequentialStream*) &global.sercanmux1, "voltage: %umV\n", val);

  // read average current
  if (this->stdCommand(STD_CMD_AI, val))
  {
    return ST_FAIL_READ_AI;
  }
  chprintf((BaseSequentialStream*) &global.sercanmux1, "average current: %dmA\n", *reinterpret_cast<int8_t*>(&val));

  // read average power
  if (this->stdCommand(STD_CMD_AP, val))
  {
    return ST_FAIL_READ_AP;
  }
  chprintf((BaseSequentialStream*) &global.sercanmux1, "average power: %dmW\n", *reinterpret_cast<int8_t*>(&val));

  // read the BATLOW pin
  chprintf((BaseSequentialStream*) &global.sercanmux1, "battery low: %s\n", (this->isBatteryLow()? "yes" : "no"));

  // read the time to empty
  if (this->stdCommand(STD_CMD_TTE, val))
  {
    return ST_FAIL_READ_TTE;
  }
  chprintf((BaseSequentialStream*) &global.sercanmux1, "time to empty: ");
  if (uint16_t(~val)) {
    chprintf((BaseSequentialStream*) &global.sercanmux1, "%u minutes", val);
  } else {
    chprintf((BaseSequentialStream*) &global.sercanmux1, "(not discharging)");
  }
  chprintf((BaseSequentialStream*) &global.sercanmux1, "\n");

  // read the time to full
  if (this->stdCommand(STD_CMD_TTF, val))
  {
    return ST_FAIL_READ_TTF;
  }
  chprintf((BaseSequentialStream*) &global.sercanmux1, "time to full: ");
  if (uint16_t(~val)) {
    chprintf((BaseSequentialStream*) &global.sercanmux1, "%u minutes", val);
  } else {
    chprintf((BaseSequentialStream*) &global.sercanmux1, "(not charging)");
  }
  chprintf((BaseSequentialStream*) &global.sercanmux1, "\n");

  return ST_OK;
}
#endif

bool
Driver::isBatteryGood() const
{
  return (palReadPad(this->batgd_pingrp, this->batgd_pin) == PAL_LOW);
}

bool
Driver::isBatteryLow() const
{
  return (palReadPad(this->batlow_pingrp, this->batlow_pin) == PAL_HIGH);
}

const Driver::UpdateData&
Driver::getStatus() const
{
  return this->status;
}

msg_t
Driver::main(void)
{
  while (!this->shouldTerminate())
  {
    this->update();
    this->eventSource.broadcastFlags(0);

    this->waitAnyEventTimeout(ALL_EVENTS, MS2ST(1000));
  }

  return RDY_OK;
}

msg_t
Driver::stdCommand(const StandardCommand cmd, uint16_t &dst)
{
  uint8_t buffer[2];
  this->tx_params.txbuf = reinterpret_cast<const uint8_t*>(&cmd);
  this->tx_params.txbytes = 1;
  this->tx_params.rxbuf = buffer;
  this->tx_params.rxbytes = 2;

  this->i2c_driver->acquireBus();
  const msg_t res = this->i2c_driver->masterTransmit(&this->tx_params);
  this->i2c_driver->releaseBus();

  if (!res) {
    dst = uint16_t((buffer[1] << 8) | buffer[0]);
#ifndef NDEBUG
  } else {
    chprintf((BaseSequentialStream*) &global.sercanmux1, "%s(%d): ERROR: i2c transmit failed (%d)\n", __FILE__ , __LINE__ , res);
#endif
  }

  return res;
}

msg_t
Driver::readName()
{
  uint8_t buffer[9];
  uint8_t reg = 0x62u;
  this->tx_params.txbuf = &reg;
  this->tx_params.txbytes = 1;
  this->tx_params.rxbuf = &buffer[0];
  this->tx_params.rxbytes = 8;

  this->i2c_driver->acquireBus();
  const msg_t res = this->i2c_driver->masterTransmit(&this->tx_params);
  this->i2c_driver->releaseBus();

  buffer[buffer[0] + 1] = '\0';

  chprintf((BaseSequentialStream*) &global.sercanmux1, "name: %u - %s\n", buffer[0], (char*)&buffer[1]);

  return res;
}

msg_t
Driver::subCommand(const ControlSubcommand cmd, uint16_t *dst)
{
  uint8_t buffer[3] = {STD_CMD_CNTL, uint8_t(cmd & 0x00FFu), uint8_t((cmd & 0xFF00u) >> 8)};
  this->tx_params.txbuf = buffer;
  this->tx_params.txbytes = 3;
  this->tx_params.rxbytes = 0;

  this->i2c_driver->acquireBus();
  msg_t res = this->i2c_driver->masterTransmit(&this->tx_params);
  this->i2c_driver->releaseBus();
  if (dst) {
    this->tx_params.txbytes = 1;

    this->tx_params.rxbuf = &buffer[1];
    this->tx_params.rxbytes = 2;

    BaseThread::sleep(US2ST(2));
    this->i2c_driver->acquireBus();
    res |= this->i2c_driver->masterTransmit(&this->tx_params);
    this->i2c_driver->releaseBus();
  }

#ifndef NDEBUG
  if (res) {
    chprintf((BaseSequentialStream*) &global.sercanmux1, "%s(%d): ERROR: i2c transmit failed (%d)\n", __FILE__ , __LINE__ , res);
  }
#endif

  if (dst && !res) {
    *dst = uint16_t((buffer[2] << 8) | buffer[1]);
  }

  return res;
}

msg_t
Driver::extCommand(const ExtendedCommand cmd, const ExtendedCommandAccess rw, uint8_t* buf, const uint8_t length, const uint8_t offset)
{
  if (!buf) {
#ifndef NDEBUG
    chprintf((BaseSequentialStream*) &global.sercanmux1, "%s(%d): ERROR: received NULL-pointer as buffer\n", __FILE__ , __LINE__);
#endif
    return ~RDY_OK;
  }
  if (rw != EXT_CMD_WRITE && rw != EXT_CMD_READ) {
#ifndef NDEBUG
    chprintf((BaseSequentialStream*) &global.sercanmux1, "%s(%d): ERROR: invalid access mode selected\n", __FILE__ , __LINE__);
#endif
    return ~RDY_OK;
  }
  if (length > 33) {
#ifndef NDEBUG
    chprintf((BaseSequentialStream*) &global.sercanmux1, "%s(%d): ERROR: length exceeds maximum of 33 bytes\n", __FILE__ , __LINE__);
#endif
    return ~RDY_OK;
  }

  uint8_t in_buffer[34];
  in_buffer[0] = cmd + offset;
  if (rw == EXT_CMD_WRITE) {
    strncpy((char*)&in_buffer[1], (char*)buf, length);
  }
  this->tx_params.txbuf = in_buffer;
  this->tx_params.txbytes = 1 + ((rw == EXT_CMD_WRITE)? length : 0);
  this->tx_params.rxbuf = (rw == EXT_CMD_READ)? buf : NULL;
  this->tx_params.rxbytes = (rw == EXT_CMD_READ)? length : 0;

  this->i2c_driver->acquireBus();
  const msg_t res = this->i2c_driver->masterTransmit(&this->tx_params);
  this->i2c_driver->releaseBus();

#ifndef NDEBUG
  if (res) {
    chprintf((BaseSequentialStream*) &global.sercanmux1, "%s(%d): ERROR: i2c transmit failed (%d)\n", __FILE__ , __LINE__ , res);
  }
#endif

  return res;
}

msg_t
Driver::readDataFlashBlock(DataFlashBlock &block, const DataFlashSubClassID sub_id, const uint8_t sub_block)
{
  block.raw[0] = sub_id;
  block.raw[1] = sub_block;
  msg_t res = this->extCommand(EXT_CMD_DFCLS, EXT_CMD_WRITE, &block.raw[0]);
  res |= this->extCommand(EXT_CMD_DFBLK, EXT_CMD_WRITE, &block.raw[1]);

  BaseThread::sleep(US2ST(1)); // Without this delay the whole block is shifted and the first byte is lost. TODO: investigate
  res |= this->extCommand(EXT_CMD_DFD, EXT_CMD_READ, block.raw, 33);

  return res;
}
