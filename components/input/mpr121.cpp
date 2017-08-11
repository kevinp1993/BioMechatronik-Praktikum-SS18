#include <amiro/bus/i2c/I2CDriver.hpp>
#include <amiro/input/mpr121.hpp>
#include <amiro/Constants.h>

namespace amiro {

MPR121::
MPR121(I2CDriver *driver, const uint8_t master_id) :
  BaseStaticThread<256>(),
  driver(driver),
  master_id(master_id),
  button_state(0x0000u) {

  chDbgCheck(master_id <= 0x03u, "MPR121 ctor master_id");
  this->tx_params.addr = MPR121::SLA | this->master_id;

}

MPR121::
~MPR121() {

}

msg_t
MPR121::
updateButtonData() {

  msg_t res;
  const uint8_t offset_touch = offsetof(MPR121::registers, touch_status);
  const uint8_t offset_electrode = offsetof(MPR121::registers, ele_filt_data);
  const uint8_t offset_baseline = offsetof(MPR121::registers, ele_baseline);
  uint8_t buffer[offsetof(MPR121::registers, mhd_rising) - offset_touch];
  uint8_t *tmp = buffer;
  uint8_t reg = offset_touch;
  uint8_t i;
  this->tx_params.txbuf = &reg;
  this->tx_params.txbytes = 1;
  this->tx_params.rxbuf = buffer;
  this->tx_params.rxbytes = sizeof(buffer);

  res = this->driver->masterTransmit(&this->tx_params);

  if (!res) {
    tmp = buffer + offset_touch;
    this->button_state = (*(tmp + 1) << 8) | *tmp;
    tmp = buffer + offset_electrode;
    for (i = 0x00; i < 13; i++) {
      this->electrode_data[i] = (*(tmp + 1) << 8) | *tmp;
      tmp += 2;
    }
    tmp = buffer + offset_baseline;
    for (i = 0x00; i < 13; i++)
      this->baseline_data[i] = *tmp++;

  }

  return res;

}

msg_t
MPR121::
softReset() {

  msg_t res;
  uint8_t buffer[2];

  buffer[0] = offsetof(MPR121::registers, soft_reset);
  buffer[1] = SOFT_RST_MAGIC;

  this->tx_params.txbuf = buffer;
  this->tx_params.txbytes = 2;
  this->tx_params.rxbytes = 0;

  res = this->driver->masterTransmit(&this->tx_params);

  if (res)
    return res;

  buffer[0] = offsetof(MPR121::registers, ele_config);
  buffer[1] = 0x00u; /* ele_config: make sure we are in stop mode */

  return this->driver->masterTransmit(&this->tx_params);

}

msg_t
MPR121::
writeConfig(const MPR121Config *cfg) {

  msg_t res;
  uint8_t buffer[6];

  buffer[0] = offsetof(MPR121::registers, auto_cfg_ctrl);
  buffer[1] = (cfg->auto_config & 0x00FFu);
  buffer[2] = (cfg->auto_config & 0xFF00u) >> 8;
  buffer[3] = cfg->up_side_limit;
  buffer[4] = cfg->low_side_limit;
  buffer[5] = cfg->target_level;

  this->tx_params.txbuf = buffer;
  this->tx_params.txbytes = 6;
  this->tx_params.rxbytes = 0;

  res = this->driver->masterTransmit(&this->tx_params);

  if (res)
    return res;

  buffer[0] = offsetof(MPR121::registers, cdc_config);
  buffer[1] = (cfg->global_config & 0x00FFu);
  buffer[2] = (cfg->global_config & 0xFF00u) >> 8;
  buffer[3] = cfg->ele_config;

  this->tx_params.txbytes = 4;

  return this->driver->masterTransmit(&this->tx_params);

}

uint16_t
MPR121::
getButtonStatus() {

  return this->button_state;

}

uint8_t
MPR121::
getButtonStatus(uint8_t ix) {

  return this->button_state & (1u << ix);

}

uint8_t
MPR121::
getBaselineData(uint8_t ix) {

  return this->baseline_data[ix];

}

uint16_t
MPR121::
getElectrodeData(uint8_t ix) {

  return this->electrode_data[ix];

}

/**
 * Main Sensor Thread.
 * Will not check whether sensor was actually configured prior to running.
 * @note   You should configure the sensor while stopped, then start.
 * @return RDY_OK on exit.
 */
msg_t
MPR121::
main(void) {

  I2CDriver *drv = this->driver;

  this->setName("MPR121");

  while (!this->shouldTerminate()) {

    drv->acquireBus();
    this->updateButtonData();
    drv->releaseBus();

    this->eventSource.broadcastFlags(0);

    this->waitAnyEventTimeout(ALL_EVENTS, CAN::UPDATE_PERIOD);
  }

  return RDY_OK;
}

/**
 * Configure the MPR121 touch sensor.
 * Will perform a soft reset and then apply the configuration \p cfg.
 * @note      Call only when sensor is stopped.
 * @param cfg The new configuration.
 * @return    RDY_OK if everything went well. Appropriate error otherwise.
 */
msg_t
MPR121::
configure(const MPR121Config *cfg) {

  I2CDriver *drv = this->driver;
  msg_t res;

  drv->acquireBus();

  // do a soft reset, in case config changes config mode
  // (auto config vs. configure all sensors individually)
  res = this->softReset();

  if (res == RDY_OK) {

    res = this->writeConfig(cfg);

  }

  drv->releaseBus();

  return res;
}


} /* amiro */
