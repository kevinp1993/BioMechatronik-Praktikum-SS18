#include <ch.hpp>
#include <hal.h>
#include <chdebug.h>
#include <chprintf.h>

#include <amiro/bus/i2c/I2CParams.hpp>
#include <amiro/bus/i2c/I2CDriver.hpp>
#include <amiro/proximity/vcnl4020.hpp>
#include <amiro/Constants.h>

using namespace chibios_rt;

namespace amiro {

VCNL4020::
~VCNL4020() {

}

VCNL4020::
VCNL4020(I2CDriver *driver, const VCNL4020Config *config) :
  BaseStaticThread<256>(),
  driver(driver),
  config(config),
  ambient(0x0000u),
  proximity(0x0000u),
  proximityOffset(0x0000u) {

  this->tx_params.addr = VCNL4020::SLA;

}

uint16_t
VCNL4020::
getAmbientLight() {

  return this->ambient;
}

void
VCNL4020::
setProximityOffset(uint16_t offset) {

  this->proximityOffset = offset;
}

uint16_t
VCNL4020::
getProximityOffset() {

  return this->proximityOffset;
}

EvtSource*
VCNL4020::
getEventSource() {

  return &this->eventSource;
}

uint16_t
VCNL4020::
getProximity() {

  return this->proximity;
}

uint16_t
VCNL4020::
getProximityScaledWoOffset() {

  if (this->proximity <= this->proximityOffset)
    return 0;

  // Scale factor for the offset-less proximity value, so that we can reach full-scale
  const float scaleFactor = float(0xFFFFu) / float((0xFFFFu - this->proximityOffset));

  return uint16_t(float(this->proximity - this->proximityOffset) * scaleFactor);
}

msg_t
VCNL4020::
main(void) {

  I2CDriver *drv = this->driver;
  msg_t res;

  this->setName("Vcnl4020");

  drv->acquireBus();

  /* exit if writing configuration fails */
  res = this->writeIRConf();

  drv->releaseBus();

  if (res)
    return RDY_RESET;

  //TODO Read calibration from eeprom

  while (!this->shouldTerminate()) {

    drv->acquireBus();

    this->readIntensities();
    drv->releaseBus();

    this->eventSource.broadcastFlags(0);

    this->waitAnyEventTimeout(ALL_EVENTS, CAN::UPDATE_PERIOD);
  }

  return RDY_OK;
}

msg_t
VCNL4020::
writeIRConf() {

  msg_t res;
  const VCNL4020Config *cfg = this->config;

  uint8_t buffer[4];
  this->tx_params.txbuf = buffer;
  this->tx_params.txbytes = 4;
  this->tx_params.rxbytes = 0;

  buffer[0] = offsetof(VCNL4020::registers, proximity_rate);
  buffer[1] = cfg->proximity_rate;
  buffer[2] = cfg->ir_led_current_mA > 200 ? 20u : cfg->ir_led_current_mA / 10u;
  buffer[3] = cfg->ambient_parameter;

  res = this->driver->masterTransmit(&this->tx_params);

  buffer[0] = offsetof(VCNL4020::registers, command);
  buffer[1] = cfg->command;

  this->tx_params.txbytes = 2;

  if (!res)
    res = this->driver->masterTransmit(&this->tx_params);

  return res;
}

msg_t
VCNL4020::
readIntensities() {

  msg_t res;
  uint8_t buffer[4];
  uint8_t reg = offsetof(VCNL4020::registers, ambient_result);
  this->tx_params.txbuf = &reg;
  this->tx_params.txbytes = 1;
  this->tx_params.rxbuf = buffer;
  this->tx_params.rxbytes = 4;

  res = this->driver->masterTransmit(&this->tx_params);

  if (!res) {

    /* update internal values */
    this->ambient = (buffer[0] << 8) | buffer[1];
    this->proximity = (buffer[2] << 8) | buffer[3];

  }

  return res;

}

uint8_t
VCNL4020::
getCheck(void) {

  int8_t resTx;
  uint8_t rxBuffer[2]; /** This is a bug workaround: 1 byte is impossible to read on STM32F1*/
  const uint8_t txBuffer = uint8_t(offsetof(VCNL4020::registers, revision));

  // Use own datastructure, so that there is now faulty
  // behaviour with the other threads using it
  I2CTxParams tx_params = this->tx_params;
  tx_params.txbuf = &txBuffer;
  tx_params.txbytes = 1;
  tx_params.rxbytes = 2; /** This is a bug workaround: 1 byte is impossible to read on STM32F1*/
  tx_params.rxbuf = rxBuffer;

  // Read the data
  resTx = this->driver->masterTransmit(&tx_params);
  // Failed to transmit
  if (resTx != RDY_OK || VCNL4020::PRODUCT_ID_REVISION != rxBuffer[0]) {
    return VCNL4020::CHECK_FAIL;
  } else {
    return VCNL4020::CHECK_OK;
  }

}

uint8_t
VCNL4020::
calibrate() {

  uint16_t proximityFloorMeanValue;

  // Get the offset
  msg_t res = calibrateOffset(proximityFloorMeanValue);
  if (res == CALIB_OK) {
    this->proximityOffset = proximityFloorMeanValue;
    //TODO Write value to eeprom
    return res;
  } else {
    return res;
  }

}

uint8_t
VCNL4020::
calibrateOffset(uint16_t &proximityFloorMeanValue) {

  uint32_t tmpProximityFloorMeanValue;

  // Register an event listener, to receive updates
  chibios_rt::EvtListener eventTimerEvtListener;
  chibios_rt::EvtSource *vcnlEvtSource;
  vcnlEvtSource = reinterpret_cast<EvtSource *>(this->getEventSource());
  vcnlEvtSource->registerOne(&eventTimerEvtListener,0);

  // Get the first values
  tmpProximityFloorMeanValue = this->getProximity();

  // Get the others with a floating mean
  const uint32_t maxValues = 20;
  for (uint32_t idxMean = 2; idxMean <= maxValues; ++idxMean) {
    this->waitOneEvent(ALL_EVENTS); /* Wait for a new update */
    tmpProximityFloorMeanValue = (((idxMean-1) * tmpProximityFloorMeanValue) + uint32_t(this->getProximity())) / idxMean;
  }

  vcnlEvtSource->unregister(&eventTimerEvtListener);

  // Cast the calculated offset to the temporary member variable
  proximityFloorMeanValue = uint16_t(tmpProximityFloorMeanValue);

  return CALIB_OK;
}

} /* amiro */
