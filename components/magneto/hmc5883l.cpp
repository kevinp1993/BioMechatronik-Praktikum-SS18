#include <ch.hpp>
#include <hal.h>
#include <chdebug.h>

#include <amiro/bus/i2c/I2CParams.hpp>
#include <amiro/bus/i2c/I2CDriver.hpp>
#include <amiro/magneto/hmc5883l.hpp>
#include <amiro/Constants.h>

using namespace chibios_rt;

namespace amiro {

HMC5883L::~HMC5883L() {

}

HMC5883L::HMC5883L(I2CDriver *driver, const HMC5883LConfig *config)
    : BaseStaticThread<256>(),
      driver(driver),
      config(config) {

  this->txParams.addr = HMC5883L::SLA;
}

EvtSource*
HMC5883L::
getEventSource() {

  return &this->eventSource;
}

msg_t
HMC5883L::
main(void) {

  msg_t res;

  this->setName("Hmc5883l");

  /* exit if writing configuration fails */
  res = this->writeConf();

  if (res) {
    return RDY_RESET;
  }

  while (!this->shouldTerminate()) {

    this->updateSensorData();

    this->eventSource.broadcastFlags(0);

    this->waitAnyEventTimeout(ALL_EVENTS, CAN::UPDATE_PERIOD);
  }

  return RDY_OK;
}

int16_t
HMC5883L::
getMagnetization(const uint8_t axis) {
  int16_t returnBuffer;
  chSysLock();
  returnBuffer = this->data[axis];
  chSysUnlock();
  return returnBuffer;

}

int32_t
HMC5883L::
getMagnetizationGauss(const uint8_t axis) {

  int16_t magLsbBuffer = this->getMagnetization(axis);
  int32_t returnBuffer = 0;

  // The gains are taken from table 9 of the manual
  switch (this->config->ctrlB & GN_7_GA) {
    case HMC5883L::GN_0_GA:
      returnBuffer = int32_t(0.73 * 1000) * int32_t(magLsbBuffer);
      break;
    case HMC5883L::GN_1_GA:
      returnBuffer = int32_t(0.92 * 1000) * int32_t(magLsbBuffer);
      break;
    case HMC5883L::GN_2_GA:
      returnBuffer = int32_t(1.22 * 1000) * int32_t(magLsbBuffer);
      break;
    case HMC5883L::GN_3_GA:
      returnBuffer = int32_t(1.52 * 1000) * int32_t(magLsbBuffer);
      break;
    case HMC5883L::GN_4_GA:
      returnBuffer = int32_t(2.27 * 1000) * int32_t(magLsbBuffer);
      break;
    case HMC5883L::GN_5_GA:
      returnBuffer = int32_t(2.56 * 1000) * int32_t(magLsbBuffer);
      break;
    case HMC5883L::GN_6_GA:
      returnBuffer = int32_t(3.03 * 1000) * int32_t(magLsbBuffer);
      break;
    case HMC5883L::GN_7_GA:
      returnBuffer = int32_t(4.35 * 1000) * int32_t(magLsbBuffer);
      break;
  }

  return returnBuffer;

}

void
HMC5883L::
updateSensorData() {

  const uint8_t sizeOfRxData =  offsetof(HMC5883L::registers, yLsb)
                              - offsetof(HMC5883L::registers, xMsb)
                              + 1;

  uint8_t rxBuffer[sizeOfRxData];

  /*Address of data register*/
  uint8_t txBuffer = offsetof(HMC5883L::registers, xMsb); /* Address of XMSB */

  // Define the RXTX structure
  this->txParams.txbuf = &txBuffer;
  this->txParams.txbytes = sizeof(txBuffer);
  this->txParams.rxbuf = rxBuffer;
  this->txParams.rxbytes = sizeof(rxBuffer);

  this->driver->acquireBus();
  this->driver->masterTransmit(&this->txParams);
  this->driver->releaseBus();

  // Copy the bytes in the right order
  chSysLock(); // TODO: check if this is really necessary b/c it inserts a DMB!!
  data[HMC5883L::AXIS_X] = (rxBuffer[0] << 8) + rxBuffer[1];
  data[HMC5883L::AXIS_Z] = (rxBuffer[2] << 8) + rxBuffer[3];
  data[HMC5883L::AXIS_Y] = (rxBuffer[4] << 8) + rxBuffer[5];
  chSysUnlock();
}

uint8_t HMC5883L::getCheck() {

  I2CTxParams txStructure;
  msg_t res;
  const uint8_t txBuffer = offsetof(HMC5883L::registers, idA);
  const uint8_t sizeOfRxData = offsetof(HMC5883L::registers, idC) - offsetof(HMC5883L::registers, idA) + 1;
  uint8_t rxBuffer[sizeOfRxData];

  txStructure.addr = SLA;
  txStructure.txbuf = &txBuffer;
  txStructure.txbytes = sizeof(txBuffer);
  txStructure.rxbuf = rxBuffer;
  txStructure.rxbytes = sizeof(rxBuffer);

  // get the identifier
  this->driver->acquireBus();
  res = this->driver->masterTransmit(&txStructure);
  this->driver->releaseBus();
  // Check
  if (rxBuffer[0] == ID_IRA && rxBuffer[1] == ID_IRB && rxBuffer[2] == ID_IRC && res == RDY_OK) {
    return HMC5883L::CHECK_OK;
  } else {
    return HMC5883L::CHECK_FAIL;
  }

}


msg_t
HMC5883L::
writeConf() {

  uint8_t txBuffer[4];
  msg_t res;

  // write control config
  txBuffer[0] = offsetof(HMC5883L::registers, ctrlA);  /* Address of ctrlA */
  txBuffer[1] = this->config->ctrlA;
  txBuffer[2] = this->config->ctrlB;
  txBuffer[3] = this->config->mode;

  this->txParams.txbuf = txBuffer;
  this->txParams.txbytes = sizeof(txBuffer);
  this->txParams.rxbytes = 0;

  this->driver->acquireBus();
  res = this->driver->masterTransmit(&this->txParams);
  this->driver->releaseBus();

  return res;

}

} /* amiro */
