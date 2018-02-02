#include <ch.hpp>
#include <hal.h>
#include <chprintf.h>
#include <string.h>

#include <amiro/util/util.h>
#include <amiro/bus/spi/HWSPIDriver.hpp>
#include <amiro/gyro/l3g4200d.hpp>
#include <amiro/Constants.h>
#include <global.hpp>

using namespace chibios_rt;

extern Global global;

namespace amiro {

L3G4200D::L3G4200D(HWSPIDriver *driver)
    : driver(driver),
      udpsPerTic(175000),
      period_us(100000) {
  this->period_ms = this->period_us * 1e-3;
  this->period_st = US2ST(this->period_us);
}

L3G4200D::~L3G4200D() {

}

chibios_rt::EvtSource*
L3G4200D::getEventSource() {
  return &this->eventSource;
}

msg_t L3G4200D::main() {
  systime_t time = System::getTime();
  this->setName("l3g4200d");

  while (!this->shouldTerminate()) {
    time += this->period_st;

    updateSensorData();
    calcAngular();

    this->eventSource.broadcastFlags(1);

    if (time >= System::getTime()) {
      chThdSleepUntil(time);
    } else {
      chprintf((BaseSequentialStream*) &global.sercanmux1, "WARNING l3g4200d: Unable to keep track\r\n");
    }
  }
  return RDY_OK;
}

int16_t
L3G4200D::
getAngularRate(const uint8_t axis) {
  return this->angularRate[axis];
}

int32_t
L3G4200D::
getAngularRate_udps(const uint8_t axis) {
  return int32_t(this->angularRate[axis]) * this->udpsPerTic;
}

int32_t
L3G4200D::
getAngular(const uint8_t axis) {
  return this->angular[axis];
}

int32_t
L3G4200D::
getAngular_ud(const uint8_t axis) {
  const int32_t angularRate_mdps = this->getAngularRate_udps(axis) * 1e-3;
  return angularRate_mdps * (this->integrationTic * this->period_ms);
}

void
L3G4200D::
calcAngular() {
  // Need to check for overflow!
  ++this->integrationTic;
  chSysLock();
  this->angular[L3G4200D::AXIS_X] += int32_t(this->angularRate[L3G4200D::AXIS_X]);
  this->angular[L3G4200D::AXIS_Y] += int32_t(this->angularRate[L3G4200D::AXIS_Y]);
  this->angular[L3G4200D::AXIS_Z] += int32_t(this->angularRate[L3G4200D::AXIS_Z]);
  chSysUnlock();
}

// TODO: Outsource, so that everyone who needs this has an own instance of the integrator
void
L3G4200D::
angularReset() {
  this->angular[L3G4200D::AXIS_X] = 0;
  this->angular[L3G4200D::AXIS_Y] = 0;
  this->angular[L3G4200D::AXIS_Z] = 0;
  this->integrationTic = 0;
}


void L3G4200D::updateSensorData() {

  const size_t buffer_size = offsetof(L3G4200D::registers, OUT_Z)
                             - offsetof(L3G4200D::registers, STATUS_REG)
                             + MEMBER_SIZE(L3G4200D::registers, OUT_Z)
                             + 1; /* addressing */
  uint8_t buffer[buffer_size];
  uint8_t sreg;

  /*Address of first data register*/
  memset(buffer, 0xFF, sizeof(buffer));
  buffer[0] = offsetof(L3G4200D::registers, STATUS_REG) | L3G4200D::SPI_READ | L3G4200D::SPI_MULT;

  this->driver->exchange(buffer, buffer, buffer_size);

  // assemble data
  sreg = buffer[1];

  chSysLock();
  if (sreg & L3G4200D::XDA)
    this->angularRate[L3G4200D::AXIS_X] = (buffer[3] << 8) + buffer[2];

  if (sreg & L3G4200D::YDA)
    this->angularRate[L3G4200D::AXIS_Y] = (buffer[5] << 8) + buffer[4];

  if (sreg & L3G4200D::ZDA)
    this->angularRate[L3G4200D::AXIS_Z] = (buffer[7] << 8) + buffer[6];
  chSysUnlock();
}

msg_t L3G4200D::configure(const L3G4200DConfig *config) {

  const size_t ctrl_reg_size = offsetof(L3G4200D::registers, CTRL_REG5)
                               - offsetof(L3G4200D::registers, CTRL_REG1)
                               + MEMBER_SIZE(L3G4200D::registers, CTRL_REG5)
                               + 1; /* addressing */

  const size_t buffer_size = ctrl_reg_size;

  uint8_t buffer[buffer_size];

  // write control config
  // this might be three-wire so we need to send ones
  memset(buffer, 0xFFu, buffer_size);
  buffer[0] = offsetof(L3G4200D::registers, CTRL_REG1) | L3G4200D::SPI_MULT | L3G4200D::SPI_WRITE;
  buffer[1] = config->ctrl1;
  buffer[2] = config->ctrl2;
  buffer[3] = config->ctrl3;
  buffer[4] = config->ctrl4;
  buffer[5] = config->ctrl5;
  this->driver->write(buffer, 6);

  // Handle the new update time
  switch(config->ctrl1 & L3G4200D::DR_MASK) {
    case L3G4200D::DR_100_HZ: this->period_us = 10000; break;
    case L3G4200D::DR_200_HZ: this->period_us =  5000; break;
    case L3G4200D::DR_400_HZ: this->period_us =  2500; break;
    case L3G4200D::DR_800_HZ: this->period_us =  1250; break;
  }
  this->period_st = US2ST(this->period_us);
  this->period_ms = this->period_us * 1e-3;

  // Handle the new full scale
  switch(config->ctrl1 & L3G4200D::FS_MASK) {
    case L3G4200D::FS_250_DPS:  this->udpsPerTic =  8750; break;
    case L3G4200D::FS_500_DPS:  this->udpsPerTic = 17500; break;
    case L3G4200D::FS_2000_DPS: this->udpsPerTic = 70000; break;
  }

  // Reset the integration
  this->angularReset();

  return RDY_OK;

}

uint8_t L3G4200D::getCheck() {

  const size_t buffer_size = 1 /* addressing */
  + 1; /* who am i */
  uint8_t buffer[buffer_size];

  // Exchange the data with the L3G4200D gyroscope
  // Specify the adress and the mode
  buffer[0] = offsetof(L3G4200D::registers, WHO_AM_I) | L3G4200D::SPI_READ;
  this->driver->exchange(buffer, buffer, buffer_size);
  // Check
  if (buffer[1] == L3G4200D::L3G4200D_ID) {
    return L3G4200D::CHECK_OK;
  } else {
    return L3G4200D::CHECK_FAIL;
  }

}

} /* amiro */
