#include <string.h>

#include <amiro/util/util.h>
#include <amiro/bus/spi/HWSPIDriver.hpp>
#include <amiro/gyro/l3g4200d.hpp>

namespace amiro {

L3G4200D::L3G4200D(HWSPIDriver *driver)
    : driver(driver) {

}

L3G4200D::~L3G4200D() {

}

chibios_rt::EvtSource*
L3G4200D::getEventSource() {
  return &this->eventSource;
}

msg_t L3G4200D::main() {

  this->setName("L3g4200d");

  while (!this->shouldTerminate()) {

    updateSensorData();

    this->eventSource.broadcastFlags(1);

    this->waitAnyEventTimeout(ALL_EVENTS, MS2ST(200));

  }
  return RDY_OK;
}

int16_t
L3G4200D::
getAngularRate(const uint8_t axis) {

  return this->angularRate[axis];

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

  if (sreg & L3G4200D::XDA)
    this->angularRate[L3G4200D::AXIS_X] = (buffer[3] << 8) + buffer[2];

  if (sreg & L3G4200D::YDA)
    this->angularRate[L3G4200D::AXIS_Y] = (buffer[5] << 8) + buffer[4];

  if (sreg & L3G4200D::ZDA)
    this->angularRate[L3G4200D::AXIS_Z] = (buffer[7] << 8) + buffer[6];

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
