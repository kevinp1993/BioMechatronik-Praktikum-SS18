#include <string.h>

#include <amiro/util/util.h>
#include <amiro/bus/spi/HWSPIDriver.hpp>
#include <amiro/radio/a2500r24a.hpp>

namespace amiro {

A2500R24A::A2500R24A(HWSPIDriver *driver)
    : driver(driver) {

}

A2500R24A::~A2500R24A() {

}

chibios_rt::EvtSource*
A2500R24A::getEventSource() {
  return &this->eventSource;
}

msg_t A2500R24A::main() {

  while (!this->shouldTerminate()) {

    updateSensorData();

    this->eventSource.broadcastFlags(0);

    this->waitAnyEventTimeout(ALL_EVENTS, MS2ST(200));

  }
  return RDY_OK;
}

void A2500R24A::updateSensorData() {

}

msg_t A2500R24A::configure(A2500R24AConfig *config) {

  return RDY_OK;

}

uint8_t A2500R24A::getCheck() {

  const size_t buffer_size = 1 /* addressing */
                              + 1; /* who am i */
  uint8_t buffer[buffer_size];

  // Exchange the data with the A2500R24A radio chip
  // Specify the adress and the mode
  buffer[0] =  offsetof(A2500R24A::registerStatus, PARTNUM) | A2500R24A::SPI_READ | A2500R24A::SPI_STATUS_ACCESS;
  this->driver->exchange(buffer, buffer, buffer_size);
  // Check
  if (buffer[1] == A2500R24A::A2500R24A_PARTNUM) {
          return A2500R24A::CHECK_OK;
  } else {
          return A2500R24A::CHECK_FAIL;
  }

}

} /* amiro */
