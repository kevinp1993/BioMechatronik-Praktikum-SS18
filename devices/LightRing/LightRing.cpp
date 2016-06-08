#include "ch.hpp"
#include "hal.h"

#include "LightRing.h"

#include <global.hpp>

using namespace chibios_rt;
using namespace amiro;
using namespace types;

extern volatile uint32_t shutdown_now;
extern Global global;


LightRing::LightRing(CANDriver *can, TLC5947 *tlc5947, fileSystemIo::FSIOLightRing *memory)
    : ControllerAreaNetworkTx(can, CAN::LIGHT_RING_ID),
      ControllerAreaNetworkRx(can, CAN::LIGHT_RING_ID),
      tlc5947(tlc5947),
      memory(memory) {
  chDbgCheck(tlc5947 != NULL, "LightRing");
}

void LightRing::setLightBrightness(int brightness) {
  this->tlc5947->setBrightness(brightness);
  this->tlc5947->update();
}

void LightRing::setLightColor(int index, Color color) {
  this->tlc5947->setColor(index, color);
  this->tlc5947->update();
}

msg_t LightRing::receiveMessage(CANRxFrame *frame) {
  int deviceId = this->decodeDeviceId(frame);

  switch (deviceId) {

    case CAN::SHELL_REPLY_ID(CAN::LIGHT_RING_ID):
      if (frame->DLC > 0) {
        sdWrite(&SD1, frame->data8, frame->DLC);
        return RDY_OK;
      }
      break;

    case CAN::SHELL_QUERY_ID(CAN::LIGHT_RING_ID):
      if (frame->DLC != 0) {
        global.sercanmux1.convCan2Serial(frame->data8, frame->DLC);
        return RDY_OK;
      } else {
        global.sercanmux1.rcvSwitchCmd(this->decodeBoardId(frame));
        return RDY_OK;
      }
      break;

    case CAN::COLOR_ID(0):
    case CAN::COLOR_ID(1):
    case CAN::COLOR_ID(2):
    case CAN::COLOR_ID(3):
    case CAN::COLOR_ID(4):
    case CAN::COLOR_ID(5):
    case CAN::COLOR_ID(6):
    case CAN::COLOR_ID(7):
      if (frame->DLC == 3) {
        int index = deviceId & 0x7;
        Color color(frame->data8[0], frame->data8[1], frame->data8[2]);
        this->setLightColor(index, color);
        return RDY_OK;
      }
      break;

    case CAN::BRIGHTNESS_ID:
      if (frame->DLC == 1) {
        int brightness = frame->data8[0];
        this->setLightBrightness(brightness);
        return RDY_OK;
      }
      break;

    case CAN::BROADCAST_SHUTDOWN:
      if (frame->DLC == 2 && frame->data16[0] == CAN::SHUTDOWN_MAGIC) {
        shutdown_now = 0x4;
        return RDY_OK;
      }
      break;

    case CAN::ROBOT_ID:
      if (frame->DLC == 1) {
        this->robotId = frame->data8[0];
        return RDY_OK;
      }
      break;

    default:
      break;
  }
  return -1;
}

ThreadReference LightRing::start(tprio_t PRIO) {
  this->ControllerAreaNetworkRx::start(PRIO + 1);
  this->ControllerAreaNetworkTx::start(PRIO);
  return NULL;
}

msg_t
LightRing::terminate(void) {
  msg_t ret = RDY_OK;

  this->ControllerAreaNetworkTx::requestTerminate();
  ret |= this->ControllerAreaNetworkTx::wait();
  this->ControllerAreaNetworkRx::requestTerminate();
  ret |= this->ControllerAreaNetworkRx::wait();

  return ret;
}

void LightRing::periodicBroadcast() {

}
