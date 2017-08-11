#include <ch.hpp>
#include <hal.h>
#include <string.h>  // memcpy

#include <amiro/Constants.h>
#include <amiro/ControllerAreaNetworkTx.h>

using namespace chibios_rt;
using namespace amiro;

using namespace types;
using namespace amiro::constants;

ControllerAreaNetworkTx::ControllerAreaNetworkTx(CANDriver *can, const uint8_t boardId)
    : BaseStaticThread<128>(),
      boardId(boardId),
      canDriver(can)
{
#ifdef STM32F4XX
  this->canConfig.mcr = CAN_MCR_ABOM | CAN_MCR_AWUM | CAN_MCR_TXFP;
  this->canConfig.btr = CAN_BTR_SJW(1) | CAN_BTR_TS2(3) | CAN_BTR_TS1(15)
                        | CAN_BTR_BRP(1);
#else
  this->canConfig.mcr = CAN_MCR_ABOM | CAN_MCR_AWUM | CAN_MCR_TXFP;
  this->canConfig.btr = CAN_BTR_SJW(1) | CAN_BTR_TS2(2) | CAN_BTR_TS1(13)
                        | CAN_BTR_BRP(1);
#endif
}

ControllerAreaNetworkTx::
~ControllerAreaNetworkTx() {

}

//----------------------------------------------------------------

void ControllerAreaNetworkTx::setLightBrightness(int brightness) {
  CANTxFrame frame;
  frame.SID = 0;
  this->encodeDeviceId(&frame, CAN::BRIGHTNESS_ID);
  frame.data8[0] = brightness;
  frame.DLC = 1;
  this->transmitMessage(&frame);
}

void ControllerAreaNetworkTx::setLightColor(int index, Color color) {
  CANTxFrame frame;
  frame.SID = 0;
  this->encodeDeviceId(&frame, CAN::COLOR_ID(index));
  frame.data8[0] = color.getRed();
  frame.data8[1] = color.getGreen();
  frame.data8[2] = color.getBlue();
  frame.DLC = 3;
  this->transmitMessage(&frame);
}

void ControllerAreaNetworkTx::setOdometry(types::position robotPosition) {
  CANTxFrame frame;
  frame.SID = 0;
  this->encodeDeviceId(&frame, CAN::SET_ODOMETRY_ID);
  // Cut of the first byte, which precission is not needed
  int32_t x_mm = (robotPosition.x >> 8);
  int32_t y_mm = (robotPosition.y >> 8);
  int16_t f_z_mrad = int16_t(robotPosition.f_z >> 8 );
  // Copy the data structure
  memcpy((uint8_t *)&(frame.data8[0]), (uint8_t *)&x_mm, 3);
  memcpy((uint8_t *)&(frame.data8[3]), (uint8_t *)&y_mm, 3);
  memcpy((uint8_t *)&(frame.data8[6]), (uint8_t *)&f_z_mrad, 2);
  frame.DLC = 8;
  this->transmitMessage(&frame);
}

//----------------------------------------------------------------

void ControllerAreaNetworkTx::setTargetSpeed(int32_t leftURpm, int32_t rightURpm) {
  CANTxFrame frame;
  frame.SID = 0;
  this->encodeDeviceId(&frame, CAN::TARGET_RPM_ID);
  frame.data32[0] = leftURpm;
  frame.data32[1] = rightURpm;
  frame.DLC = 8;
  this->transmitMessage(&frame);
}

void ControllerAreaNetworkTx::setTargetSpeed(kinematic &targetSpeed) {
  CANTxFrame frame;
  frame.SID = 0;
  this->encodeDeviceId(&frame, CAN::TARGET_SPEED_ID);
  frame.data32[0] = targetSpeed.x;
  frame.data32[1] = targetSpeed.w_z;
  frame.DLC = 8;
  this->transmitMessage(&frame);
}

void ControllerAreaNetworkTx::setTargetPosition(types::position &targetPosition, uint32_t targetTime) {
  CANTxFrame frame;
  frame.SID = 0;
  this->encodeDeviceId(&frame, CAN::TARGET_POSITION_ID);
  frame.data32[0] = targetPosition.x;
  frame.data32[1] = targetPosition.f_z;
  frame.data32[1] = targetTime;
  frame.DLC = 12;
  this->transmitMessage(&frame);
}

void ControllerAreaNetworkTx::setKinematicConstants(float Ed, float Eb) {
  CANTxFrame frame;
  frame.SID = 0;
  this->encodeDeviceId(&frame, CAN::SET_KINEMATIC_CONST_ID);
  frame.data32[0] = Ed;
  frame.data32[1] = Eb;
  frame.DLC = 8;
  this->transmitMessage(&frame);
}

void ControllerAreaNetworkTx::broadcastShutdown() {
    CANTxFrame frame;
    frame.SID = 0x00u;
    this->encodeDeviceId(&frame, CAN::BROADCAST_SHUTDOWN);
    frame.data16[0] = CAN::SHUTDOWN_MAGIC;
    frame.DLC = 2;
    this->transmitMessage(&frame);

}

//----------------------------------------------------------------

void ControllerAreaNetworkTx::txQueryShell(uint8_t toBoardId, char *textdata, uint16_t size) {
    uint16_t cnt = size;
    CANTxFrame frame;
    frame.SID = 0;
    frame.DLC = 8;
    this->encodeDeviceId(&frame, CAN::SHELL_QUERY_ID(toBoardId));

    while (cnt > 8) {
        memcpy(frame.data8,&textdata[size-cnt],8) ;
        this->transmitMessage(&frame);
        cnt -= 8;
    }
    memcpy(frame.data8,&textdata[size-cnt],cnt) ;
    frame.DLC = cnt;
    this->transmitMessage(&frame);
}


void ControllerAreaNetworkTx::txReplyShell(uint8_t toBoardId, char *textdata, uint16_t size) {
    uint16_t cnt = size;
    CANTxFrame frame;
    frame.SID = 0;
    frame.DLC = 8;
    this->encodeDeviceId(&frame, CAN::SHELL_REPLY_ID(toBoardId));

    while (cnt > 8) {
        memcpy(frame.data8,&textdata[size-cnt],8) ;
        this->transmitMessage(&frame);
        cnt -= 8;
    }
    memcpy(frame.data8,&textdata[size-cnt],cnt) ;
    frame.DLC = cnt;
    this->transmitMessage(&frame);
}

//----------------------------------------------------------------

msg_t ControllerAreaNetworkTx::main(void) {
  evtInit(&this->evtimer, CAN::UPDATE_PERIOD);

  this->eventTimerEvtSource = reinterpret_cast<EvtSource *>(&this->evtimer.et_es);

  this->eventTimerEvtSource->registerOne(&this->eventTimerEvtListener, CAN::PERIODIC_TIMER_ID);

  evtStart(&this->evtimer);

  this->setName("ControllerAreaNetworkTx");

  while (!this->shouldTerminate()) {
    eventmask_t eventMask = this->waitOneEvent(ALL_EVENTS);
    switch (eventMask) {
      case EVENT_MASK(CAN::PERIODIC_TIMER_ID):
        updateSensorVal();
        periodicBroadcast();
        break;
    }
  }

  evtStop(&this->evtimer);
  this->eventTimerEvtSource->unregister(&this->eventTimerEvtListener);

  return RDY_OK;
}

msg_t ControllerAreaNetworkTx::updateSensorVal() {
  return RDY_OK;
}

//----------------------------------------------------------------

void ControllerAreaNetworkTx::encodeBoardId(CANTxFrame *frame, int board) {
  frame->SID |= (board & CAN::BOARD_ID_MASK) << CAN::BOARD_ID_SHIFT;
}

void ControllerAreaNetworkTx::encodeDeviceId(CANTxFrame *frame, int device) {
  frame->SID |= (device & CAN::DEVICE_ID_MASK) << CAN::DEVICE_ID_SHIFT;
}

void ControllerAreaNetworkTx::encodeIndexId(CANTxFrame *frame, int index) {
  frame->SID |= (index & CAN::INDEX_ID_MASK) << CAN::INDEX_ID_SHIFT;
}

//----------------------------------------------------------------

void ControllerAreaNetworkTx::transmitMessage(CANTxFrame *frame) {
  this->encodeBoardId(frame, boardId);
  frame->IDE = CAN_IDE_STD;
  frame->RTR = CAN_RTR_DATA;
  /**
   * We cannot use TIME_INFINITE here b/c
   * lower boards might crash/power down etc.
   * and block CAN bus (though they should not...).
   * Then, we get stuck here and cannot terminate CAN thread
   * Therefore:
   *
   * 1 us       * (  1 + 11 +   1 +   1 +   1 +   4 +   64 +  15 +     1 +   1 +     1 +   7) * 5        = 545 us
   * 1/ (1 MHz) * (SOF + ID + RTR + IDE + RES + DLC + DATA + CRC + DELIM + ACK + DELIM + EOF) * #RETRIES
   */

  BaseThread::sleep(MS2ST(2));
  canTransmit(this->canDriver, CAN_TX_MAILBOXES, frame, US2ST(545));
}
