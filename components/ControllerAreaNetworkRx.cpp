#include <ch.hpp>
#include <hal.h>
#include <string.h>  // memcpy

#include <amiro/Constants.h>
#include <amiro/ControllerAreaNetworkRx.h>

#include <global.hpp>

using namespace chibios_rt;
using namespace amiro;

extern Global global;

using namespace types;
using namespace amiro::constants;

ControllerAreaNetworkRx::ControllerAreaNetworkRx(CANDriver *can, const uint8_t boardId)
    : BaseStaticThread<128>(),
      boardId(boardId),
      canDriver(can) {
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

ControllerAreaNetworkRx::
~ControllerAreaNetworkRx() {

}

//----------------------------------------------------------------

msg_t ControllerAreaNetworkRx::receiveSensorVal(CANRxFrame *frame) {
  int deviceId = this->decodeDeviceId(frame);

  // chprintf((BaseSequentialStream*) &global.sercanmux1, "DeviceId: %d\r\n",deviceId);

  switch (deviceId) {
    case CAN::PROXIMITY_RING_ID(0):
    case CAN::PROXIMITY_RING_ID(1):
    case CAN::PROXIMITY_RING_ID(2):
    case CAN::PROXIMITY_RING_ID(3):
    case CAN::PROXIMITY_RING_ID(4):
    case CAN::PROXIMITY_RING_ID(5):
    case CAN::PROXIMITY_RING_ID(6):
    case CAN::PROXIMITY_RING_ID(7):
      if (frame->DLC == 2) {
        int index = deviceId & 0x7;
        proximityRingValue[index] = frame->data16[0];
        return RDY_OK;
      }
      break;

    case CAN::ACTUAL_SPEED_ID:
      if (frame->DLC == 8) {
        actualSpeed[0] = frame->data32[0];
        actualSpeed[1] = frame->data32[1];
        return RDY_OK;
      }
      break;

    case CAN::ODOMETRY_ID:
      if (frame->DLC == 8) {
        this->robotPosition.x = (frame->data8[0] << 8 | frame->data8[1] << 16 | frame->data8[2] << 24);
        this->robotPosition.y = (frame->data8[3] << 8 | frame->data8[4] << 16 | frame->data8[5] << 24);
        this->robotPosition.f_z = (frame->data8[6] << 8 | frame->data8[7] << 16);
        return RDY_OK;
      }
      break;

    case CAN::PROXIMITY_FLOOR_ID:
      // chprintf((BaseSequentialStream*) &global.sercanmux1, "CAN::PROXIMITY_FLOOR_ID");
      if (frame->DLC == 8) {
        proximityFloorValue[0] = frame->data16[0];
        proximityFloorValue[1] = frame->data16[1];
        proximityFloorValue[2] = frame->data16[2];
        proximityFloorValue[3] = frame->data16[3];
        return RDY_OK;
      }
      break;

    case CAN::POWER_STATUS_ID:
      if (frame->DLC == 6) {
        powerStatus.charging_flags.value = frame->data8[0];
        powerStatus.state_of_charge = frame->data8[1];
        powerStatus.minutes_remaining = frame->data16[1];
        powerStatus.power_consumption = frame->data16[2];
        return RDY_OK;
      }
      break;

    case CAN::ROBOT_ID:
      if (frame->DLC == 1) {
        robotId = frame->data8[0];
        return RDY_OK;
      }
      break;

    case CAN::MAGNETOMETER_X_ID:
      if (frame->DLC == 4) {
        magnetometerValue[0] = frame->data32[0];
        return RDY_OK;
      }
      break;

    case CAN::MAGNETOMETER_Y_ID:
      if (frame->DLC == 4) {
        magnetometerValue[1] = frame->data32[0];
        return RDY_OK;
      }
      break;

    case CAN::MAGNETOMETER_Z_ID:
      if (frame->DLC == 4) {
        magnetometerValue[2] = frame->data32[0];
        return RDY_OK;
      }
      break;

    case CAN::GYROSCOPE_ID:
      if (frame->DLC == 6) {
        gyroscopeValue[0] = frame->data16[0];
        gyroscopeValue[1] = frame->data16[1];
        gyroscopeValue[2] = frame->data16[2];
        return RDY_OK;
      }
      break;

    default:
      break;
  }

  return RDY_RESET;
}

uint16_t ControllerAreaNetworkRx::getProximityRingValue(int index) {
  return this->proximityRingValue[index];
}

void ControllerAreaNetworkRx::getActualSpeed(types::kinematic &targetSpeed) {
  targetSpeed.x = this->actualSpeed[0];
  targetSpeed.w_z = this->actualSpeed[1];
}

types::position ControllerAreaNetworkRx::getOdometry() {
  return this->robotPosition;
}

power_status &ControllerAreaNetworkRx::getPowerStatus() {
  return this->powerStatus;
}

uint8_t ControllerAreaNetworkRx::getRobotID() {
  return this->robotId;
}

int32_t ControllerAreaNetworkRx::getMagnetometerValue(int axis)
{
  return this->magnetometerValue[axis];
}

int16_t ControllerAreaNetworkRx::getGyroscopeValue(int axis)
{
  return this->gyroscopeValue[axis];
}


uint16_t ControllerAreaNetworkRx::getProximityFloorValue(int index) {
  return this->proximityFloorValue[index];
}

//----------------------------------------------------------------

msg_t ControllerAreaNetworkRx::main(void) {

  this->rxFullCanEvtSource = reinterpret_cast<EvtSource *>(&this->canDriver->rxfull_event);

  this->rxFullCanEvtSource->registerOne(&this->rxFullCanEvtListener, CAN::RECEIVED_ID);

  canStart(this->canDriver, &this->canConfig);

  this->setName("ControllerAreaNetworkRx");

  while (!this->shouldTerminate()) {
    eventmask_t eventMask = this->waitOneEventTimeout(ALL_EVENTS, MS2ST(10000));
    switch (eventMask) {

      case EVENT_MASK(CAN::RECEIVED_ID):
        CANRxFrame rxframe;
        msg_t message = canReceive(this->canDriver, CAN_ANY_MAILBOX, &rxframe, TIME_IMMEDIATE);
        if (message == RDY_OK) {
          // chprintf((BaseSequentialStream*) &global.sercanmux1, "Rx Message");
          message = this->receiveMessage(&rxframe);
          if (message != RDY_OK)
            this->receiveSensorVal(&rxframe);
        }
        break;
    }
  }

  canStop(this->canDriver);
  this->rxFullCanEvtSource->unregister(&this->rxFullCanEvtListener);

  return RDY_OK;
}

msg_t ControllerAreaNetworkRx::receiveMessage(CANRxFrame *frame) {
  (void)frame;
  return RDY_OK;
}

//----------------------------------------------------------------

int ControllerAreaNetworkRx::decodeBoardId(CANRxFrame *frame) {
  return (frame->SID >> CAN::BOARD_ID_SHIFT) & CAN::BOARD_ID_MASK;
}

int ControllerAreaNetworkRx::decodeDeviceId(CANRxFrame *frame) {
  return (frame->SID >> CAN::DEVICE_ID_SHIFT) & CAN::DEVICE_ID_MASK;
}

int ControllerAreaNetworkRx::decodeIndexId(CANRxFrame *frame) {
  return (frame->SID >> CAN::INDEX_ID_SHIFT) & CAN::INDEX_ID_MASK;
}

//----------------------------------------------------------------

int ControllerAreaNetworkRx::rxCmdShell(CANRxFrame *frame) {
  return (frame->SID >> CAN::BOARD_ID_SHIFT) & CAN::BOARD_ID_MASK;
}
