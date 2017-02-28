#include "ch.hpp"
#include "hal.h"
#include "qei.h"
#include "DiWheelDrive.h"

#include <global.hpp>

using namespace chibios_rt;
using namespace amiro;
using namespace types;

extern volatile uint32_t shutdown_now;
extern Global global;

DiWheelDrive::DiWheelDrive(CANDriver *can)
    : ControllerAreaNetworkTx(can, CAN::DI_WHEEL_DRIVE_ID),
      ControllerAreaNetworkRx(can, CAN::DI_WHEEL_DRIVE_ID),
      bcCounter(0)
{
}

msg_t DiWheelDrive::receiveMessage(CANRxFrame *frame) {
  int deviceId = this->decodeDeviceId(frame);

  switch (deviceId) {

    case CAN::SHELL_REPLY_ID(CAN::DI_WHEEL_DRIVE_ID):
      if (frame->DLC > 0) {
        sdWrite(&SD1, frame->data8, frame->DLC);
        return RDY_OK;
      }
      break;

    case CAN::SHELL_QUERY_ID(CAN::DI_WHEEL_DRIVE_ID):
      if (frame->DLC != 0) {
        global.sercanmux1.convCan2Serial(frame->data8, frame->DLC);
        return RDY_OK;
      } else {
        global.sercanmux1.rcvSwitchCmd(this->decodeBoardId(frame));
        return RDY_OK;
      }
      break;

    case CAN::TARGET_SPEED_ID:
      if (frame->DLC == 8) {
        global.distcontrol.deactivateController();
        kinematic targetVelocity;
        targetVelocity.x = frame->data32[0];
        targetVelocity.w_z = frame->data32[1];
        global.motorcontrol.setTargetSpeed(targetVelocity);
        return RDY_OK;
      }
      break;

    case CAN::TARGET_RPM_ID:
      if (frame->DLC == 8) {
        global.distcontrol.deactivateController();
        global.motorcontrol.setTargetRPM(frame->data32[0], frame->data32[1]);
        return RDY_OK;
      }
      break;

    case CAN::SET_ODOMETRY_ID:
      if (frame->DLC == 8) {
        int32_t robotPositionX = (frame->data8[0] << 8 | frame->data8[1] << 16 | frame->data8[2] << 24);
        int32_t robotPositionY = (frame->data8[3] << 8 | frame->data8[4] << 16 | frame->data8[5] << 24);
        int32_t robotPositionF_Z = (frame->data8[6] << 8 | frame->data8[7] << 16);
        global.odometry.setPosition(float(robotPositionX)*1e-6,float(robotPositionY)*1e-6,float(robotPositionF_Z)*1e-6);
        return RDY_OK;
      }
      break;

    case CAN::BROADCAST_SHUTDOWN:
      if (frame->DLC == 2 && frame->data16[0] == CAN::SHUTDOWN_MAGIC) {
        shutdown_now = 0x4;
        return RDY_OK;
      }
      break;

    case CAN::CALIBRATE_PROXIMITY_FLOOR:
      // Dont care about the payload but start the calibration
      // TODO Care about the payload. Differ between:
      // 1: Do fresh calibration (Save values to memory and to temporary values)
      // 2: Remove temporary Calibration and get uncalibrated values
      // 3: Load calibration from memory
      this->calibrate();
      break;

    case CAN::TARGET_POSITION_ID:
      if (frame->DLC == 8) {
        // Robot target position [x] = µm, [f_z] = µrad, [t] = ms
        int32_t robotPositionX = (frame->data8[0] << 8 | frame->data8[1] << 16 | frame->data8[2] << 24);
        int32_t robotPositionF_Z = (frame->data8[3] << 8 | frame->data8[4] << 16 | frame->data8[5] << 24);
        uint16_t targetTimeMilliSeconds = (frame->data8[6] | frame->data8[7] << 8);
        //chprintf((BaseSequentialStream*) &SD1, "\nx=%d\nf_z=%d\nt=%d", robotPositionX, robotPositionF_Z, targetTimeMilliSeconds);
        global.distcontrol.setTargetPosition(robotPositionX, robotPositionF_Z, targetTimeMilliSeconds);
        return RDY_OK;
      }
      break;

    case CAN::SET_KINEMATIC_CONST_ID:
      if (frame->DLC == 8) {
/*        // Set (but do not store) Ed
        global.motorcontrol.setWheelDiameterCorrectionFactor(static_cast<float>(frame->data32[0]), false);
        // Set (but do not store) Eb
        global.motorcontrol.setActualWheelBaseDistance(static_cast<float>(frame->data32[1]), false);
        return RDY_OK;*/
        // Set (but do not store) Ed
        uint32_t ed_int = static_cast<uint32_t>(frame->data32[0]);
        float ed_float = static_cast<float>(ed_int)/1000000.0;
        global.motorcontrol.setWheelDiameterCorrectionFactor(ed_float, false);
        // Set (but do not store) Eb
        uint32_t eb_int = static_cast<uint32_t>(frame->data32[1]);
        float eb_float = static_cast<float>(eb_int)/1000000.0;
        global.motorcontrol.setActualWheelBaseDistance(eb_float, false);
        //chprintf((BaseSequentialStream*) &SD1, "Edi=%i, Edf=%f, Ebi=%i, Ebf=%f\n", ed_int, ed_float, eb_int, eb_float);
        return RDY_OK;
      }
      break;

    case CAN::POWER_STATUS_ID:
      if (frame->DLC == 6) {
        // The power status is evaluated by inherited ControllerAreaNetworkRx object, but depending on the flags the power path controller needs to enabled or disabled.
        types::power_status::ChargingState charging_flags;
        charging_flags.value = frame->data8[0];
        global.ltc4412.enable(charging_flags.content.diwheeldrive_enable_power_path);
        // Do not return with RDY_OK, or the inherited ControllerAreaNetworkRx object would not evaluate the rest of this message.
      }
    break;

    default:
      break;
  }
  return -1;
}

msg_t DiWheelDrive::updateSensorVal() {

  // Update robot velocity values
  kinematic currentVelocity = global.motorcontrol.getCurrentVelocity();
  this->actualSpeed[0] = currentVelocity.x;
  this->actualSpeed[1] = currentVelocity.w_z;

  // Update odometry values
  this->robotPosition = global.odometry.getPosition();

  // Update proximity values
  for (int idx = 0; idx < 4; ++idx)
    this->proximityFloorValue[idx] = global.vcnl4020[idx].getProximityScaledWoOffset();

  // Update magnetometer values
  for (uint8_t axis = 0; axis < 3; ++axis) {
    this->magnetometerValue[axis] = global.hmc5883l.getMagnetizationGauss(axis);
  }

  // Update gyroscope values
  for (uint8_t axis = 0; axis < 3; ++axis) {
    this->gyroscopeValue[axis] = global.l3g4200d.getAngularRate(axis);
  }

  return 0;
}

void DiWheelDrive::periodicBroadcast() {
  CANTxFrame frame;
  frame.SID = 0;

  // Send the velocites µm/s of the x axis and µrad/s around z axis: start
  this->encodeDeviceId(&frame, CAN::ACTUAL_SPEED_ID);
  frame.data32[0] = this->actualSpeed[0];
  frame.data32[1] = this->actualSpeed[1];
  frame.DLC = 8;
  this->transmitMessage(&frame);

  // Send the valocites µm/s of the x axis and µrad/s around z axis: end
  // Send the odometry: start
  BaseThread::sleep(US2ST(10)); // Use to sleep for 10 CAN cycle (@1Mbit), otherwise the cognition-board might not receive all messagee
  // Set the frame id
  frame.SID = 0;
  this->encodeDeviceId(&frame, CAN::ODOMETRY_ID);
  // Cut of the first byte, which precission is not needed
  int32_t x_mm = (this->robotPosition.x >> 8);
  int32_t y_mm = (this->robotPosition.y >> 8);
  int16_t f_z_mrad = int16_t(this->robotPosition.f_z >> 8 );
  // Copy the data structure
  memcpy((uint8_t *)&(frame.data8[0]), (uint8_t *)&x_mm, 3);
  memcpy((uint8_t *)&(frame.data8[3]), (uint8_t *)&y_mm, 3);
  memcpy((uint8_t *)&(frame.data8[6]), (uint8_t *)&f_z_mrad, 2);
  frame.DLC = 8;
  this->transmitMessage(&frame);

  // Send the odometry: end
  // Send the proximity values of the floor: start
  BaseThread::sleep(US2ST(10)); // Use to sleep for 10 CAN cycle (@1Mbit), otherwise the cognition-board might not receive all messagee
  // Set the frame id
  frame.SID = 0;
  this->encodeDeviceId(&frame, CAN::PROXIMITY_FLOOR_ID);
  frame.data16[0] = this->proximityFloorValue[0];
  frame.data16[1] = this->proximityFloorValue[1];
  frame.data16[2] = this->proximityFloorValue[2];
  frame.data16[3] = this->proximityFloorValue[3];
  frame.DLC = 8;
  this->transmitMessage(&frame);

  // Send the magnetometer data
  for (uint8_t axis = 0; axis < 3; ++axis) {
    frame.SID = 0;
    this->encodeDeviceId(&frame, CAN::MAGNETOMETER_X_ID + axis); // Y- and Z-axis have according IDs
    frame.data32[0] = this->magnetometerValue[axis];
    frame.DLC = 4;
    this->transmitMessage(&frame);
  }

  // Send gyroscope data
  frame.SID = 0;
  this->encodeDeviceId(&frame, CAN::GYROSCOPE_ID);
  frame.data16[0] = this->gyroscopeValue[0];
  frame.data16[1] = this->gyroscopeValue[1];
  frame.data16[2] = this->gyroscopeValue[2];
  frame.DLC = 6;
  this->transmitMessage(&frame);

  // Send the board ID (board ID of DiWheelDrive = Robot ID)
  if (this->bcCounter % 10 == 0) {
    frame.SID = 0;
    this->encodeDeviceId(&frame, CAN::ROBOT_ID);
    frame.data8[0] = this->robotId;
    frame.DLC = 1;
    this->transmitMessage(&frame);
  }

  ++this->bcCounter;
}

void DiWheelDrive::calibrate() {
  // Stop sending and receiving of values to indicate the calibration phase
//   eventTimerEvtSource->unregister(&this->eventTimerEvtListener);
//   rxFullCanEvtSource->unregister(&this->rxFullCanEvtListener);

  this->calibrateProximityFloorValues();

  // Start sending and receving of values
//   eventTimerEvtSource->registerOne(&this->eventTimerEvtListener, CAN::PERIODIC_TIMER_ID);
//   rxFullCanEvtSource->registerOne(&this->rxFullCanEvtListener, CAN::RECEIVED_ID);

}

void DiWheelDrive::calibrateProximityFloorValues() {

  uint16_t buffer;
  for (uint8_t idx = 0; idx < 4; ++idx) {
    global.vcnl4020[idx].calibrate();
    buffer = global.vcnl4020[idx].getProximityOffset();
    global.memory.setVcnl4020Offset(buffer,idx);
  }

}

ThreadReference DiWheelDrive::start(tprio_t PRIO) {
  // set the robot ID as the board ID, which is read from the memory
  if (global.memory.getBoardId(&this->robotId) != fileSystemIo::FileSystemIoBase::OK) {
    this->robotId = 0;
  }

  this->ControllerAreaNetworkRx::start(PRIO + 1);
  this->ControllerAreaNetworkTx::start(PRIO);
  return NULL;
}

msg_t
DiWheelDrive::terminate(void) {
  msg_t ret = RDY_OK;

  this->ControllerAreaNetworkTx::requestTerminate();
  ret |= this->ControllerAreaNetworkTx::wait();
  this->ControllerAreaNetworkRx::requestTerminate();
  ret |= this->ControllerAreaNetworkRx::wait();

  return ret;
}
