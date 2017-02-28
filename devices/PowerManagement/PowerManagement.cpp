#include "ch.hpp"
#include "hal.h"
#include "PowerManagement.h"

#include <amiro/proximity/vcnl4020.hpp>
#include <global.hpp>

#include <algorithm>
#include <chprintf.h>

using namespace chibios_rt;
using namespace amiro;

extern Global global;

PowerManagement::PowerManagement(CANDriver *can)
    : ControllerAreaNetworkTx(can, CAN::POWER_MANAGEMENT_ID),
      ControllerAreaNetworkRx(can, CAN::POWER_MANAGEMENT_ID),
      bc_counter(0)
{
  this->powerStatus.charging_flags.value = 0;
}

msg_t PowerManagement::receiveMessage(CANRxFrame *frame) {
  int deviceId = this->decodeDeviceId(frame);
  switch (deviceId) {

    case CAN::SHELL_REPLY_ID(CAN::POWER_MANAGEMENT_ID):
      if (frame->DLC > 0) {
        sdWrite(&SD1, frame->data8, frame->DLC);
        return RDY_OK;
      }
      break;

    case CAN::SHELL_QUERY_ID(CAN::POWER_MANAGEMENT_ID):
      if (frame->DLC != 0) {
        global.sercanmux1.convCan2Serial(frame->data8, frame->DLC);
        return RDY_OK;
      } else {
        global.sercanmux1.rcvSwitchCmd(this->decodeBoardId(frame));
        return RDY_OK;
      }
      break;

    case CAN::CALIBRATE_PROXIMITY_RING:
      // Dont care about the payload but start the calibration
      // TODO Care about the payload. Differ between:
      // 1: Do fresh calibration (Save values to memory and to temporary values)
      // 2: Remove temporary Calibration and get uncalibrated values
      // 3: Load calibration from memory
      this->calibrate();
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

msg_t PowerManagement::updateSensorVal() {

  // update charger status
  this->powerStatus.charging_flags.content.powermanagement_plugged_in = global.ltc4412.isPluggedIn();

  // update fuel gauges values
  const BQ27500::Driver::UpdateData* power[2] {
    &global.bq27500[constants::PowerManagement::BAT_A].getStatus(),
    &global.bq27500[constants::PowerManagement::BAT_B].getStatus()
  };
  this->powerStatus.charging_flags.content.powermanagement_charging = (this->powerStatus.charging_flags.content.powermanagement_plugged_in &&
                                                                       this->powerStatus.charging_flags.content.vsys_higher_than_9V &&
                                                                       power[0]->minutes_to_empty == uint16_t(~0) &&
                                                                       power[1]->minutes_to_empty == uint16_t(~0))?
                                                                       true : false;
  this->powerStatus.charging_flags.content.diwheeldrive_charging = (this->powerStatus.charging_flags.content.diwheeldrive_enable_power_path &&
                                                                    this->powerStatus.charging_flags.content.vsys_higher_than_9V &&
                                                                    power[0]->minutes_to_empty == uint16_t(~0) &&
                                                                    power[1]->minutes_to_empty == uint16_t(~0))?
                                                                    true : false;
  this->powerStatus.state_of_charge = (power[0]->state_of_charge + power[1]->state_of_charge) / 2;
  if (this->powerStatus.charging_flags.content.powermanagement_charging || this->powerStatus.charging_flags.content.diwheeldrive_charging) {
    /*
     * Assumption:
     * When charging there is enough power available to charge both batteries at full rate simultaneously.
     * Thus, the second battery will not charge faster when the first battery is fully charged.
     */
    this->powerStatus.minutes_remaining = std::max(power[0]->minutes_to_full, power[1]->minutes_to_full);
  } else {
    /*
     * Computation of the remaining discharging time:
     * Take the time until the first of the two batteries is empty and add the remaining time of the second battery but half.
     *        time = min(a,b) + (max(a,b) - min(a,b))/2
     * <=>  2*time = 2*min(a,b) + max(a,b) - min(a,b)
     * <=>  2*time = min(a,b) + max(a,b)
     * <=>  2*time = a + b
     * <=>    time = (a + b)/2
     */
    this->powerStatus.minutes_remaining = (power[0]->minutes_to_empty + power[1]->minutes_to_empty) / 2;
  }
  this->powerStatus.power_consumption = (power[0]->average_power_mW + power[1]->average_power_mW) / 2;

  // update infrared sensor value
  // Note: The CANRx Value will never be updated in this thread
  for (int idx = 0; idx < 8; idx++)
    this->proximityRingValue[idx] = global.vcnl4020[idx].getProximityScaledWoOffset();

  return 0;
}

void PowerManagement::periodicBroadcast() {
  CANTxFrame frame;
  if (this->bc_counter % 10 == 0) {
    frame.SID = 0;
    this->encodeDeviceId(&frame, CAN::POWER_STATUS_ID);
    frame.data8[0] = this->powerStatus.charging_flags.value;
    frame.data8[1] = this->powerStatus.state_of_charge;
    frame.data16[1] = this->powerStatus.minutes_remaining;
    frame.data16[2] = this->powerStatus.power_consumption;
    frame.DLC = 6;
    this->transmitMessage(&frame);
  }
  for (int i = 0; i < 8; i++) {
    frame.SID = 0;
    this->encodeDeviceId(&frame, CAN::PROXIMITY_RING_ID(i));
    frame.data16[0] = this->proximityRingValue[i];
    frame.DLC = 2;
    this->transmitMessage(&frame);
    BaseThread::sleep(US2ST(10)); // Use to sleep for 10 CAN cycle (@1Mbit), otherwise the cognition-board might not receive all messagee
  }
  ++this->bc_counter;
}

void PowerManagement::calibrate() {
  // Stop sending and receiving of values to indicate the calibration phase
//   eventTimerEvtSource->unregister(&this->eventTimerEvtListener);
//   rxFullCanEvtSource->unregister(&this->rxFullCanEvtListener);

  this->calibrateProximityRingValues();

  // Start sending and receving of values
//   eventTimerEvtSource->registerOne(&this->eventTimerEvtListener, CAN::PERIODIC_TIMER_ID);
//   rxFullCanEvtSource->registerOne(&this->rxFullCanEvtListener, CAN::RECEIVED_ID);
}

void PowerManagement::calibrateProximityRingValues() {

  uint16_t buffer;
  for (uint8_t idx = 0; idx < 8; ++idx) {
    global.vcnl4020[idx].calibrate();
    buffer = global.vcnl4020[idx].getProximityOffset();
    global.memory.setVcnl4020Offset(buffer,idx);
  }
}

ThreadReference PowerManagement::start(tprio_t PRIO) {
  this->ControllerAreaNetworkRx::start(PRIO + 1);
  this->ControllerAreaNetworkTx::start(PRIO);
  return NULL;
}

types::power_status&
PowerManagement::getPowerStatus()
{
  return this->powerStatus;
}

msg_t PowerManagement::terminate(void) {
  msg_t ret = RDY_OK;

  this->ControllerAreaNetworkTx::requestTerminate();
  ret |= this->ControllerAreaNetworkTx::wait();
  this->ControllerAreaNetworkRx::requestTerminate();
  ret |= this->ControllerAreaNetworkRx::wait();

  return ret;
}
