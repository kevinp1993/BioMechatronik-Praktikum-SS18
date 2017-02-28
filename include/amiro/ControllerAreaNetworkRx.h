#ifndef AMIRO_CONTROLLER_AREA_NETWORK_RX_H_
#define AMIRO_CONTROLLER_AREA_NETWORK_RX_H_

#include <evtimer.h>
#include <amiro/Color.h>
#include <Types.h>  // ::kinematic

#include <amiro/Constants.h>  // CAN::* macros

namespace amiro {

  class ControllerAreaNetworkRx : public chibios_rt::BaseStaticThread<128> {
  public:
    ControllerAreaNetworkRx(CANDriver *can, const uint8_t boardId);
    virtual ~ControllerAreaNetworkRx() = 0;

    uint16_t getProximityRingValue(int index);
    uint16_t getProximityFloorValue(int index);
    void getActualSpeed(types::kinematic &targetSpeed);
    types::position getOdometry();
    types::power_status& getPowerStatus();
    uint8_t getRobotID();
    int32_t getMagnetometerValue(int axis);
    int16_t getGyroscopeValue(int axis);

    void calibrateProximityRingValues();
    void calibrateProximityFloorValues();

    int rxCmdShell(CANRxFrame *frame);

  protected:
    virtual msg_t main();
    virtual msg_t receiveMessage(CANRxFrame *frame);

    int decodeBoardId(CANRxFrame *frame);
    int decodeDeviceId(CANRxFrame *frame);
    int decodeIndexId(CANRxFrame *frame);

    int boardId;
    uint16_t proximityRingValue[8];
    int actualSpeed[2];
    uint16_t proximityFloorValue[4];
    int32_t magnetometerValue[3];
    int16_t gyroscopeValue[3];
    types::position robotPosition;
    types::power_status powerStatus;
    uint8_t robotId;
    chibios_rt::EvtListener rxFullCanEvtListener;
    chibios_rt::EvtSource *rxFullCanEvtSource;

  private:
    msg_t receiveSensorVal(CANRxFrame *frame);

    CANDriver *canDriver;
    CANConfig canConfig;

  };

}

#endif /* AMIRO_CONTROLLER_AREA_NETWORK_RX_H_ */

