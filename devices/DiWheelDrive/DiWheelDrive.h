#ifndef AMIRO_DI_WHEEL_DRIVE_H_
#define AMIRO_DI_WHEEL_DRIVE_H_

#include <amiro/ControllerAreaNetworkRx.h>
#include <amiro/ControllerAreaNetworkTx.h>
#include <amiro/MotorIncrements.h>
#include <amiro/MotorControl.h>
#include <amiro/Odometry.h>
#include <amiro/proximity/vcnl4020.hpp>
#include <Types.h> // kinematic
#include <amiro/FileSystemInputOutput/FSIODiWheelDrive.hpp>
#include <amiro/power/ltc4412.hpp>
#include <string.h>  // memcpy

namespace amiro {

  class DiWheelDrive : public ControllerAreaNetworkTx, public ControllerAreaNetworkRx {
  public:
    DiWheelDrive(CANDriver *can);//, MotorControl *motorcontrol, Odometry *odometry, VCNL4020 (*vcnl4020)[4], fileSystemIo::FSIODiWheelDrive *memory, BaseLTC4412wEN *ltc4412);
    virtual void  calibrate();

    /** \brief Handle the termination of ControllerAreaNetworkTx and ControllerAreaNetworkRx threads
     *         This includes waiting until the threads have terminated, thus this function might block a relatively long time.
     */
    msg_t terminate(void);

    /** \brief Handle the start of ControllerAreaNetworkTx and ControllerAreaNetworkRx threads */
    ThreadReference start(tprio_t PRIO);

  protected:
    virtual msg_t receiveMessage(CANRxFrame *frame);
    virtual msg_t updateSensorVal();
    virtual void periodicBroadcast();

    void  calibrateProximityFloorValues();

  private:
    uint32_t bcCounter;
//    MotorControl *motorcontrol;
//    Odometry *odometry;
//    VCNL4020 (*vcnl4020)[4];
//    fileSystemIo::FSIODiWheelDrive *memory;
//    BaseLTC4412wEN *ltc4412;
    int actualSpeed[2];
    types::position robotPosition;

  };

}

#endif /* AMIRO_DI_WHEEL_DRIVE_H_ */
