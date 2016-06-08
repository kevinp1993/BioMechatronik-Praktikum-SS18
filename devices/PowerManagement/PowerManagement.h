#ifndef AMIRO_POWER_MANAGEMENT_H_
#define AMIRO_POWER_MANAGEMENT_H_

#include <amiro/ControllerAreaNetworkRx.h>
#include <amiro/ControllerAreaNetworkTx.h>
#include <amiro/proximity/vcnl4020.hpp>
#include <amiro/FileSystemInputOutput/FSIOPowerManagement.hpp>
#include <amiro/power/bq27500.hpp>
#include <amiro/power/ltc4412.hpp>

namespace amiro {

  class VCNL4020;

  class PowerManagement : public ControllerAreaNetworkTx, public ControllerAreaNetworkRx  {
  public:
    PowerManagement(CANDriver *can);
    virtual void  calibrate();

    /** \brief Handle the termination of ControllerAreaNetworkTx and ControllerAreaNetworkRx threads
     *         This includes waiting until the threads have terminated, thus this function might block a relatively long time.
     */
    msg_t terminate(void);

    /** \brief Handle the start of ControllerAreaNetworkTx and ControllerAreaNetworkRx threads */
    ThreadReference start(tprio_t PRIO);

    types::power_status& getPowerStatus();

  protected:
    virtual msg_t receiveMessage(CANRxFrame *frame);
    virtual msg_t updateSensorVal();
    virtual void periodicBroadcast();

    void  calibrateProximityRingValues();

  private:
    uint32_t bc_counter;

  };

}

#endif /* AMIRO_POWER_MANAGEMENT_H_ */
