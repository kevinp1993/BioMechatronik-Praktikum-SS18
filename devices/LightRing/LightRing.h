#ifndef _AMIRO_LIGTH_RING_H_
#define _AMIRO_LIGTH_RING_H_

#include <amiro/Constants.h>

#include <amiro/ControllerAreaNetworkRx.h>
#include <amiro/ControllerAreaNetworkTx.h>
#include <amiro/leds/tlc5947.hpp>
#include <amiro/FileSystemInputOutput/FSIOLightRing.hpp>

using namespace chibios_rt;

namespace amiro {

  class LightRing : public ControllerAreaNetworkTx, public ControllerAreaNetworkRx {
  public:
    LightRing(CANDriver *can, TLC5947 *tlc5947, fileSystemIo::FSIOLightRing *memory);
    void setLightBrightness(int brightness);
    void setLightColor(int index, Color color);

    /** \brief Handle the termination of ControllerAreaNetworkTx and ControllerAreaNetworkRx threads
     *         This includes waiting until the threads have terminated, thus this function might block a relatively long time.
     */
    msg_t terminate(void);

    /** \brief Handle the start of ControllerAreaNetworkTx and ControllerAreaNetworkRx threads */
    ThreadReference start(tprio_t PRIO);

  protected:
    virtual msg_t receiveMessage(CANRxFrame *frame);
    virtual void periodicBroadcast();

  private:
    TLC5947 *tlc5947;
    fileSystemIo::FSIOLightRing *memory;
  };

}

#endif /* _AMIRO_LIGTH_RING_H_ */

/** @} */
