#ifndef AMIRO_CONTROLLER_AREA_NETWORK_TX_H_
#define AMIRO_CONTROLLER_AREA_NETWORK_TX_H_

#include <evtimer.h>
#include <amiro/Color.h>
#include <Types.h>  // ::kinematic

#include <amiro/Constants.h>  // CAN::* macros

namespace amiro {

  class ControllerAreaNetworkTx : public chibios_rt::BaseStaticThread<128> {
  public:
    ControllerAreaNetworkTx(CANDriver *can, const uint8_t boardId);
    virtual ~ControllerAreaNetworkTx() = 0;

    /**
     * \brief Setting the brightness of the light ring
     *
     * @param brightness Brightness level 0 .. 200
     */
    void setLightBrightness(int brightness);

    /**
     * \brief Setting the color of the LED with the given index
     *
     * @param index Index of the LED
     * @param color Color of the LED
     */
    void setLightColor(int index, Color color);

    /**
     * \brief Setting the desired speed in as kinematic struct
     *
     * @param targetSpeed Desired speed of the robot
     */
    void setTargetSpeed(types::kinematic &targetSpeed);

    /**
     * \brief Setting the desired speed in µ rounds per minute
     *
     * @param leftURpm Desired speed of the left wheel
     * @param rightURpm Desired speed of the right wheel
     */
    void setTargetSpeed(int32_t leftURpm, int32_t rightURpm);

    /**
     * \brief Setting odometry for the Euler-Collatz algorithm on the DiWheelDrive board
     *
     * \notice Only bits 31..8 of robotPosition.x and robotPosition.y will be sent,
     * i.e. values are divided by 1024 before being sent.
     * Only bits 23..8 of orientation vector f_z be sent, i.e.
     * f_z is devided by 1024 and clamped to int16_t.
     * To avoid buffer overflow, only values between between [0, 2 * pi * 1e6] are valid.
     * This is due to size constraints of CAN message frames.
     *
     * @param robotPosition Desired robot position.
     */
    void setOdometry(types::position robotPosition);

    /**
     * \brief Setting target position of current robot
     *        local frame
     *
     * @param targetPosition Desired robot position
     * @param targetTime Desired time reaching this position
     */
    void setTargetPosition(types::position &targetPosition, uint32_t targetTime);

    /**
     * \brief Setting correction ratios of the differential kinematic
     *
     * @param Ed Diameter ratio
     * @param Eb Base width ratio
     */
    void setKinematicConstants(float Ed, float Eb);

    void txQueryShell(uint8_t toBoardId, char *textdata, uint16_t size);
    void txReplyShell(uint8_t toBoardId, char *textdata, uint16_t size);

    void broadcastShutdown();

  protected:
    virtual msg_t main();
    virtual msg_t updateSensorVal();
    void transmitMessage(CANTxFrame *frame);
    virtual void periodicBroadcast() = 0;

    void encodeBoardId(CANTxFrame *frame, int board);
    void encodeDeviceId(CANTxFrame *frame, int device);
    void encodeIndexId(CANTxFrame *frame, int index);

    int boardId;
    chibios_rt::EvtListener eventTimerEvtListener;
    chibios_rt::EvtSource *eventTimerEvtSource;

  private:
    EvTimer evtimer;
    CANDriver *canDriver;
    CANConfig canConfig;

  };

}

#endif /* AMIRO_CONTROLLER_AREA_NETWORK_TX_H_ */
