#ifndef AMIRO_MOTOR_DISTCONTROL_H_
#define AMIRO_MOTOR_DISTCONTROL_H_

#include <amiro/MotorIncrements.h>
#include <amiro/MotorControl.h>
#include <amiro/Constants.h>
#include <Types.h>

#include <chprintf.h>

namespace amiro {

  class DistControl : public chibios_rt::BaseStaticThread<256> {
  public:
    /**
     * Constructor
     *
     * @param mc object for retrieving the motors
     * @param mi object for retrieving the motor increments of the qei
     */
    DistControl(MotorControl* mc, MotorIncrements* mi);

    /**
     * Get the current target distance in um.
     *
     * @return target distance in um
     */
    int getCurrentTargetDist(void);

    /**
     * Get the current target angle in urad.
     *
     * @return target distance in urad
     */
    int getCurrentTargetAngle(void);

    /**
     * Sets the target position.
     *
     * @param distance Distance to drive in um
     * @param angle Angle to turn in urad
     * @param time Given time to drive in ms
     */
    void setTargetPosition(int32_t distance, int32_t angle, uint16_t time);

    /**
     * Gives if the distance controller is active.
     *
     * @return true if active, false if inactive
     */
    bool isActive(void);

    /**
     * Deactivates the controller.
     * The motor velocities won't be set in any way!
     */
    void deactivateController(void);



  protected:
    virtual msg_t main(void);

  private:
    /**
     * Calculates the velocities by using the error distance and error angle.
     */
    void calcVelocities(void);

    MotorControl* motorControl;
    MotorIncrements* motorIncrements;
    bool controllerActive, drivingForward, turningLeft, newVelocities;
    const uint32_t period;
    int32_t increment[2];
    float incrementDifference[2];
    float actualDistance[2]; // m
    int32_t fullDistance[2]; // um
    int32_t realDistance; // um
    int32_t realAngle; // urad
    int32_t targetDistance; // um
    int32_t targetAngle; // urad
    int32_t errorDistance; // um
    int32_t errorAngle; // urad
    uint32_t restTime; // us
    types::kinematic maxVelocity;
    types::kinematic targetVelocity;
    types::kinematic minVelocity;
  };
}

#endif /* AMIRO_MOTOR_DISTCONTROL_H_ */
