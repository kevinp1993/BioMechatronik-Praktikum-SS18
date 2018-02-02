#ifndef AMIRO_ODOMETRY_H_
#define AMIRO_ODOMETRY_H_

#include <amiro/MotorControl.h>
#include <amiro/gyro/l3g4200d.hpp>

#include <Types.h> // types::position

namespace amiro {

  class Odometry : public chibios_rt::BaseStaticThread<512> {
  public:
    /**
     * Constructor
     *
     * @param mi object for retrieving the motor increments of the qei
     * @param gyro object for retrieving the gyroscope data
     * @param mc object for retrieving calibration parameters
     */
    Odometry(MotorIncrements* mi, L3G4200D* gyroscope);

    /**
     * Set the position of the roboter
     *
     * @param pX Position x in meter
     * @param pY Position y in meter
     * @param pPhi Orientation phi in radiant
     */
    void setPosition(float pX, float pY, float pPhi);

    /**
     * Reset the position to [0,0,0]
     */
    void resetPosition();

    /**
     * Set the position of the roboter
     *
     * @param *Cp
     */
    void setError(float* Cp);

    /**
     * Reset the error to [0,0,0;0,0,0;0,0,0]
     */
    void resetError();

    /**
     * Updates the wheelBaseDistance variable,
     * because it may change after an calibration.
     */
    void updateWheelBaseDistance();

    /**
     * Get the current position
     * @return position
     */
    types::position getPosition();

    chibios_rt::EvtSource* getEventSource();

  protected:
    virtual msg_t main(void);

  private:

    /**
     * Calculate the actual distance of both wheels
     */
    void updateDistance();

    /**
     * This is th PID controller for controling
     * the speed. It gets the new duty cicle for the pwm
     * and saves it to this->pwmPercentage to reach the
     * target speed this->targetSpeed
     */
    void updateOdometry();

    MotorIncrements* motorIncrements; // QEI driver
    L3G4200D* gyro;  // Gyroscope driver
    chibios_rt::EvtSource eventSource;
    const unsigned int period;
    int incrementsPerRevolution;
    int updatesPerMinute;
    float wheelCircumference; // wheel circumference in meter
    float wheelBaseDistanceSI; // Base width in meter
    float distance[2]; // Moved distance per wheel in meter left:0 right:1
    float wheelError[2]; // error for left:0 and right:1 wheel
    float pX, pY; // Position in meter
    float pPhi; // Orientation in Rad
    float Cp3x3[9];  // Covariance (position error) (3x3 matrix)
    int32_t increment[2]; // Absolute number of difference since last update
    float incrementDifference[2]; // Difference between old and current absolute increments
  };

}

#endif /* AMIRO_ODOMETRY_ */
