#ifndef AMIRO_MOTOR_CONTROL_H_
#define AMIRO_MOTOR_CONTROL_H_

#include <amiro/MotorIncrements.h>
#include <amiro/Constants.h>
#include <Types.h>
#include <amiro/FileSystemInputOutput/FSIODiWheelDrive.hpp>

namespace amiro {

  class MotorControl : public chibios_rt::BaseStaticThread<512> {
  public:
    /**
     * Constructor
     *
     * @param pwm pulse width modulation driver (pwmd)
     *            Can be any free PWMDx in 'ChibiOS-RT/os/hal/platforms/STM32/TIMv1/pwm_lld.h'
     * @param mi object for retrieving the motor increments of the qei
     * @param port GPIO port for motor control (should be the macro 'GPIOB')
     * @param pad GPIO command for motor control (should be the macro 'GPIOB_POWER_EN' for enable)
     * @param memory Memory interface to load/store parameters
     */
    MotorControl(PWMDriver* pwm, MotorIncrements* mi, GPIO_TypeDef* port, int pad, fileSystemIo::FSIODiWheelDrive *memory);

    /**
     * Get the current speed of the left wheel in rounds/min
     *
     * @return speed of left wheel in rounds/min
     */
    int getCurrentRPMLeft();

    /**
     * Get the current speed of the right wheel in rounds/min
     *
     * @return speed of right wheel in rounds/min
     */
    int getCurrentRPMRight();

    chibios_rt::EvtSource* getEventSource();

    /**
     * Sets the target velocity
     *
     * @param targetVelocity Kartesian kinematic vector
     */
    void setTargetSpeed(const types::kinematic &targetVelocity);

    /**
     * Sets the target velocity in µ rounds per minute for every wheel
     *
     * @param targetURpmLeft Rounds per minute in µ1/min of the left wheel
     * @param targetURpmLeft Rounds per minute in µ1/min of the right wheel
     */
    void setTargetRPM(int32_t targetURpmLeft, int32_t targetURpmRight);

    /**
     * Get the current velocitiy as a kinematic struct
     *
     * @return Current velocity
     */
    types::kinematic getCurrentVelocity();

    /**
     * Sets the correction factor for the wheel diameter.
     * The factor Ed is the ratio of the wheel diameters
     * <Ed = wheelDiameterRight / wheelDiameterLeft> as
     * introduced in eq. 3 by J. Borenstein (Correction of
     * Systematic Odometry Errors in Mobile Robots). This
     * function calculates then the correction factors for every
     * wheel by eq. 17a and 17b.
     *
     * @param Ed Wheel diameter ratio
     * @param storeEbToMemory Do override Ed in the memory with the given value
     * @return Return value of the memory interface
     */
    msg_t setWheelDiameterCorrectionFactor(float Ed = 1.0f, bool_t storeEdToMemory = false);

    /**
     * Sets the correction factor for the wheel base width.
     * The factor Eb is the ratio of the actual and nominal
     * base width <Eb = bActual / bNominal> as introduced
     * in eq. 3 by J. Borenstein (Correction of
     * Systematic Odometry Errors in Mobile Robots). This
     * function calculates then the actual base width of the
     * robot using eq. 4.
     *
     * @param Eb Base width ratio
     * @param storeEbToMemory Do override Eb in the memory with the given value
     * @return Return value of the memory interface
     */
    msg_t setActualWheelBaseDistance(float Eb = 1.0f, bool_t storeEbToMemory = false);

    /**
     * Calculate the current increments of both wheels and
     * update oldIncrement and incrementsDifference,
     * which is the corrected difference between the current increments
     * and the old ones.
     * The corrected difference is the original difference between the old
     * increments and new increments, multiplied with the wheelDiameterCorrectionFactor.
     * The incremennt source is given by motorIncrements.
     *
     * @param motorIncrements Increment source
     * @param oldIncrement Old increments, which are updated
     * @param incrementDifference Corrected difference between old and current increments
     * @return Return value of the memory interface
     */
    static void updateIncrements(MotorIncrements* motorIncrements, int32_t (&oldIncrement)[2], float (&incrementDifference)[2]);

    /**
     * Calculate the current speed of both wheels and
     * updates actualSpeed.
     *
     * @param incrementDifference Difference between old and current increments
     * @param actualSpeed Actual speed of both wheels
     * @param Update period
     */
    static void updateSpeed(const float (&incrementDifference)[2], int32_t (&actualSpeed)[2], const uint32_t period);

    /**
     * Calculate the current driven distance of both wheels and
     * updates actualDistance.
     *
     * @param incrementDifference Difference between old and current increments
     * @param actualDistance Actual distance driven
     */
    static void updateDistance(const float (&incrementDifference)[2], float (&actualDistance)[2]);

    /**
     * Prints Control Gains
     */
    void printGains();

    /**
     * @brief Resets control gains.
     */
    void resetGains();

  protected:
    virtual msg_t main(void);

  private:

    /**
     * Calculate the velocitiy in the robot frame
     * and saves it to this->currentVelocity
     */
    void calcVelocity();

    /**
     * PID Controller that works directly on the forward velocity v of the robots center and its
     * angular velocity w around its z axis. The methods setLeftWheelSpeed() and setRightWheelSpeed()
     * are used to set the final pwm values.
     */
    void PIDController();

    /**
     * Deletes the oldest entry in lastVelocitiesV[] and lastVelocitiesW[], pushes the other values up and saves the currentVelocity.x(w_z) in the last entry.
     */
    void updateDerivativeLastVelocities();


    ////////////////////////////////////////////////
    /////////////////Calibration////////////////////
    ////////////////////////////////////////////////
    /**
     * Finds a prefactor for the stronger motor to match its power level to the weaker motor.
     */
    void calibrate();

    /**
     * Finds the P, I and D gains for the PID Controller using the Nichols-Ziegler Method.
     */
    void calibrateZiegler();

    /**
     * Counts the number of sign changes from the last 30 velocities.
     */
    int getNumberofSignChanges();

    /**
     * Update the past lastVelocitiesV array.
     */
    void updatePastVelocities();


    /**
     * Sets wheel speed according to the output of the PID Controller
     */
    void setLeftWheelSpeed(int diffv, int diffw);
    /**
     * Sets wheel speed according to the output of the PID Controller
     */
    void setRightWheelSpeed(int diffv, int diffw);

    int getLeftWheelSpeed();
    int getRightWheelSpeed();


    /**
     * Write the duty cicle from this->pwmPercentage
     * to the PWM driver
     */
    void writePulseWidthModulation();

    /**
     * Control logic to save space in main loop
     */
    void controllerAndCalibrationLogic();



    PWMDriver* pwmDriver;
    PWMConfig pwmConfig;
    MotorIncrements* motorIncrements;
    GPIO_TypeDef *powerEnablePort;
    const int powerEnablePad;
    chibios_rt::EvtSource eventSource;
    //const uint32_t period;
    uint32_t period;
    fileSystemIo::FSIODiWheelDrive *memory;
    int32_t actualSpeed[2];
    float actualDistance[2];
    float errorSum[2];
    int32_t increment[2];
    float incrementDifference[2];
    int pwmPercentage[2];
    types::kinematic targetVelocity;
    types::kinematic currentVelocity;
    types::kinematic lastVelocity;
    float errorSumDiff;
    bool newTargetVelocities;
    int delay = 0;
    float Ed;
    float Eb;

    int pGain = 1000;

    float iGain = 0.08;
    int antiWindupV = 7100000;
    int antiWindupW = 200000000;
    int accumulatedErrorV =0; // accumulated velocity error
    int accumulatedErrorW =0; // accumulated omega error

    float dGain =0.01;
    int lastVelocitiesV[6];
    int lastVelocitiesW[6];

    //motor calibration variables
    int numberOfLastVelocitiesV = 30;
    int lastVelocitiesVBig[30];
    int rightWValues[3];
    int leftWValues[3];
    float motorCalibrationFactor =1.0;
    //ziegler calibration variables
    int zieglerHelp= 1;
    int zieglerHelp2 = 0;
    bool ziegler= true;
    bool ziegler2 = true;
    float zieglerPeriod;
    int zieglerCalibrationTime = 0;
    int wheelCalibrationTime = 0;
    bool startedZieglerCalibration = false;
    bool startedWheelCalibration = false;
    bool motorCalibration = true;




  public:
    bool isCalibrating = false;
    static float wheelDiameterCorrectionFactor[2];
    static float actualWheelBaseDistanceSI;


  };
}

#endif /* AMIRO_MOTOR_CONTROL_H_ */
