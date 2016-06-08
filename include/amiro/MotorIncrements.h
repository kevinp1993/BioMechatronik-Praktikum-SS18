#ifndef AMIRO_MOTOR_INCREMENTS_H_
#define AMIRO_MOTOR_INCREMENTS_H_

namespace amiro {

  class MotorIncrements {

    public:
    /**
     * Constructor
     * This only creates the config. To start the qei driver, one needs to call
     * qeiInit() after halInit() in the main process first. Then run this->start().
     *
     * @param qei0 quadrature encoder for left wheel from 'amiro-os/hal/include/qei.h'
     * @param qei0 quadrature encoder for right wheel from 'amiro-os/hal/include/qei.h'
     */
    MotorIncrements(QEIDriver* qei0, QEIDriver* qei1);

    /**
     * Get the encoder position
     *
     * @return position of the encode
     */
    int qeiGetPosition(int idxQei);

    /**
     * Get the range of the qei config
     *
     * @return value of the range of the qei
     */
    int getQeiConfigRange();

    /**
     * Starts the qei driver (First run qeiInit() in the main process)
     */
    void start();

  private:
    QEIDriver* qeiDriver[2];
    QEIConfig qeiConfig;
  };

}

#endif /* AMIRO_MOTOR_INCREMENTS_H_ */
