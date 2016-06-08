#ifndef AMIRO_BASE_SENSOR_H_
#define AMIRO_BASE_SENSOR_H_

/* ABOUT THE USED PREPROCESSOR DIRECTIVES IN THIS FILE
 * AMIRO_NCALIBRATION:
 *  If defined, the calibration() function will be ignored for compilation.
 *  Thus, all inheriting classes must use this directive, too.
 * AMIRO_NSELFTEST:
 *  If defined, the selftest() function will be ignored for compilation.
 *  Thus, all inheriting classes must use this directive, too.
 */

namespace amiro {

  /**
   * This is an interface class, which should be used as a blueprint for all drivers of the AMiRo.
   */
  template<typename Init_t = void, typename Calib_t = void>
  class BaseSensor
  {
  public:
    BaseSensor() {}
    virtual ~BaseSensor() {}

    /**
     * Some error codes that should be returned by the implementations of the virtual functions of this class.
     */
    enum ErrorCodes {
      SUCCESS = 0x00u, OK = 0x00u,
      ERROR = 0x80u, FAIL = 0x80u,
      NOT_IMPLEMENTED = 0xFFu
    };

    /**
     * Initializes the hardware (e.g. sets register values).
     * This function is optional.
     *
     * @param[in,out] driver specific initialization data (should be used for arguments and results); optional
     * @return an error code
     */
    virtual inline msg_t init(Init_t* = NULL)
    {
      return NOT_IMPLEMENTED;
    }

    /**
     * Reads the most important values from the hardware and stores them in local buffers.
     *
     * @return an error code
     */
    virtual msg_t update() = 0;

    /**
     * If required, wakes the hardware from power saving mode into an active state.
     * (Re)Initializes all local buffers with valid values, thus the update() function should be called.
     *
     * @return an error code
     */
    virtual msg_t wakeup() = 0;

    /**
     * Puts the hardware in a power saving mode to reduce energy consumption.
     * This function is optional.
     * @return an error code
     */
    virtual msg_t hibernate()
    {
      return NOT_IMPLEMENTED;
    }

#ifndef AMIRO_NCALIBRATION
    /**
     * Runs a calibration of the hardware.
     * This function is optional.
     *
     * @param[in,out] driver specific calibration data (should be used for arguments and results); optional
     *
     * @return an error code
     */
    virtual msg_t calibration(Calib_t* = NULL)
    {
      return NOT_IMPLEMENTED;
    }
#else
    static uint8_t calibration(Calib_t* = NULL)
    {
      return NOT_IMPLEMENTED;
    }
#endif

#ifndef AMIRO_NSELFTEST
    /**
     * Runs a self-test on the hardware and/or communication.
     * This function is optional but highly recommended.
     *
     * @return an error code
     */
    virtual msg_t selftest()
    {
      return NOT_IMPLEMENTED;
    }
#else
    static msg_t selftest()
    {
      return NOT_IMPLEMENTED;
    }
#endif

  };

}

#endif /* AMIRO_BASE_SENSOR_H_ */
