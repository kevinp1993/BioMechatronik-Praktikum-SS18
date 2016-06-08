/*
 * pca9544.hpp
 *
 *  Created on: Jun 6, 2014
 *      Author: Robert ABEL
 */

#ifndef AMIRO_PCA9544_HPP_
#define AMIRO_PCA9544_HPP_

#include <ch.hpp>
#include <hal.h>
#include <amiro/bus/i2c/I2CParams.hpp>
#include <amiro/bus/i2c/I2CMultiplexer.hpp>

/**
 * @brief   AMiRo
 */
namespace amiro {

  class I2CDriver;

  /**
   * @brief   Input/Output driver class
   */
  template <bool ignore_deselect>
  class PCA9544 : public I2CMultiplexer {

    enum { SLA = 0x70u };
    enum { BUS_OFF = 0x00u, BUS_ON = 0x04u };

  public:

    PCA9544(I2CDriver *driver, const uint8_t master_id);
    virtual ~PCA9544() {}

    virtual msg_t select(const uint8_t bus_id);
    virtual msg_t deselect();

    uint8_t getInterruptSource();

    /**
     * Return types of getCheck()
     */
    enum {
      CHECK_OK   = 0x00u,
      CHECK_FAIL = 0x01u,
    };

    /**
     * Checks the multiplexer by writing all possible bit
     * combinations into the MUX register and directly
     * reading from it.
     * TODO Check possible register write corruption if vcnl4020 is running in parallel
     *
     * @return [CHECK_OK | CHECK_FAIL]
     */
    uint8_t getCheck(void);

  private:

    uint8_t master_id;
    I2CTxParams tx_params;
    I2CRxParams rx_params;
    uint8_t selected;

  };

} /* amiro */

#include <components/bus/i2c/mux/pca9544.tpp>

#endif /* AMIRO_PCA9544_HPP_ */
