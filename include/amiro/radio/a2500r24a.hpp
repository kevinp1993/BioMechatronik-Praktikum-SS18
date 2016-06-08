#ifndef A2500R24A_HPP_
#define A2500R24A_HPP_

#include <ch.hpp>

namespace amiro {

class HWSPIDriver;

class A2500R24A : public chibios_rt::BaseStaticThread<256> {

 public:

  struct A2500R24AConfig {
  };

  /**
   * Return types of getCheck()
   */
  enum {
    CHECK_OK = 0x00u,
    CHECK_FAIL = 0x01u,
  };

  //private:

  struct registers {
    uint8_t IOCFG2;
    uint8_t IOCFG1;
    uint8_t IOCFG0;
    uint8_t FIFOTHR;
    uint8_t SYNC1;
    uint8_t SYNC0;
    uint8_t PKTLEN;
    uint8_t PKTCTRL1;
    uint8_t PKTCTRL0;
    uint8_t ADDR;
    uint8_t CHANNR;
    uint8_t FSCTRL1;
    uint8_t FSCTRL0;
    uint8_t FREQ2;
    uint8_t FREQ1;
    uint8_t FREQ0;
    uint8_t MDMCFG4;
    uint8_t MDMCFG3;
    uint8_t MDMCFG2;
    uint8_t MDMCFG1;
    uint8_t MDMCFG0;
    uint8_t DEVIATN;
    uint8_t MCSM2;
    uint8_t MCSM1;
    uint8_t MCSM0;
    uint8_t FOCCFG;
    uint8_t BSCFG;
    uint8_t AGCCTRL2;
    uint8_t AGCCTRL1;
    uint8_t AGCCTRL0;
    uint8_t WOREVT1;
    uint8_t WOREVT0;
    uint8_t WORCTRL;
    uint8_t FREND1;
    uint8_t FREND0;
    uint8_t FSCAL3;
    uint8_t FSCAL2;
    uint8_t FSCAL1;
    uint8_t FSCAL0;
    uint8_t RCCTRL1;
    uint8_t RCCTRL0;
    uint8_t FSTEST;
    uint8_t PTEST;
    uint8_t AGCTEST;
    uint8_t TEST2;
    uint8_t TEST1;
    uint8_t TEST0;
    uint8_t reserved_0x2F;
  }__attribute__((packed));

  struct registerStrobe {
    uint8_t configurationRegister[offsetof(A2500R24A::registers, reserved_0x2F)+1];
    uint8_t SRES;
    uint8_t SFSTXON;
    uint8_t SXOFF;
    uint8_t SCAL;
    uint8_t SRX;
    uint8_t STX;
    uint8_t SIDLE;
    uint8_t reserved_0x37;
    uint8_t SWOR;
    uint8_t SPWD;
    uint8_t SFRX;
    uint8_t SFTX;
    uint8_t SWORRST;
    uint8_t SNOP;
    uint8_t PATABLE;
    uint8_t RX_TX_FIFO;
  }__attribute__((packed));

  struct registerStatus {
    uint8_t configurationRegister[offsetof(A2500R24A::registers, reserved_0x2F)+1];
    uint8_t PARTNUM;
    uint8_t VERSION;
    uint8_t FREQEST;
    uint8_t LQI;
    uint8_t RSSI;
    uint8_t MARCSTATE;
    uint8_t WORTIME1;
    uint8_t WORTIME0;
    uint8_t PKTSTATUS;
    uint8_t VCO_VC_DAC;
    uint8_t TXBYTES;
    uint8_t RXBYTES;
    uint8_t RCCTRL1_STATUS;
    uint8_t RCCTRL0_STATUS;
  }__attribute__((packed));

  enum {
    A2500R24A_PARTNUM = 0x80u,
    A2500R24A_VERSION = 0x03u,
  };

  enum {
    // For register access below address 0x30 the burst mode is available
    // to read multiple bytes
    SPI_BURST_ENA     = 0x40u,
    SPI_BURST_DIS     = 0x00u,
    // For addresses above (and including) 0x30 either the status or strobe access
    // has to be choosen, instead of burst mode
    SPI_STATUS_ACCESS = 0x40u,
    SPI_STROBE_ACCESS = 0x00u,
    // Read or write the register address
    SPI_READ      = 0x80u,
    SPI_WRITE     = 0x00u,
  };

 public:
  A2500R24A(HWSPIDriver* driver);
  virtual ~A2500R24A();

  chibios_rt::EvtSource* getEventSource();
  msg_t configure(A2500R24AConfig* config);

  /**
   * Check the presence of the radio chip by reading
   * the identifier register and comparing it to the standard
   * value
   */
  uint8_t getCheck();

 protected:
  virtual msg_t main();

 private:
  inline void updateSensorData();

 private:

  HWSPIDriver* driver;
  chibios_rt::EvtSource eventSource;

};

} /* amiro */

#endif /* A2500R24A_HPP_ */
