#ifndef SERIAL_CAN_MUX_HPP_
#define SERIAL_CAN_MUX_HPP_

#include <ch.hpp>

#define OFFSET_TIME_RESET_MS                2000
#define INPUT_BL_RESET_ADDITIONAL_BYTES     0
#define INPUT_BL_RESET_LENGTH               3

namespace amiro {

  class SerialCanMux : public chibios_rt::BaseSequentialStreamInterface {
  public:
    SerialCanMux(SerialDriver *_sd_, CANDriver *can, const uint8_t boardId);
    virtual ~SerialCanMux();

    void convCan2Serial(uint8_t *inputs, size_t n);
    void sendSwitchCmd(uint8_t setid);
    void rcvSwitchCmd(uint8_t setid);

    size_t write(const uint8_t *bp, size_t n);
    size_t read(uint8_t *bp, size_t n);
    msg_t put(uint8_t b);
    msg_t get(void);

  protected:

  private:
    /* Reset command. */
    const uint8_t inputBLReset[3] = {0x02, 0xFF, 0x00};
//    const uint8_t inputchshell[3] = {0x00, 0x68, 0x0D};
//    const uint8_t inputsh_board[8] = {0x73, 0x68, 0x5F, 0x62, 0x6F, 0x61, 0x72, 0x6C};

    void restartSystem(void);
    bool checkByteForBLReset(uint8_t *inputs, size_t size);
    msg_t sendViaCan(uint32_t id, uint8_t *inputs, size_t size);

    uint8_t inputAddBytes[INPUT_BL_RESET_ADDITIONAL_BYTES];    // Buffer for additional bytes.
    uint8_t resetStatus;      // Status of reset
    uint8_t resetAddBytes;    // Counter of additional bytes

    SerialDriver *sd_;
    CANDriver *canDriver;
    uint8_t myID;
    volatile uint8_t queryShellID;
    volatile uint8_t replyShellID;

    CANTxFrame txmsg;
    uint8_t inputChar[16] = {};
  };

} /* amiro */

#endif /* SERIAL_CAN_MUX_HPP_ */


