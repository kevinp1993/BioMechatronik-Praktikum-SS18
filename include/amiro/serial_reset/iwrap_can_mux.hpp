#ifndef IWRAP_CAN_MUX_HPP_
#define IWRAP_CAN_MUX_HPP_

#include <ch.hpp>

#define BLUETOOTH_START_FRAME   0xBF
/** \brief Define bluetooth link address to iWRAP control */
#define BLUETOOTH_LINK          0x00
/** \brief Define bluetooth frame flags */
#define BLUETOOTH_FLAGS         0x00
/** \brief Define nLINK = {LINK} XOR 0xFF */
#define BLUETOOTH_NLINK         0xFF

/* States of bluetooth communication */
/** \brief bluetooth waiting for start frame signal */
#define BLUETOOTH_STATUS_START    0
/** \brief bluetooth waiting for link address */
#define BLUETOOTH_STATUS_LINK     1
/** \brief bluetooth waiting for frame flags */
#define BLUETOOTH_STATUS_FLAGS    2
/** \brief bluetooth waiting for data length */
#define BLUETOOTH_STATUS_LENGTH   3
/** \brief bluetooth waiting for data */
#define BLUETOOTH_STATUS_DATA     4
/** \brief bluetooth waiting for frame end (nLINK) */
#define BLUETOOTH_STATUS_END      5

#define OFFSET_TIME_RESET_MS                2000
#define INPUT_BL_RESET_ADDITIONAL_BYTES     0
#define INPUT_BL_RESET_LENGTH               3

namespace amiro {

  class IwrapCanMux : public chibios_rt::BaseSequentialStreamInterface {
  public:
    IwrapCanMux(SerialDriver *_sd_, CANDriver *can, const uint8_t boardId);
    virtual ~IwrapCanMux();

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

//    void checkBluetoothStatus(void)
    void restartSystem(void);
    bool checkByteForBLReset(uint8_t *inputs, size_t size);
    msg_t sendViaCan(uint32_t id, uint8_t *inputs, size_t size);

    uint8_t inputAddBytes[INPUT_BL_RESET_ADDITIONAL_BYTES];    // Buffer for additional bytes.
    uint8_t resetStatus;      // Status of reset
    uint8_t resetAddBytes;    // Counter of additional bytes
    uint8_t bluetoothStatus;

    SerialDriver *sd_;
    CANDriver *canDriver;
    uint8_t myID;
    volatile uint8_t queryShellID;
    volatile uint8_t replyShellID;

    CANTxFrame txmsg;
    uint8_t inputChar[16] = {};
  };

} /* amiro */

#endif /* IWRAP_CAN_MUX_HPP_ */


