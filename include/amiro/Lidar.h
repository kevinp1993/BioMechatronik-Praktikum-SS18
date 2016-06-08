#ifndef LIDAR_H_
#define LIDAR_H_

// Top view
/*                                                 __________
//                                               /     |     \
//                                              /      O      \
//                                             /  H O K U Y O  \
//                                             |                |
//                                             |                |
//                                              \    U R G     /
// scannedData[NUMBER_OF_STEPS] = END_STEP       \/          \/   STARTING_STEP = scannedData[0]
//                                                \__________/
*/

#include <string.h> // memcpy
#include <stdlib.h>

#include <amiro/bus/spi/HWSPIDriver.hpp>
#include <amiro/Constants.h> // CAN::LIGHT_RING_ID
#include <ch.hpp>
#include <hal.h>
#include <chprintf.h>
#include <evtimer.h>

// DO CONFIGURATION HERE : START
// Important: SD_SPEED_PREFIX + SD_SPEED has to have 6 characters: e.g. 019200 or 115200
#define SD_SPEED_PREFIX   // Prefix < 0     | 0     |        |        |        |        >
#define SD_SPEED 250000 // Baudrate < 19200 | 57600 | 115200 | 250000 | 500000 | 750000 >
#define UPDATE_LIDAR_PERIOD_MSEC MS2ST(1000)  // Periodic sensor value fetch in ms
// Scan acquire message : START
// Uncomment the specific character decoding to make it available
#define USE_2CHAR_CODE
// #define USE_3CHAR_CODE  // TODO Not implemented yet
#define DATA_ACQ_2CHAR_CODE "MS"
#define DATA_ACQ_3CHAR_CODE "MD"
// Important: STARTING_STEP_PREFIX + STARTING_STEP has to have 4 characters: e.g. 0005 0044 0320
#define STARTING_STEP_PREFIX 00
#define STARTING_STEP 44
// Important: END_STEP_PREFIX + END_STEP has to have 4 characters: e.g. 0005 0725 0320
#define END_STEP_PREFIX 0
#define END_STEP 725
#define CLUSTER_COUNT 00
#define SCAN_INTERVALL 0
#define NUMBER_OF_INTERVALL 01
// Scan acquire message : END
// DO CONFIGURATION HERE : END

// Internal calculations and definitions
#if defined(USE_2CHAR_CODE) && defined(USE_3CHAR_CODE)
#  error("USE_2CHAR_CODE and USE_3CHAR_CODE are defined. Only one of them is allowed!")
#elif !defined(USE_2CHAR_CODE) && !defined(USE_3CHAR_CODE)
#  error("USE_2CHAR_CODE and USE_3CHAR_CODE are not defined. Define one of them!")
#elif defined(USE_2CHAR_CODE)
#  define DATA_ACQ_CODE DATA_ACQ_2CHAR_CODE
#  define NUMBER_OF_CHARACTERS (END_STEP - STARTING_STEP + 1) * 2
#elif defined(USE_3CHAR_CODE)
#  define DATA_ACQ_CODE DATA_ACQ_3CHAR_CODE
#  define NUMBER_OF_CHARACTERS (END_STEP - STARTING_STEP + 1) * 3
#endif
#define NUMBER_OF_STEPS (END_STEP - STARTING_STEP + 1)

#define STR_HELPER(x) #x
#define STR(x) STR_HELPER(x)

#define LF "\n"

namespace amiro {

  class Lidar : public chibios_rt::BaseStaticThread<256> {
  public:
    /**
    * Possible setups for the LIDAR: POWER_ONLY (The LIDAR will be powererd, but no serial IO is performed [Need for USB IO]); SERIAL_IO (Power and serial communication via lidar driver)
    */
    enum SETUP {POWER_ONLY, SERIAL_IO};

    /**
    * Writes the last scan into the given array
    *
    * @param scanData Array of size NUMBER_OF_STEPS which will be override
    *                 with the scanned data
    *
    * @return True if scanData holds the current data, False if no data have been copied
    */
    bool_t getScan(uint16_t (&scanData)[NUMBER_OF_STEPS]);

    /**
    * Returns the value NUMBER_OF_STEPS which is the amount of datapoints per scan
    *
    * @return Amount of data in one scan
    */
    uint16_t getNumberOfValues();

  protected:
    virtual msg_t main();

    /**
    * Do a scan and update the datapoints in scanData
    */
    msg_t updateSensorVal();

    /**
    * Print received data until "\n\n" was received
    */
    void printData();

    /**
    * Receive data until "\n\n" and compare it to the given string
    * Note: If the command did not work, use printData() instead
    *       and compare the reply with chapter "9. Response to Invalid Commands"
    *
    * @param compareString String to compare with
    *
    * @return True if string was equal, false if not
    */
    bool_t checkDataString(const char compareString[]);
    /**
    *  Send the command which let the LIDAR send its details
    */
    void printDetails();

    /**
    *  Send the command which let the LIDAR send its specifications
    */
    void printSpecification();

    /**
    *  Send the command which let the LIDAR send its information
    */
    void printInformation();

    /**
    *  Tells you, if the sensor is ready for work
    *
    *  @return True for operability, False if not
    */
    inline bool_t getIsReady();

    /**
    *  Do the two character encoding after the manual
    *
    *  @param char1 High character
    *  @param char2 Low character
    *
    *  @return Encoded data
    */
    inline uint16_t twoCharacterEncoding(uint8_t &char1, uint8_t &char2);

    /**
    *  Blocking function which writes the received character to the given variable
    *
    *  @param data Container for the variable
    *  @param timeoutMs Timeout in millisecond
    *
    *  @return True if data was received, False if timed out (NOTE timeoutMs should by much smaller than UPDATE_LIDAR_PERIOD_MSEC)
    */
    bool_t getData(uint8_t &data, uint32_t timeoutMs);

    /**
    * Container for the scanned data
    *
    * @brief The "+1" character is for holding the last sum which is transmitted
    *        by the LIDAR. It is not used, but it makes the algorithm much easier
    *        we we do not care about it
    */
    static uint8_t scannedData[NUMBER_OF_CHARACTERS + 1];

    /**
    *  Flushes the input queue of the SD2 serial device
    */
    inline void flushSD2InputQueue();

    /**
    * Possible states for the receiver state machine
    */
    enum STATE {SEND_SCAN_CMD, DATA_VALID_CHECK, DATA_START_CHECK, DATA_RECORD, DATA_DECODE, DATA_SHOW, FAIL, FINISH};

    /**
    * Current state of the machine
    */
    STATE step = SEND_SCAN_CMD;

    /**
    * This is the status which comes after the command echo "\n99b\n"
    */
    const uint8_t validScanTag[5] =  {10,57,57,98,10};

    /**
    * Index for the received status
    */
    uint8_t checkStatusIdx = 0;

    /**
    * Index for the received status
    */

    /**
    * Hold the board id
    */
    uint8_t boardId;

    /**
    * Hold the setup configuration for the lidar
    */
    SETUP setup;

    /**
    * Holds the current input
    */
    uint8_t newInput = 0xFF;

    /**
    * Holds the last input
    */
    uint8_t lastInput = 0xFF;

    /**
    * Counter for the received characters of the data points
    */
    uint16_t dataIdx = 0;

    /**
    * Temporary variable which hold the amount of receved bytes
    */
    uint32_t dataCounter = 0;

    /**
    * Holds the information, if scannedData holds the last sensor scan
    */
    bool_t isReady = false;

    /**
    * Holds the information, if the datachunk was not transmitted completely
    */
    bool_t isFail = false;

  private:

    EvTimer evtimer;

  public:
  /**
    * Constructor
    *
    * @param boardId Board ID which can be found in ControllerAreaNetwork.h.
    *                It is used to setup the LIDAR for exactly this board
    */
    Lidar(const uint8_t boardId, Lidar::SETUP setup);

    /**
    * Constructor
    *
    * Disable the LIDAR
    * TODO Why "virtual ~Lidar() = 0;"
    */
    ~Lidar();

  };

}
#endif /* LIDAR_H_ */
