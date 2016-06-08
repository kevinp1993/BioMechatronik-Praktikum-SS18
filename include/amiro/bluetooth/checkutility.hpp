#ifndef AMIRO_WT12_CHECKUTILITY_H_
#define AMIRO_WT12_CHECKUTILITY_H_

#include <ch.hpp>

namespace amiro {

  class WT12CheckUtility
  {
  public:
    enum Constants {
      MUX_StartOfFrame = 0xBFu
    };

  private:
    struct Message {
      struct Head {
        uint8_t start_of_frame;
        uint8_t link_id;
        uint8_t flags;
        uint16_t length;
      };

      Head head;
      char* message;
      uint8_t n_link;
    };

  public:
    explicit WT12CheckUtility();
    ~WT12CheckUtility();

    void reset();
    bool waitForReady(const bool print = false);
    uint8_t selftest();
    void msgPrint(const char* message, const uint16_t length);
    void msgPrintMulti(const char* message_buf, const uint16_t* length_buf, const uint16_t num_messages);
    void sendMessage(char* message, uint8_t link_id = 0xFF);
    void sendMessageRaw(const char* message);
    uint16_t receiveMessage(char* message, const uint16_t max_length);
    uint16_t receiveMessages(char* message_buf, const uint16_t msg_buf_size, uint16_t* length_buf, const uint16_t len_buf_size, const char* last_msg, const char* filter_in = "", const char* filter_out = NULL);

    void receiveRaw(uint8_t* buffer, uint16_t num_bytes, const bool print = false);
    void receiveRawLine(const bool print = true);
    void clearMessageBuffer(const bool print = false);

  };
}

#endif // AMIRO_WT12_CHECKUTILITY_H_
