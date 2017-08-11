#include <amiro/bluetooth/checkutility.hpp>

#include <chprintf.h>
#include <hal.h>

#include <cstring>

using namespace chibios_rt;
using namespace amiro;

WT12CheckUtility::WT12CheckUtility()
{}

WT12CheckUtility::~WT12CheckUtility()
{}

void
WT12CheckUtility::reset()
{
  palWritePad(GPIOC, GPIOC_BT_RST, PAL_HIGH);
  BaseThread::sleep(MS2ST(10));
  palWritePad(GPIOC, GPIOC_BT_RST, PAL_LOW);

  return;
}

bool
WT12CheckUtility::waitForReady(const bool print)
{
  char buffer[8];
  const uint16_t length = this->receiveMessage(buffer, 8);

  if (print) {
    msgPrint(buffer, length);
  }

  return (strcmp(buffer, "READY.\r\n") == 0);
}

uint8_t
WT12CheckUtility::selftest()
{
  return 0;
}

void
WT12CheckUtility::msgPrint(const char* message, const uint16_t length)
{
  if (length > 0)
  {
    chSequentialStreamWrite((BaseSequentialStream*)&global.sercanmux1, (uint8_t*)message, length);
//    chprintf((BaseSequentialStream*) &global.sercanmux1, "[%d]\n", length);
  }
  return;
}

void
WT12CheckUtility::msgPrintMulti(const char* message_buf, const uint16_t* length_buf, const uint16_t num_messages)
{
  uint16_t pos = 0;
  for (uint16_t i = 0; i < num_messages; ++i)
  {
    this->msgPrint(&message_buf[pos], length_buf[i]);
    pos += length_buf[i];
  }

  return;
}

void
WT12CheckUtility::sendMessage(char* message, uint8_t link_id)
{
  const uint16_t length = strlen(message);
  if (length >= 1 << 10) {
    chprintf((BaseSequentialStream*) &global.sercanmux1, "%s(%d): ERROR: message too long!\n", __FILE__ , __LINE__);
    return;
  }

  Message msg;
  msg.head.start_of_frame = MUX_StartOfFrame;
  msg.head.link_id = link_id;
  msg.head.flags = 0x00u;
  msg.head.length = length;
  msg.message = message;
  msg.n_link = ~link_id;

  chSequentialStreamPut((BaseSequentialStream*)&SD3, msg.head.start_of_frame);
  chSequentialStreamPut((BaseSequentialStream*)&SD3, msg.head.link_id);
  chSequentialStreamPut((BaseSequentialStream*)&SD3, msg.head.flags | (msg.head.length >> 8));
  chSequentialStreamPut((BaseSequentialStream*)&SD3, msg.head.length & 0xFFu);
  chSequentialStreamWrite((BaseSequentialStream*)&SD3, (uint8_t*)msg.message, msg.head.length);
  chSequentialStreamPut((BaseSequentialStream*)&SD3, msg.n_link);

  return;
}

void
WT12CheckUtility::sendMessageRaw(const char* message)
{
  chSequentialStreamWrite((BaseSequentialStream*)&SD3, (uint8_t*)message, strlen(message));
  return;
}

uint16_t
WT12CheckUtility::receiveMessage(char* message, const uint16_t max_length)
{
  Message msg;
  msg.message = message;

  msg.head.start_of_frame = chSequentialStreamGet((BaseSequentialStream*)&SD3);
  msg.head.link_id = chSequentialStreamGet((BaseSequentialStream*)&SD3);
  msg.head.flags = chSequentialStreamGet((BaseSequentialStream*)&SD3);
  msg.head.length = (msg.head.flags & 0x03u) << 8;
  msg.head.flags = msg.head.flags >> 2;
  msg.head.length += uint8_t(chSequentialStreamGet((BaseSequentialStream*)&SD3));

  if (msg.head.length > max_length)
  {
    chprintf((BaseSequentialStream*) &global.sercanmux1, "%s(%d): ERROR: message too long (%d of max %d bytes)!\n", __FILE__ , __LINE__, msg.head.length, max_length);

    for (uint16_t i = 0; i < msg.head.length; ++i) {
      chSequentialStreamGet((BaseSequentialStream*)&SD3);
    }
    msg.head.length = 0;
  }
  else
  {
    chSequentialStreamRead((BaseSequentialStream*)&SD3, (uint8_t*)msg.message, msg.head.length);
  }

  msg.n_link = chSequentialStreamGet((BaseSequentialStream*)&SD3);

  if (uint8_t(msg.head.link_id ^ msg.n_link) != 0xFFu)
  {
    chprintf((BaseSequentialStream*) &global.sercanmux1, "%s(%d): ERROR: message link id does not match (0x%02X vs 0x%02X)!\n", __FILE__ , __LINE__, msg.head.link_id, msg.n_link);
  }

  return msg.head.length;
}

uint16_t
WT12CheckUtility::receiveMessages(char* message_buf, const uint16_t msg_buf_size, uint16_t* length_buf, const uint16_t len_buf_size, const char* last_msg, const char* filter_in, const char* filter_out)
{
  if (msg_buf_size == 0 || len_buf_size == 0) {
    return 0;
  }

  for (uint16_t i = 0; i < len_buf_size; ++i) {
    length_buf[i] = 0;
  }

  uint16_t msg_pos = 0;
  uint16_t msg_count = 0;
  while (true)
  {
    length_buf[msg_count] = this->receiveMessage(&message_buf[msg_pos], msg_buf_size-msg_pos);

//    if (msg_count >= 0) {
//      msgPrint(&message_buf[msg_pos], length_buf[msg_count]);
//    }

    if (length_buf[msg_count] == 0) {
      chprintf((BaseSequentialStream*) &global.sercanmux1, "%s(%d): WARNING: receiving aborted after %d messages (%d bytes)\n", __FILE__ , __LINE__, msg_count, msg_pos);
      return msg_count;
    }

    if (strncmp(&message_buf[msg_pos], last_msg, strlen(last_msg)) == 0) {
      return msg_count+1;
    }

    if (strncmp(&message_buf[msg_pos], filter_in, strlen(filter_in)) == 0)
    {
      if (filter_out == NULL || strncmp(&message_buf[msg_pos], filter_out, strlen(filter_out)) != 0)
      {
        msg_pos += length_buf[msg_count];
        ++msg_count;
      }
    }

    if (msg_count > len_buf_size) {
      chprintf((BaseSequentialStream*) &global.sercanmux1, "%s(%d): ERROR: maximum messages received (%d)!\n", __FILE__ , __LINE__, len_buf_size);
      return len_buf_size;
    }

  }

  return 0xFFFFu; // unreachable
}

void
WT12CheckUtility::receiveRaw(uint8_t* buffer, uint16_t num_bytes, const bool print)
{
  uint16_t i;

  for (i = 0; i < num_bytes; ++i)
  {
    buffer[i] = chSequentialStreamGet((BaseSequentialStream*)&SD3);
  }

  if (print)
  {
    for (i = 0; i < num_bytes; ++i)
    {
      chprintf((BaseSequentialStream*) &global.sercanmux1, "%02X ", buffer[i]);
    }
    chprintf((BaseSequentialStream*) &global.sercanmux1, "\n");
  }

  return;
}

void
WT12CheckUtility::receiveRawLine(const bool print)
{
  uint8_t byte = 0;;
  while (byte != '\n')
  {
    byte = chSequentialStreamGet((BaseSequentialStream*) &SD3);
    if (print) {
      chprintf((BaseSequentialStream*) &global.sercanmux1, "%c", byte);
    }
  }

  return;
}

void
WT12CheckUtility::clearMessageBuffer(const bool print)
{
  uint8_t byte;
  while (!sdGetWouldBlock(&SD3))
  {
    byte = sdGet(&SD3);
    //byte = chSequentialStreamGet((BaseSequentialStream*) &SD3);
    if (print) {
      chprintf((BaseSequentialStream*) &global.sercanmux1, "0x%02X <%c>\n", byte, byte);
    }
    BaseThread::sleep(MS2ST(1));
  }

  return;
}

