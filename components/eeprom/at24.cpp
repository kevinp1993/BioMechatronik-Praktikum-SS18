#include <ch.hpp>
#include <hal.h>

#include <type_traits>

#include <amiro/bus/i2c/I2CDriver.hpp>
#include <amiro/bus/i2c/I2CParams.hpp>

#include <amiro/eeprom/at24.hpp>

#define AT24_GET_PAGE_OFFSET(_ADDRESS_, _PAGE_SIZE_) (_ADDRESS_ & (_PAGE_SIZE_ - 1u))

namespace amiro {

const struct BaseFileStreamVMT eeprom_at24_base_file_stream_methods = {

    /* .write       */ AT24::write,
    /* .read        */ AT24::read,
    /* .put         */ 0,
    /* .get         */ 0,
    /* .close       */ AT24::close,
    /* .geterror    */ EEPROM::geterror,
    /* .getsize     */ EEPROM::getsize,
    /* .getposition */ EEPROM::getposition,
    /* .lseek       */ EEPROM::lseek,

};

AT24::
AT24(size_t size, uint8_t page_size, uint16_t max_t_wr, I2CDriver* i2c_driver) :
  EEPROM(&eeprom_at24_base_file_stream_methods, size, page_size, max_t_wr, AT24_SLA, i2c_driver) {

}

AT24::
~AT24() {

}

msg_t
AT24::
poll_ack(void* instance) {

  EEPROM* bfs = (EEPROM*) instance;

  register I2CDriver* bfs_i2c_driver = bfs->i2c_driver;
  register msg_t ret_val = RDY_OK;
  register i2cflags_t i2c_err_flags;

  uint8_t dummy_buf[2];

  I2CRxParams i2c_rxparams;
  i2c_rxparams.addr = bfs->i2c_txparams.addr;
  i2c_rxparams.rxbuf = dummy_buf;
  i2c_rxparams.rxbytes = sizeof(dummy_buf);

  for (register uint16_t wr_time = 0x0000u; wr_time < bfs->max_t_wr; wr_time++) {

    bfs_i2c_driver->acquireBus();

    ret_val = bfs_i2c_driver->masterReceive(&i2c_rxparams);

    i2c_err_flags = bfs->i2c_driver->getErrors();

    bfs_i2c_driver->releaseBus();

    // Wait cycle over
    if (ret_val == RDY_OK)
      break;

    // Check for errors, ignoring only I2CD_ACK_FAILURE
    if (i2c_err_flags & ~I2CD_ACK_FAILURE) {
      break;
    }

    chThdSleepMicroseconds(10);

  }

  // This can only be the case when
  // * timeout reached
  // * i2cError other than I2CD_ACK_FAILURE within time limit
  if (ret_val != RDY_OK)
    bfs->error = i2c_err_flags;

  return ret_val;

}


size_t
AT24::
write(void* instance, const uint8_t* bp, size_t n) {

  EEPROM* bfs = (EEPROM*) instance;

  i2cflags_t i2c_err_flags;
  uint8_t i;
  uint8_t j;
  uint8_t scratchpad[2 + AT24_MAX_PAGE_SIZE];
  msg_t ret_val = RDY_OK;
  fileoffset_t cur_pos = bfs->position;

  register size_t tx_bytes;
  register const uint8_t* ptr = bp;
  register size_t num_bytes;
  register uint8_t page_size = bfs->page_size;
  register I2CTxParams* bfs_i2c_txparams = &bfs->i2c_txparams;

  // If no bytes are to be written, shortcut stop
  if (!n)
    return 0;

  // Clear error
  bfs->error = EEPROM_ERROR_NONE;

#if HAL_USE_I2C

  // number of bytes remaining in page starting at cur_pos
  // b/c of address rollover withing current page
  tx_bytes = AT24_GET_PAGE_OFFSET(cur_pos, page_size);
  // adjust for AT24_MAX_PAGE_SIZE < page_size
  tx_bytes = AT24_MAX_PAGE_SIZE - AT24_GET_PAGE_OFFSET(tx_bytes, AT24_MAX_PAGE_SIZE);

  for (num_bytes = n; num_bytes > 0;) {

    // write address
    i = 0;
    // Support for 16bit-addressable devices
    if (bfs->size > 0x0080u)
      scratchpad[i++] = (cur_pos >> 8) & 0xFFu;

    scratchpad[i++] = cur_pos & 0xFFu;

    // adjust number of bytes to transfer if end of buffer
    if (num_bytes < tx_bytes)
      tx_bytes = num_bytes;

    // copy data
    for (j = 0; j < tx_bytes; j++) {
      scratchpad[i++] = *ptr++;
    }

    // acknowledge polling

    // acknowledge polling and
    // address device
    for (register uint16_t wr_time = 0x0000u; wr_time < bfs->max_t_wr; wr_time++) {

      bfs->i2c_driver->acquireBus();

      bfs_i2c_txparams->txbuf = scratchpad;
      bfs_i2c_txparams->txbytes = i;
      bfs_i2c_txparams->rxbytes = 0;

      ret_val = bfs->i2c_driver->masterTransmit(bfs_i2c_txparams);

      i2c_err_flags = bfs->i2c_driver->getErrors();

      bfs->i2c_driver->releaseBus();

      // Write cycle over
      if (ret_val == RDY_OK)
        break;

      // Check for errors, ignoring only I2CD_ACK_FAILURE
      if (i2c_err_flags & ~I2CD_ACK_FAILURE) {
        break;
      }

      chThdSleepMicroseconds(10);

    }

    // This can only be the case when
    // * timeout reached
    // * i2cError other than I2CD_ACK_FAILURE within time limit
    if (ret_val != RDY_OK)
      break;

    cur_pos += tx_bytes;
    num_bytes -= tx_bytes;
    tx_bytes = AT24_MAX_PAGE_SIZE;

  }

  if (ret_val != RDY_OK)
    bfs->error = i2c_err_flags;

#endif

  bfs->position = cur_pos;

  // number of bytes to transfer - number of bytes not transferred
  return n - num_bytes;

}

size_t
AT24::
read(void* instance, uint8_t* bp, size_t n) {

  EEPROM* bfs = (EEPROM*) instance;

  uint8_t i;
  uint8_t scratchpad[2];
  msg_t ret_val;
  register I2CTxParams* bfs_i2c_txparams = &bfs->i2c_txparams;

  // If no bytes are to be read, shortcut stop
  if (!n)
    return 0;

  // Clear error
  bfs->error = EEPROM_ERROR_NONE;

#if HAL_USE_I2C

  // Fill address buffer
  i = 0;
  // Support for 16bit-addressable devices
  if (bfs->size > 0x0080u)
    scratchpad[i++] = (bfs->position >> 8) & 0xFFu;

  scratchpad[i++] = bfs->position & 0xFFu;

  // if device does not answer within timeout, don't read anything
  if (poll_ack(bfs) != RDY_OK)
    return 0;

  bfs->i2c_driver->acquireBus();

  bfs_i2c_txparams->txbuf = scratchpad;
  bfs_i2c_txparams->txbytes = i;
  bfs_i2c_txparams->rxbuf = bp;
  bfs_i2c_txparams->rxbytes = n;

  // address device
  // and read data
  ret_val = bfs->i2c_driver->masterTransmit(bfs_i2c_txparams);

  if (ret_val != RDY_OK)
    bfs->error = bfs->i2c_driver->getErrors();

  bfs->i2c_driver->releaseBus();

  // We cannot tell where I²C transfer went wrong
  // therefore report 0 bytes read
  if (ret_val != RDY_OK)
    return 0;

#endif

  return n;

}

/**
 * Close EEPROM device.
 * \note EEPROMs do not support close semantics.
 * \return FILE_OK Closing an EEPROM device will never fail.
 */
uint32_t
AT24::
close(void* instance) {

  (void) instance;
  return FILE_OK;

}

} /* amiro */
