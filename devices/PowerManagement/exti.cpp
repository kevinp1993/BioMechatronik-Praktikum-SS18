#include <ch.hpp>
#include <hal.h>

#include <board.h>

#include "exti.hpp"

volatile uint32_t shutdown_now = 0x00000000u;
volatile uint32_t pathdc_change = 0x00000000u;

EXTConfig extcfg = {

  {
    /* channel 0 */
    {
      /* mode */ EXT_CH_MODE_DISABLED,
      /* cb   */ NULL,
    },
    /* channel 1 */
    {
      /* mode */ EXT_CH_MODE_DISABLED,
      /* cb   */ NULL,
    },
    /* channel 2 */
    {
      /* mode */ EXT_CH_MODE_DISABLED,
      /* cb   */ NULL,
    },
    /* channel 3 */
    {
      /* mode */ EXT_CH_MODE_DISABLED,
      /* cb   */ NULL,
    },
    /* channel 4 */
    {
      /* mode */ EXT_CH_MODE_DISABLED,
      /* cb   */ NULL,
    },
    /* channel 5 */
    {
      /* mode */ EXT_CH_MODE_DISABLED,
      /* cb   */ NULL,
    },
    /* channel 6 */
    {
      /* mode */ EXT_CH_MODE_DISABLED,
      /* cb   */ NULL,
    },
    /* channel 7 */
    {
      /* mode */ EXT_CH_MODE_DISABLED,
      /* cb   */ NULL,
    },
    /* channel 8 */
    {
      /* mode */ EXT_MODE_GPIOC | EXT_CH_MODE_AUTOSTART | EXT_CH_MODE_BOTH_EDGES,
      /* cb   */ pathdc_cb,
    },
    /* channel 9 */
    {
      /* mode */ EXT_CH_MODE_DISABLED,
      /* cb   */ NULL,
    },
    /* channel A */
    {
      /* mode */ EXT_CH_MODE_DISABLED,
      /* cb   */ NULL,
    },
    /* channel B */
    {
      /* mode */ EXT_CH_MODE_DISABLED,
      /* cb   */ NULL,
    },
    /* channel C */
    {
      /* mode */ EXT_CH_MODE_DISABLED,
      /* cb   */ NULL,
    },
    /* channel D */
    {
      /* mode */ EXT_MODE_GPIOC | EXT_CH_MODE_AUTOSTART | EXT_CH_MODE_FALLING_EDGE,
      /* cb   */ power_down_cb,
    },
    /* channel E */
    {
      /* mode */ EXT_CH_MODE_DISABLED,
      /* cb   */ NULL,
    },
    /* channel F */
    {
      /* mode */ EXT_CH_MODE_DISABLED,
      /* cb   */ NULL,
    },
    /* channel 10 */
    {
      /* mode */ EXT_CH_MODE_DISABLED,
      /* cb   */ NULL,
    },
    /* channel 11 */
    {
      /* mode */ EXT_CH_MODE_DISABLED,
      /* cb   */ NULL,
    },
    /* channel 12 */
    {
      /* mode */ EXT_CH_MODE_DISABLED,
      /* cb   */ NULL,
    },
    /* channel 13 */
    {
      /* mode */ EXT_CH_MODE_DISABLED,
      /* cb   */ NULL,
    },
    /* channel 14 */
    {
      /* mode */ EXT_CH_MODE_DISABLED,
      /* cb   */ NULL,
    },
    /* channel 15 */
    {
      /* mode */ EXT_CH_MODE_DISABLED,
      /* cb   */ NULL,
    },
    /* channel 16 */
    {
      /* mode */ EXT_CH_MODE_DISABLED,
      /* cb   */ NULL,
    },
  }
};

void power_down_cb(EXTDriver *extp, expchannel_t channel) {

  (void) extp;
  (void) channel;
  // make sure we don't accidentally power down when somebody tries to wake us
  if (!palReadPad(GPIOC, GPIOC_SYS_PD_N))
    palWritePad(GPIOC, GPIOC_SYS_INT_N, PAL_LOW); // indicate that the module needs some time to shut down
    shutdown_now = 5; // = SHUTDOWN_HANDLE_REQUEST in main.cpp
}

void pathdc_cb(EXTDriver *extp, expchannel_t channel) {

  (void) extp;
  (void) channel;

  pathdc_change = ~0u;

}
