#ifndef _BOARD_H_
#define _BOARD_H_

/*
 * Setup for AMiRo LightRing board.
 */

/*
 * Board identifier.
 */
#define BOARD_LIGHT_RING
#define BOARD_NAME              "AMiRo LightRing"
#define BOARD_VERSION           "1.0"

/*
 * Board frequencies.
 */
#define STM32_LSECLK            0
#define STM32_HSECLK            8000000

/*
 * MCU type as defined in the ST header file stm32f1xx.h.
 */
#define STM32F10X_HD

/*
 * IO pins assignments.
 */
#define GPIOA_LASER_RX          2
#define GPIOA_LASER_TX          3
#define GPIOA_LIGHT_BLANK       4
#define GPIOA_LIGHT_SCLK        5
#define GPIOA_LIGHT_MOSI        7
#define GPIOA_PROG_RX           9
#define GPIOA_PROG_TX           10
#define GPIOA_CAN_RX            11
#define GPIOA_CAN_TX            12

#define GPIOB_LASER_EN          2
#define GPIOB_LASER_OC_N        5
#define GPIOB_SYS_UART_DN       6
#define GPIOB_WL_GDO2           8
#define GPIOB_WL_GDO0           9
#define GPIOB_MEM_SCL           10
#define GPIOB_MEM_SDA           11
#define GPIOB_WL_SS_N           12
#define GPIOB_WL_SCLK           13
#define GPIOB_WL_MISO           14
#define GPIOB_WL_MOSI           15

#define GPIOC_LIGHT_XLAT        4
#define GPIOC_SYS_UART_RX       10
#define GPIOC_SYS_UART_TX       11
#define GPIOC_SYS_PD_N          14

#define GPIOD_OSC_IN            0
#define GPIOD_OSC_OUT           1
#define GPIOD_SYS_INT_N         2

/*
 * I/O ports initial setup, this configuration is established soon after reset
 * in the initialization code.
 */
#define PIN_MODE_INPUT(n)               (0x4U << (((n) % 8) * 4))
#define PIN_MODE_INPUT_PULLX(n)         (0x8U << (((n) % 8) * 4))
#define PIN_MODE_INPUT_ANALOG(n)        (0x0U << (((n) % 8) * 4))
/* Push Pull output 50MHz */
#define PIN_MODE_OUTPUT_PUSHPULL(n)     (0x3U << (((n) % 8) * 4))
/* Open Drain output 50MHz */
#define PIN_MODE_OUTPUT_OPENDRAIN(n)    (0x7U << (((n) % 8) * 4))
/* Alternate Push Pull output 50MHz */
#define PIN_MODE_ALTERNATE_PUSHPULL(n)  (0xbU << (((n) % 8) * 4))
/* Alternate Open Drain output 50MHz */
#define PIN_MODE_ALTERNATE_OPENDRAIN(n) (0xfU << (((n) % 8) * 4))

/*
 * Port A setup.
 */
#define VAL_GPIOACRL            (PIN_MODE_INPUT_PULLX(0) | \
                                 PIN_MODE_INPUT_PULLX(1) | \
                                 PIN_MODE_ALTERNATE_PUSHPULL(GPIOA_LASER_RX) | \
                                 PIN_MODE_INPUT_PULLX(GPIOA_LASER_TX) | \
                                 PIN_MODE_OUTPUT_PUSHPULL(GPIOA_LIGHT_BLANK) | \
                                 PIN_MODE_ALTERNATE_PUSHPULL(GPIOA_LIGHT_SCLK) | \
                                 PIN_MODE_INPUT_PULLX(6) | \
                                 PIN_MODE_ALTERNATE_PUSHPULL(GPIOA_LIGHT_MOSI))
#define VAL_GPIOACRH            (PIN_MODE_INPUT_PULLX(8) | \
                                 PIN_MODE_ALTERNATE_PUSHPULL(GPIOA_PROG_RX) | \
                                 PIN_MODE_INPUT_PULLX(GPIOA_PROG_TX) | \
                                 PIN_MODE_INPUT(GPIOA_CAN_RX) | \
                                 PIN_MODE_ALTERNATE_PUSHPULL(GPIOA_CAN_TX) | \
                                 PIN_MODE_INPUT_PULLX(13) | \
                                 PIN_MODE_INPUT_PULLX(14) | \
                                 PIN_MODE_INPUT_PULLX(15))
#define VAL_GPIOAODR            0xFFFF

/*
 * Port B setup.
 */
#define VAL_GPIOBCRL            (PIN_MODE_INPUT_PULLX(0) | \
                                 PIN_MODE_INPUT_PULLX(1) | \
                                 PIN_MODE_OUTPUT_PUSHPULL(GPIOB_LASER_EN) | \
                                 PIN_MODE_INPUT_PULLX(3) | \
                                 PIN_MODE_INPUT_PULLX(4) | \
                                 PIN_MODE_INPUT(GPIOB_LASER_OC_N) | \
                                 PIN_MODE_OUTPUT_OPENDRAIN(GPIOB_SYS_UART_DN) | \
                                 PIN_MODE_INPUT_PULLX(7))
#define VAL_GPIOBCRH            (PIN_MODE_INPUT_PULLX(GPIOB_WL_GDO2) | \
                                 PIN_MODE_INPUT_PULLX(GPIOB_WL_GDO0) | \
                                 PIN_MODE_ALTERNATE_OPENDRAIN(GPIOB_MEM_SCL) | \
                                 PIN_MODE_ALTERNATE_OPENDRAIN(GPIOB_MEM_SDA) | \
                                 PIN_MODE_OUTPUT_PUSHPULL(GPIOB_WL_SS_N) | \
                                 PIN_MODE_ALTERNATE_PUSHPULL(GPIOB_WL_SCLK) | \
                                 PIN_MODE_INPUT(GPIOB_WL_MISO) | \
                                 PIN_MODE_ALTERNATE_PUSHPULL(GPIOB_WL_MOSI))
#define VAL_GPIOBODR            0xFFFB /* initially LASER_EN is deactivated */

/*
 * Port C setup.
 */
#define VAL_GPIOCCRL            (PIN_MODE_INPUT_PULLX(0) | \
                                 PIN_MODE_INPUT_PULLX(1) | \
                                 PIN_MODE_INPUT_PULLX(2) | \
                                 PIN_MODE_INPUT_PULLX(3) | \
                                 PIN_MODE_OUTPUT_PUSHPULL(GPIOC_LIGHT_XLAT) | \
                                 PIN_MODE_INPUT_PULLX(5) | \
                                 PIN_MODE_INPUT_PULLX(6) | \
                                 PIN_MODE_INPUT_PULLX(7))
#define VAL_GPIOCCRH            (PIN_MODE_INPUT_PULLX(8) | \
                                 PIN_MODE_INPUT_PULLX(9) | \
                                 PIN_MODE_INPUT(GPIOC_SYS_UART_RX) | \
                                 PIN_MODE_INPUT(GPIOC_SYS_UART_TX) | \
                                 PIN_MODE_INPUT_PULLX(12) | \
                                 PIN_MODE_INPUT_PULLX(13) | \
                                 PIN_MODE_OUTPUT_OPENDRAIN(GPIOC_SYS_PD_N) | \
                                 PIN_MODE_INPUT_PULLX(15))
#define VAL_GPIOCODR            0xFFFF

/*
 * Port D setup.
 */
#define VAL_GPIODCRL            (PIN_MODE_INPUT(GPIOD_OSC_IN) | \
                                 PIN_MODE_INPUT(GPIOD_OSC_OUT) | \
                                 PIN_MODE_OUTPUT_OPENDRAIN(GPIOD_SYS_INT_N) | \
                                 PIN_MODE_INPUT_PULLX(3) | \
                                 PIN_MODE_INPUT_PULLX(4) | \
                                 PIN_MODE_INPUT_PULLX(5) | \
                                 PIN_MODE_INPUT_PULLX(6) | \
                                 PIN_MODE_INPUT_PULLX(7))
#define VAL_GPIODCRH            0x88888888
#define VAL_GPIODODR            0xFFFB /* initially SYS_INT_N indicates that the OS is busy */

/*
 * Port E setup.
 */
#define VAL_GPIOECRL            0x88888888 /*  PE7...PE0 */
#define VAL_GPIOECRH            0x88888888 /* PE15...PE8 */
#define VAL_GPIOEODR            0xFFFF

/*
 * Port F setup.
 */
#define VAL_GPIOFCRL            0x88888888 /*  PF7...PF0 */
#define VAL_GPIOFCRH            0x88888888 /* PF15...PF8 */
#define VAL_GPIOFODR            0xFFFF

/*
 * Port G setup.
 */
#define VAL_GPIOGCRL            0x88888888 /*  PG7...PG0 */
#define VAL_GPIOGCRH            0x88888888 /* PG15...PG8 */
#define VAL_GPIOGODR            0xFFFF

#if !defined(_FROM_ASM_)
#ifdef __cplusplus
extern "C" {
#endif
  void boardInit(void);
  void boardRequestShutdown(void);
  void boardStandby(void);
  void boardClearI2CBus(const uint8_t scl_pad, const uint8_t sda_pad);
#ifdef __cplusplus
}
#endif
#endif /* _FROM_ASM_ */

#endif /* _BOARD_H_ */
