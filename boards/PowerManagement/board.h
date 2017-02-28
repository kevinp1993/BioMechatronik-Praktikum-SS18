#ifndef _BOARD_H_
#define _BOARD_H_

/*
 * Setup for AMiRo PowerManagement board.
 */

/*
 * Board identifier.
 */
#define BOARD_POWER_MANAGEMENT
#define BOARD_NAME              "AMiRo PowerManagement"
#define BOARD_VERSION           "1.1"

/*
 * Board frequencies.
 */
#define STM32_LSECLK            0
#define STM32_HSECLK            8000000

/*
 * Board voltages.
 * Required for performance limits calculation.
 */
#define STM32_VDD               330

/*
 * MCU type as defined in the ST header file stm32f4xx.h.
 */
#define STM32F40_41xxx

/*
 * IO pins assignments.
 */
#define GPIOA_WKUP              0
#define GPIOA_SYS_UART_TX       2
#define GPIOA_SYS_UART_RX       3
#define GPIOA_SYS_SPI_SS0_N     4
#define GPIOA_SYS_SPI_SCLK      5
#define GPIOA_SYS_SPI_MISO      6
#define GPIOA_SYS_SPI_MOSI      7
#define GPIOA_SYS_REG_EN        8
#define GPIOA_PROG_RX           9
#define GPIOA_PROG_TX           10
#define GPIOA_CAN_RX            11
#define GPIOA_CAN_TX            12
#define GPIOA_SYS_SPI_SS1_N     15

#define GPIOB_IR_INT1_N         0
#define GPIOB_VSYS_SENSE        1
#define GPIOB_POWER_EN          2
#define GPIOB_SYS_UART_DN       3
#define GPIOB_CHARGE_STAT2A     4
#define GPIOB_BUZZER            5
#define GPIOB_GAUGE_BATLOW2     6
#define GPIOB_GAUGE_BATGD2_N    7
#define GPIOB_GAUGE_SCL2        8
#define GPIOB_GAUGE_SDA2        9
#define GPIOB_GAUGE_SCL1        10
#define GPIOB_GAUGE_SDA1        11
#define GPIOB_LED               12
#define GPIOB_BT_RTS            13
#define GPIOB_BT_CTS            14
#define GPIOB_SYS_UART_UP       15

#define GPIOC_CHARGE_STAT1A     0
#define GPIOC_GAUGE_BATLOW1     1
#define GPIOC_GAUGE_BATGD1_N    2
#define GPIOC_CHARGE_EN1_N      3
#define GPIOC_IR_INT2_N         4
#define GPIOC_TOUCH_INT_N       5
#define GPIOC_SYS_DONE          6
#define GPIOC_SYS_PROG_N        7
#define GPIOC_PATH_DC           8
#define GPIOC_SYS_SPI_DIR       9
#define GPIOC_BT_RX             10
#define GPIOC_BT_TX             11
#define GPIOC_SYS_INT_N         12
#define GPIOC_SYS_PD_N          13
#define GPIOC_SYS_WARMRST_N     14
#define GPIOC_BT_RST            15

#define GPIOD_CHARGE_EN2_N      2

#define GPIOH_OSC_IN            0
#define GPIOH_OSC_OUT           1

/*
 * I/O ports initial setup, this configuration is established soon after reset
 * in the initialization code.
 */
#define PIN_MODE_INPUT(n)       (0U << ((n) * 2))
#define PIN_MODE_OUTPUT(n)      (1U << ((n) * 2))
#define PIN_MODE_ALTERNATE(n)   (2U << ((n) * 2))
#define PIN_MODE_ANALOG(n)      (3U << ((n) * 2))
#define PIN_OTYPE_PUSHPULL(n)   (0U << (n))
#define PIN_OTYPE_OPENDRAIN(n)  (1U << (n))
#define PIN_OSPEED_2M(n)        (0U << ((n) * 2))
#define PIN_OSPEED_25M(n)       (1U << ((n) * 2))
#define PIN_OSPEED_50M(n)       (2U << ((n) * 2))
#define PIN_OSPEED_100M(n)      (3U << ((n) * 2))
#define PIN_PUDR_FLOATING(n)    (0U << ((n) * 2))
#define PIN_PUDR_PULLUP(n)      (1U << ((n) * 2))
#define PIN_PUDR_PULLDOWN(n)    (2U << ((n) * 2))
#define PIN_AFIO_AF(n, v)       ((v##U) << ((n % 8) * 4))

/*
 * Port A setup.
 */
#define VAL_GPIOA_MODER         (PIN_MODE_INPUT(GPIOA_WKUP) | \
                                 PIN_MODE_INPUT(1) | \
                                 PIN_MODE_ALTERNATE(GPIOA_SYS_UART_TX) | \
                                 PIN_MODE_ALTERNATE(GPIOA_SYS_UART_RX) | \
                                 PIN_MODE_INPUT(GPIOA_SYS_SPI_SS0_N) | \
                                 PIN_MODE_ALTERNATE(GPIOA_SYS_SPI_SCLK) | \
                                 PIN_MODE_ALTERNATE(GPIOA_SYS_SPI_MOSI) | \
                                 PIN_MODE_ALTERNATE(GPIOA_SYS_SPI_MISO) | \
                                 PIN_MODE_INPUT(GPIOA_SYS_SPI_SS1_N) | \
                                 PIN_MODE_ALTERNATE(GPIOA_PROG_RX) | \
                                 PIN_MODE_ALTERNATE(GPIOA_PROG_TX) | \
                                 PIN_MODE_ALTERNATE(GPIOA_CAN_RX) | \
                                 PIN_MODE_ALTERNATE(GPIOA_CAN_TX) | \
                                 PIN_MODE_INPUT(13) | \
                                 PIN_MODE_INPUT(14) | \
                                 PIN_MODE_OUTPUT(GPIOA_SYS_REG_EN))
#define VAL_GPIOA_OTYPER        (PIN_OTYPE_PUSHPULL(GPIOA_SYS_UART_TX) | \
                                 PIN_OTYPE_PUSHPULL(GPIOA_SYS_SPI_SS0_N) | \
                                 PIN_OTYPE_PUSHPULL(GPIOA_PROG_RX) | \
                                 PIN_OTYPE_PUSHPULL(GPIOA_CAN_TX) | \
                                 PIN_OTYPE_PUSHPULL(GPIOA_SYS_REG_EN))
#define VAL_GPIOA_OSPEEDR       0xFFFFFFFF
#define VAL_GPIOA_PUPDR         (PIN_PUDR_FLOATING(GPIOA_WKUP) | \
                                 PIN_PUDR_PULLUP(1) | \
                                 PIN_PUDR_FLOATING(GPIOA_SYS_UART_TX) | \
                                 PIN_PUDR_FLOATING(GPIOA_SYS_UART_RX) | \
                                 PIN_PUDR_FLOATING(GPIOA_SYS_SPI_SS0_N) | \
                                 PIN_PUDR_FLOATING(GPIOA_SYS_SPI_SCLK) | \
                                 PIN_PUDR_FLOATING(GPIOA_SYS_SPI_MOSI) | \
                                 PIN_PUDR_FLOATING(GPIOA_SYS_SPI_MISO) | \
                                 PIN_PUDR_FLOATING(GPIOA_SYS_SPI_SS1_N) | \
                                 PIN_PUDR_FLOATING(GPIOA_PROG_RX) | \
                                 PIN_PUDR_PULLUP(GPIOA_PROG_TX) | \
                                 PIN_PUDR_FLOATING(GPIOA_CAN_RX) | \
                                 PIN_PUDR_FLOATING(GPIOA_CAN_TX) | \
                                 PIN_PUDR_PULLUP(13) | \
                                 PIN_PUDR_PULLUP(14) | \
                                 PIN_PUDR_FLOATING(GPIOA_SYS_REG_EN))

#define VAL_GPIOA_ODR           0xFFFF
#define VAL_GPIOA_AFRL          (PIN_AFIO_AF(GPIOA_SYS_UART_TX, 7) | \
                                 PIN_AFIO_AF(GPIOA_SYS_UART_RX, 7) | \
                                 PIN_AFIO_AF(GPIOA_SYS_SPI_SCLK, 5) | \
                                 PIN_AFIO_AF(GPIOA_SYS_SPI_MISO, 5) | \
                                 PIN_AFIO_AF(GPIOA_SYS_SPI_MOSI, 5))
#define VAL_GPIOA_AFRH          (PIN_AFIO_AF(GPIOA_PROG_RX, 7) | \
                                 PIN_AFIO_AF(GPIOA_PROG_TX, 7) | \
                                 PIN_AFIO_AF(GPIOA_CAN_RX, 9) | \
                                 PIN_AFIO_AF(GPIOA_CAN_TX, 9))

/*
 * Port B setup.
 */
#define VAL_GPIOB_MODER         (PIN_MODE_INPUT(GPIOB_IR_INT1_N) | \
                                 PIN_MODE_ANALOG(GPIOB_VSYS_SENSE) | \
                                 PIN_MODE_OUTPUT(GPIOB_POWER_EN) | \
                                 PIN_MODE_OUTPUT(GPIOB_SYS_UART_DN) | \
                                 PIN_MODE_INPUT(GPIOB_CHARGE_STAT2A) | \
                                 PIN_MODE_ALTERNATE(GPIOB_BUZZER) | \
                                 PIN_MODE_INPUT(GPIOB_GAUGE_BATLOW2) | \
                                 PIN_MODE_INPUT(GPIOB_GAUGE_BATGD2_N) | \
                                 PIN_MODE_ALTERNATE(GPIOB_GAUGE_SCL2) | \
                                 PIN_MODE_ALTERNATE(GPIOB_GAUGE_SDA2) | \
                                 PIN_MODE_ALTERNATE(GPIOB_GAUGE_SCL1) | \
                                 PIN_MODE_ALTERNATE(GPIOB_GAUGE_SDA1) | \
                                 PIN_MODE_OUTPUT(GPIOB_LED) | \
                                 PIN_MODE_ALTERNATE(GPIOB_BT_RTS) | \
                                 PIN_MODE_ALTERNATE(GPIOB_BT_CTS) | \
                                 PIN_MODE_OUTPUT(GPIOB_SYS_UART_UP))
#define VAL_GPIOB_OTYPER        (PIN_OTYPE_PUSHPULL(GPIOB_POWER_EN) | \
                                 PIN_OTYPE_OPENDRAIN(GPIOB_SYS_UART_DN) | \
                                 PIN_OTYPE_PUSHPULL(GPIOB_BUZZER) | \
                                 PIN_OTYPE_OPENDRAIN(GPIOB_GAUGE_SCL2) | \
                                 PIN_OTYPE_OPENDRAIN(GPIOB_GAUGE_SDA2) | \
                                 PIN_OTYPE_OPENDRAIN(GPIOB_GAUGE_SCL1) | \
                                 PIN_OTYPE_OPENDRAIN(GPIOB_GAUGE_SDA1) | \
                                 PIN_OTYPE_OPENDRAIN(GPIOB_LED) | \
                                 PIN_OTYPE_PUSHPULL(GPIOB_BT_CTS) | \
                                 PIN_OTYPE_OPENDRAIN(GPIOB_SYS_UART_UP))
#define VAL_GPIOB_OSPEEDR       0xFFFFFFFF
#define VAL_GPIOB_PUPDR         0x00000000
#define VAL_GPIOB_ODR           0xFFFF
#define VAL_GPIOB_AFRL          (PIN_AFIO_AF(GPIOB_BUZZER, 2))
#define VAL_GPIOB_AFRH          (PIN_AFIO_AF(GPIOB_GAUGE_SCL2, 4) | \
                                 PIN_AFIO_AF(GPIOB_GAUGE_SDA2, 4) | \
                                 PIN_AFIO_AF(GPIOB_GAUGE_SCL1, 4) | \
                                 PIN_AFIO_AF(GPIOB_GAUGE_SDA1, 4) | \
                                 PIN_AFIO_AF(GPIOB_BT_RTS, 7) | \
                                 PIN_AFIO_AF(GPIOB_BT_CTS, 7))

/*
 * Port C setup.
 */
#define VAL_GPIOC_MODER         (PIN_MODE_INPUT(GPIOC_CHARGE_STAT1A) | \
                                 PIN_MODE_INPUT(GPIOC_GAUGE_BATLOW1) | \
                                 PIN_MODE_INPUT(GPIOC_GAUGE_BATGD1_N) | \
                                 PIN_MODE_OUTPUT(GPIOC_CHARGE_EN1_N) | \
                                 PIN_MODE_INPUT(GPIOC_IR_INT2_N) | \
                                 PIN_MODE_INPUT(GPIOC_TOUCH_INT_N) | \
                                 PIN_MODE_INPUT(GPIOC_SYS_DONE) | \
                                 PIN_MODE_OUTPUT(GPIOC_SYS_PROG_N) | \
                                 PIN_MODE_INPUT(GPIOC_PATH_DC) | \
                                 PIN_MODE_OUTPUT(GPIOC_SYS_SPI_DIR) | \
                                 PIN_MODE_ALTERNATE(GPIOC_BT_RX) | \
                                 PIN_MODE_ALTERNATE(GPIOC_BT_TX) | \
                                 PIN_MODE_OUTPUT(GPIOC_SYS_INT_N) | \
                                 PIN_MODE_OUTPUT(GPIOC_SYS_PD_N) | \
                                 PIN_MODE_OUTPUT(GPIOC_SYS_WARMRST_N) | \
                                 PIN_MODE_OUTPUT(GPIOC_BT_RST))
#define VAL_GPIOC_OTYPER        (PIN_OTYPE_OPENDRAIN(GPIOC_CHARGE_EN1_N) | \
                                 PIN_OTYPE_OPENDRAIN(GPIOC_SYS_PROG_N) | \
                                 PIN_OTYPE_OPENDRAIN(GPIOC_SYS_SPI_DIR) | \
                                 PIN_OTYPE_PUSHPULL(GPIOC_BT_RX) | \
                                 PIN_OTYPE_OPENDRAIN(GPIOC_SYS_INT_N) | \
                                 PIN_OTYPE_OPENDRAIN(GPIOC_SYS_PD_N) | \
                                 PIN_OTYPE_OPENDRAIN(GPIOC_SYS_WARMRST_N) | \
                                 PIN_OTYPE_OPENDRAIN(GPIOC_BT_RST))
#define VAL_GPIOC_OSPEEDR       0xFFFFFFFF
#define VAL_GPIOC_PUPDR         0x00000000
#define VAL_GPIOC_ODR           0xEEFF /* emulate open drain for PATH_DC. This is required to prevent accidental shortcuts. Furthermore, pull down SYS_INT_N to indicate the OS is starting */
#define VAL_GPIOC_AFRL          0x00000000
#define VAL_GPIOC_AFRH          (PIN_AFIO_AF(GPIOC_BT_RX, 7) | \
                                 PIN_AFIO_AF(GPIOC_BT_TX, 7))

/*
 * Port D setup.
 */
#define VAL_GPIOD_MODER         (PIN_MODE_INPUT(0) | \
                                 PIN_MODE_INPUT(1) | \
                                 PIN_MODE_OUTPUT(GPIOD_CHARGE_EN2_N) | \
                                 PIN_MODE_INPUT(3) | \
                                 PIN_MODE_INPUT(4) | \
                                 PIN_MODE_INPUT(5) | \
                                 PIN_MODE_INPUT(6) | \
                                 PIN_MODE_INPUT(7) | \
                                 PIN_MODE_INPUT(8) | \
                                 PIN_MODE_INPUT(9) | \
                                 PIN_MODE_INPUT(10) | \
                                 PIN_MODE_INPUT(11) | \
                                 PIN_MODE_INPUT(12) | \
                                 PIN_MODE_INPUT(13) | \
                                 PIN_MODE_INPUT(14) | \
                                 PIN_MODE_INPUT(15))
#define VAL_GPIOD_OTYPER        (PIN_OTYPE_OPENDRAIN(GPIOD_CHARGE_EN2_N))
#define VAL_GPIOD_OSPEEDR       0xFFFFFFFF
#define VAL_GPIOD_PUPDR         (PIN_PUDR_PULLUP(0) | \
                                 PIN_PUDR_PULLUP(1) | \
                                 PIN_PUDR_FLOATING(GPIOD_CHARGE_EN2_N) | \
                                 PIN_PUDR_PULLUP(3) | \
                                 PIN_PUDR_PULLUP(4) | \
                                 PIN_PUDR_PULLUP(5) | \
                                 PIN_PUDR_PULLUP(6) | \
                                 PIN_PUDR_PULLUP(7) | \
                                 PIN_PUDR_PULLUP(8) | \
                                 PIN_PUDR_PULLUP(9) | \
                                 PIN_PUDR_PULLUP(10) | \
                                 PIN_PUDR_PULLUP(11) | \
                                 PIN_PUDR_PULLUP(12) | \
                                 PIN_PUDR_PULLUP(13) | \
                                 PIN_PUDR_PULLUP(14) | \
                                 PIN_PUDR_PULLUP(15))
#define VAL_GPIOD_ODR           0x0FFF
#define VAL_GPIOD_AFRL          0x00000000
#define VAL_GPIOD_AFRH          0x00000000

/*
 * Port E setup.
 */
#define VAL_GPIOE_MODER         0x00000000
#define VAL_GPIOE_OTYPER        0x00000000
#define VAL_GPIOE_OSPEEDR       0xFFFFFFFF
#define VAL_GPIOE_PUPDR         0xFFFFFFFF
#define VAL_GPIOE_ODR           0xFFFF
#define VAL_GPIOE_AFRL          0x00000000
#define VAL_GPIOE_AFRH          0x00000000

/*
 * Port F setup.
 * All input with pull-up.
 */
#define VAL_GPIOF_MODER         0x00000000
#define VAL_GPIOF_OTYPER        0x00000000
#define VAL_GPIOF_OSPEEDR       0xFFFFFFFF
#define VAL_GPIOF_PUPDR         0xFFFFFFFF
#define VAL_GPIOF_ODR           0xFFFF
#define VAL_GPIOF_AFRL          0x00000000
#define VAL_GPIOF_AFRH          0x00000000

/*
 * Port G setup.
 * All input with pull-up.
 */
#define VAL_GPIOG_MODER         0x00000000
#define VAL_GPIOG_OTYPER        0x00000000
#define VAL_GPIOG_OSPEEDR       0xFFFFFFFF
#define VAL_GPIOG_PUPDR         0xFFFFFFFF
#define VAL_GPIOG_ODR           0xFFFF
#define VAL_GPIOG_AFRL          0x00000000
#define VAL_GPIOG_AFRH          0x00000000

/*
 * Port H setup.
 */
#define VAL_GPIOH_MODER         (PIN_MODE_INPUT(GPIOH_OSC_IN) | \
                                 PIN_MODE_INPUT(GPIOH_OSC_OUT) | \
                                 PIN_MODE_INPUT(2) | \
                                 PIN_MODE_INPUT(3) | \
                                 PIN_MODE_INPUT(4) | \
                                 PIN_MODE_INPUT(5) | \
                                 PIN_MODE_INPUT(6) | \
                                 PIN_MODE_INPUT(7) | \
                                 PIN_MODE_INPUT(8) | \
                                 PIN_MODE_INPUT(9) | \
                                 PIN_MODE_INPUT(10) | \
                                 PIN_MODE_INPUT(11) | \
                                 PIN_MODE_INPUT(12) | \
                                 PIN_MODE_INPUT(13) | \
                                 PIN_MODE_INPUT(14) | \
                                 PIN_MODE_INPUT(15))
#define VAL_GPIOH_OTYPER        0x00000000
#define VAL_GPIOH_OSPEEDR       0xFFFFFFFF
#define VAL_GPIOH_PUPDR         (PIN_PUDR_FLOATING(GPIOH_OSC_IN) | \
                                 PIN_PUDR_FLOATING(GPIOH_OSC_OUT) | \
                                 PIN_PUDR_PULLUP(2) | \
                                 PIN_PUDR_PULLUP(3) | \
                                 PIN_PUDR_PULLUP(4) | \
                                 PIN_PUDR_PULLUP(5) | \
                                 PIN_PUDR_PULLUP(6) | \
                                 PIN_PUDR_PULLUP(7) | \
                                 PIN_PUDR_PULLUP(8) | \
                                 PIN_PUDR_PULLUP(9) | \
                                 PIN_PUDR_PULLUP(10) | \
                                 PIN_PUDR_PULLUP(11) | \
                                 PIN_PUDR_PULLUP(12) | \
                                 PIN_PUDR_PULLUP(13) | \
                                 PIN_PUDR_PULLUP(14) | \
                                 PIN_PUDR_PULLUP(15))
#define VAL_GPIOH_ODR           0xFFFF
#define VAL_GPIOH_AFRL          0x00000000
#define VAL_GPIOH_AFRH          0x00000000

/*
 * Port I setup.
 * All input with pull-up.
 */
#define VAL_GPIOI_MODER         0x00000000
#define VAL_GPIOI_OTYPER        0x00000000
#define VAL_GPIOI_OSPEEDR       0xFFFFFFFF
#define VAL_GPIOI_PUPDR         0xFFFFFFFF
#define VAL_GPIOI_ODR           0xFFFF
#define VAL_GPIOI_AFRL          0x00000000
#define VAL_GPIOI_AFRH          0x00000000

#if !defined(_FROM_ASM_)
#ifdef __cplusplus
extern "C" {
#endif
  void boardInit(void);
  void boardWriteIoPower(int value);
  void boardWriteLed(int value);
  void boardWriteSystemPower(int value);
  void boardWriteWarmRestart(const uint8_t value);
  void boardChargerSetState(uint8_t chrg_mask, uint8_t state);
  void boardBluetoothSetState(uint8_t state);
  void boardRequestShutdown(void);
  void boardStandby(void);
  void boardStop(const uint8_t lpds, const uint8_t fpds);
  void boardWakeup(void);
  void boardClearI2CBus(const uint8_t scl_pad, const uint8_t sda_pad);
  void boardResetBQ27500I2C(const uint8_t scl_pad, const uint8_t sda_pad);
#ifdef __cplusplus
}
#endif
#endif /* _FROM_ASM_ */

#endif /* _BOARD_H_ */
