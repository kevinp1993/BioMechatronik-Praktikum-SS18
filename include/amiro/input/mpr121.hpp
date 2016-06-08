#ifndef MPR121_HPP_
#define MPR121_HPP_

#include <ch.hpp>
#include <amiro/bus/i2c/I2CParams.hpp>

namespace amiro {

  class I2CDriver;

  class MPR121 : public chibios_rt::BaseStaticThread<256>{

  enum { SLA = 0x5Au };

  struct thresholds {

    uint8_t touch_threshold;
    uint8_t release_threshold;

  } __attribute__((packed));

  struct charge_time {

    uint8_t cdt0 :3;
    uint8_t res0 :1;
    uint8_t cdt1 :3;
    uint8_t res1 :1;

  }__attribute__((packed));

  struct registers {

    uint16_t           touch_status;
    uint16_t           oor_status;
    uint16_t           ele_filt_data[13]; /* LE */
    uint8_t            ele_baseline[13];
    uint8_t            mhd_rising;
    uint8_t            nhd_amt_rising;
    uint8_t            ncl_rising;
    uint8_t            fdl_rising;
    uint8_t            mhd_falling;
    uint8_t            nhd_amt_falling;
    uint8_t            ncl_falling;
    uint8_t            fdl_falling;
    uint8_t            ndh_amt_touched;
    uint8_t            ncl_touched;
    uint8_t            fdl_touched;
    uint8_t            eleprox_mhd_rising;
    uint8_t            eleprox_nhd_amt_rising;
    uint8_t            eleprox_ncl_rising;
    uint8_t            eleprox_fdl_rising;
    uint8_t            eleprox_mhd_falling;
    uint8_t            eleprox_nhd_amt_falling;
    uint8_t            eleprox_ncl_falling;
    uint8_t            eleprox_fdl_falling;
    uint8_t            eleprox_nhd_amt_touched;
    uint8_t            eleprox_ncl_touched;
    uint8_t            eleprox_fdl_touched;
    struct thresholds  ele_thresholds[13];
    uint8_t            debounce;
    uint8_t            cdc_config;
    uint8_t            cdt_config;
    uint8_t            ele_config;
    uint8_t            ele_current[13];
    struct charge_time ele_charge_time[(13 + 1)/2];
    uint16_t           gpio_ctrl;
    uint8_t            gpio_data;
    uint8_t            gpio_ddr;
    uint8_t            gpio_en;
    uint8_t            gpio_set;
    uint8_t            gpio_clear;
    uint8_t            gpio_toggle;
    uint16_t           auto_cfg_ctrl;
    uint8_t            auto_cfg_usl;
    uint8_t            auto_cfg_lsl;
    uint8_t            auto_cfg_target;
    uint8_t            soft_reset;

  } __attribute__((packed));

  public:

  struct MPR121Config {

    uint16_t global_config;
    uint8_t  ele_config;
    uint16_t auto_config;
    uint8_t  up_side_limit;
    uint8_t  low_side_limit;
    uint8_t  target_level;

  };

  enum {

    FFI_6     = 0x0000u,
    FFI_10    = 0x0040u,
    FFI_18    = 0x0080u,
    FFI_34    = 0x00C0u,

    CDT_OFF   = 0x0000u,
    CDT_0_5   = 0x2000u,
    CDT_1     = 0x4000u,
    CDT_2     = 0x6000u,
    CDT_4     = 0x8000u,
    CDT_8     = 0xA000u,
    CDT_16    = 0xC000u,
    CDT_32    = 0xE000u,
    SFI_4     = 0x0000u,
    SFI_6     = 0x0800u,
    SFI_10    = 0x1000u,
    SFI_18    = 0x1800u,
    ESI_1     = 0x0000u,
    ESI_2     = 0x0100u,
    ESI_4     = 0x0200u,
    ESI_8     = 0x0300u,
    ESI_16    = 0x0400u,
    ESI_32    = 0x0500u,
    ESI_64    = 0x0600u,
    ESI_128   = 0x0700u,
  };

  enum {

    CL_ON_CURRENT  = 0x00u,
    CL_OFF         = 0x40u,
    CL_ON_HIGH5    = 0x80u,
    CL_ON_ALL      = 0xC0u,
    ELEPROX_0      = 0x00u,
    ELEPROX_2      = 0x10u,
    ELEPROX_4      = 0x20u,
    ELEPROX_12     = 0x30u,

  };

  enum {

    AC_ENABLE              = 0x0001u,
    AC_RECONF_EN           = 0x0002u,
    BVA_ON_CURRENT         = 0x0000u,
    BVA_OFF                = 0x0004u,
    BVA_ON_HIGH5           = 0x0008u,
    BVA_ON_ALL             = 0x000Cu,
    RETRY_OFF              = 0x0000u,
    RETRY_2                = 0x0010u,
    RETRY_4                = 0x0020u,
    RETRY_8                = 0x0030u,
    /* skip FFI */
    AC_FAIL_INT_EN         = 0x0100u,
    AC_RECONF_FAIL_INT_EN  = 0x0200u,
    AC_OUT_OF_RANGE_INT_EN = 0x0400u,
    AC_SKIP_CHRG_TIME_SRCH = 0x8000u,

  };

  enum {

    SOFT_RST_MAGIC = 0x63u,

  };

  public:

    MPR121(I2CDriver *driver, const uint8_t master_id);
    virtual ~MPR121();

    chibios_rt::EvtSource* getEventSource();
    uint16_t getButtonStatus();
    uint8_t getButtonStatus(uint8_t ix);
    uint16_t getElectrodeData(uint8_t ix);
    uint8_t getBaselineData(uint8_t ix);
    msg_t configure(const MPR121Config *cfg);

  protected:
    virtual msg_t main(void);

  private:

    inline msg_t softReset();
    inline msg_t writeConfig(const MPR121Config *cfg);
    inline msg_t updateButtonData();

  private:

    I2CDriver *driver;
    uint8_t master_id;
    chibios_rt::EvtSource eventSource;

    uint16_t button_state;
    uint16_t electrode_data[13];
    uint8_t baseline_data[13];
    I2CTxParams tx_params;

};


} /* amiro */

#endif /* MPR121_HPP_ */
