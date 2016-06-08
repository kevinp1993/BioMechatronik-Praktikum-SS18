/*
 * LTC4412 - PowerPath Controller
 */

#ifndef AMIRO_LTC4412_H_
#define AMIRO_LTC4412_H_

#include <hal.h>

namespace amiro {

class BaseLTC4412
{
public:
  BaseLTC4412()
  {}

  virtual ~BaseLTC4412()
  {}

  virtual bool isPluggedIn() const = 0;
};

class BaseLTC4412wEN : public BaseLTC4412
{
public:
  BaseLTC4412wEN()
  {}

  virtual ~BaseLTC4412wEN()
  {}

  virtual void enable(const bool enable = true) = 0;

  virtual bool isEnabled() const = 0;

  virtual bool isPluggedIn() const = 0;
};



template <uintptr_t STAT_PadGrp, uint8_t STAT_Pad>
class LTC4412 : public BaseLTC4412
{
public:
  LTC4412()
  {}

  virtual ~LTC4412()
  {}

  bool isPluggedIn() const
  {
    return (palReadPad((GPIO_TypeDef*)STAT_PadGrp, STAT_Pad) == PAL_HIGH);
  }

};

template <uintptr_t STAT_PadGrp, uint8_t STAT_Pad, uintptr_t EN_PadGrp, uint8_t EN_Pad>
class LTC4412wEN : public BaseLTC4412wEN
{
private:
  bool is_enabled;

public:
  LTC4412wEN(const bool enable = false) :
    is_enabled(enable)
  {
    this->enable(enable);
  }

  virtual ~LTC4412wEN()
  {}

  void enable(const bool enable = true)
  {
    // writing to a port is not atomic, thus the system must be locked
    chSysLock();
    palWritePad((GPIO_TypeDef*)EN_PadGrp, EN_Pad, enable? PAL_LOW : PAL_HIGH);
    this->is_enabled = enable;
    chSysUnlock();
  }

  bool isEnabled() const
  {
    return this->is_enabled;
  }

  bool isPluggedIn() const
  {
    return (palReadPad((GPIO_TypeDef*)STAT_PadGrp, STAT_Pad) == PAL_HIGH);
  }

};

}

#endif /* AMIRO_LTC4412_H_ */
