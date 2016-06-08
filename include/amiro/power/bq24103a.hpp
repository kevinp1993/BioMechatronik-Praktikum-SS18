/*
 * BQ24103A - Battery Charger
 */

#ifndef AMIRO_BQ24103A_H_
#define AMIRO_BQ24103A_H_

namespace amiro {

class BaseBQ24103A
{
public:
  BaseBQ24103A()
  {}

  virtual ~BaseBQ24103A()
  {}

  virtual void enable(const bool en = true) = 0;

  virtual bool isEnabled() const = 0;

  virtual bool isCharging() const = 0;
};

template <uintptr_t EN_PadGrp, uint32_t EN_Pad, uintptr_t STAT_PadGrp, uint32_t STAT_Pad>
class BQ24103A : public BaseBQ24103A
{
private:
  bool is_enabled;

public:
  BQ24103A(const bool enable = false) :
    is_enabled(enable)
  {
    this->enable(enable);
  }

  virtual ~BQ24103A()
  {}

  void enable(const bool enable = true)
  {
    // writing to a port is not atomic, thus the system must be locked
    chSysLock();
    palWritePad((GPIO_TypeDef*)EN_PadGrp, EN_Pad, (enable? PAL_LOW : PAL_HIGH));
    this->is_enabled = enable;
    chSysUnlock();
    return;
  }

  bool isEnabled() const
  {
    return this->is_enabled;
  }

  bool isCharging() const
  {
    return (palReadPad((GPIO_TypeDef*)STAT_PadGrp, STAT_Pad) == PAL_LOW);
  }
};

}

#endif /* AMIRO_BQ24103A_H_ */
