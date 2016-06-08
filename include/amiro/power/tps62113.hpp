/*
 * TPS62113 - StepDown Converter
 */

#ifndef AMIRO_TPS62113_H_
#define AMIRO_TPS62113_H_

namespace amiro {

  template <uintptr_t PadGrp, uint32_t Pad>
  class TPS62113
  {
    public:
    TPS62113()
    {}

    ~TPS62113()
    {}

    static void enable(const bool en = true)
    {
      palWritePad((GPIO_TypeDef*)PadGrp, Pad, (en? PAL_HIGH : PAL_LOW));
      return;
    }
  };

}

#endif /* AMIRO_TPS62113_H_ */
