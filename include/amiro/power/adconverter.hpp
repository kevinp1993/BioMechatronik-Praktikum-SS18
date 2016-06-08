/*
 * ADC for monitoring the system voltage and handling the charger logic
 */

#ifndef AMIRO_ADCONVERTER_HPP
#define AMIRO_ADCONVERTER_HPP

#include <ch.hpp>
#include <hal.h>
#include <evtimer.h>

namespace amiro {

class ADConverter : public chibios_rt::BaseStaticThread<256>
{
public:
  enum Events {
    EVT_WATCHDOG = 0x00,
    EVT_TIMER = 0x01
  };

  static uint16_t uV2Adc(const uint32_t uV);
  static uint32_t adc2uV(const uint16_t adc);

private:
  ADCDriver& driver;
  ADCConversionGroup& conversion_group;
  uint32_t watchdog_threshold_uV;
  chibios_rt::EvtListener evt_listener;
  chibios_rt::EvtSource* evt_source;
  EvTimer evt_timer;

public:
  ADConverter(ADCDriver &adcdrv, ADCConversionGroup& adccgrp, const uint32_t threshold_uV);
  virtual ~ADConverter();

  ADCConversionGroup& getConversionGroup() {return this->conversion_group;}
  const ADCConversionGroup& getConversionGroup() const {return this->conversion_group;}

protected:
  virtual msg_t main(void);

private:
  void enableWatchdog(const bool enable = true);

};

}

#endif // AMIRO_ADCONVERTER_HPP

