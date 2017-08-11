#include <amiro/power/adconverter.hpp>

#include <global.hpp>
#include <amiro/Constants.h>
#include <amiro/power/ina219.hpp>
#include <amiro/util/util.h>

#include <chprintf.h>

using namespace amiro;

extern Global global;

void adcVsysErrorCallback(ADCDriver *adcp, adcerror_t err)
{
  if (err == ADC_ERR_WATCHDOG) {
    chSysLockFromIsr();
    if (adcp->thread != NULL) {
      chEvtSignalI(adcp->thread, EVENT_MASK(ADConverter::EVT_WATCHDOG));
      adcp->thread = NULL;
    }
    chSysUnlockFromIsr();
  }

  return;
}

void adcTimerCallback(Thread *tp)
{
  if (tp != NULL) {
    chEvtSignal(tp, EVENT_MASK(ADConverter::EVT_TIMER));
  }
  return;
}

uint16_t
ADConverter::uV2Adc(const uint32_t uV)
{
  // NOTE: The factor 5.33 is an approximation of the voltage divider.
  //       Its value should be 5.0, but due to too large resistors the factor is shifted.
  // TODO: 5.33 should be replaced by a non linear function, which needs to be optained from the system.

  // get the current VDD voltage
  global.ina219[constants::PowerManagement::INA_VDD].update();
  INA219::BusVoltage vdd = global.ina219[constants::PowerManagement::INA_VDD].getVoltage();
  return float(uV)/1000000 / 5.33f / (float(vdd.voltage_uV)/1000000) * 4096;
}

uint32_t ADConverter::adc2uV(const uint16_t adc)
{
  // NOTE: The factor 5.33 is an approximation of the voltage divider.
  //       Its value should be 5.0, but due to too large resistors the factor is shifted.
  // TODO: 5.33 should be replaced by a non linear function, which needs to be optained from the system.

  // get the current VDD voltage
  global.ina219[constants::PowerManagement::INA_VDD].update();
  INA219::BusVoltage vdd = global.ina219[constants::PowerManagement::INA_VDD].getVoltage();
  return float(adc)/4096 * (float(vdd.voltage_uV)/1000000) * 5.33f * 1000000;
}

ADConverter::ADConverter(ADCDriver& adcdrv, ADCConversionGroup& adccgrp, const uint32_t threshold_uV) :
  driver(adcdrv), conversion_group(adccgrp), watchdog_threshold_uV(threshold_uV)
{
}


ADConverter::~ADConverter()
{
}

msg_t
ADConverter::main()
{
  // initialize a timer event for later use
  evtInit(&this->evt_timer, MS2ST(1));
  this->evt_source = reinterpret_cast<EvtSource*>(&this->evt_timer.et_es);
  this->evt_source->registerOne(&this->evt_listener, EVT_TIMER);

  // initialize the watchdog
  this->enableWatchdog(true);
  this->conversion_group.htr = ADC_HTR(uV2Adc(this->watchdog_threshold_uV));
  this->conversion_group.ltr = 0;
  this->driver.thread = this->thread_ref;

  adcStartConversion(&this->driver, &this->conversion_group, global.adc1_buffer, ARRAY_SIZE(global.adc1_buffer));

  while (!this->shouldTerminate())
  {
    eventmask_t event = this->waitOneEventTimeout(ALL_EVENTS, MS2ST(1000));

    switch (event)
    {
      // watchdog event (fired by ADC interrupt)
      case EVENT_MASK(EVT_WATCHDOG):
      {
        // disable the chargers
        global.bq24103a[constants::PowerManagement::BAT_P7]->enable(false);
        global.bq24103a[constants::PowerManagement::BAT_P8]->enable(false);

        // to avoid multiple interrupts because of noise, wait 1ms before setting the chargers and ADC
        this->enableWatchdog(false);
        evtStart(&this->evt_timer);
        break;
      }

      // timer event
      case EVENT_MASK(EVT_TIMER):
      {
        evtStop(&this->evt_timer);

        // read the current voltage
        this->conversion_group.circular = FALSE;
        adcConvert(&this->driver, &this->conversion_group, global.adc1_buffer, ARRAY_SIZE(global.adc1_buffer));
        this->conversion_group.circular = TRUE;
        this->enableWatchdog(true); // Watchdog was disabled in ISR

        if (global.adc1_buffer[0] > uV2Adc(this->watchdog_threshold_uV))
        {
          chprintf((BaseSequentialStream*)&global.sercanmux1, "%f > 9V detected: now charging\n", ((float)adc2uV(global.adc1_buffer[0]))/1000000.0f);
          global.robot.getPowerStatus().charging_flags.content.vsys_higher_than_9V = true;
          global.bq24103a[constants::PowerManagement::BAT_P7]->enable(true);
          global.bq24103a[constants::PowerManagement::BAT_P8]->enable(true);
          this->conversion_group.htr = ADC_HTR_HT;
          this->conversion_group.ltr = ADC_LTR(uV2Adc(this->watchdog_threshold_uV));
        } else {
          chprintf((BaseSequentialStream*)&global.sercanmux1, "%f < 9V detected: not charging\n", ((float)adc2uV(global.adc1_buffer[0]))/1000000.0f);
          global.robot.getPowerStatus().charging_flags.content.vsys_higher_than_9V = false;
          this->conversion_group.htr = ADC_HTR(uV2Adc(this->watchdog_threshold_uV));
          this->conversion_group.ltr = 0;
        }
        this->driver.thread = this->thread_ref;
        adcStartConversion(&this->driver, &this->conversion_group, global.adc1_buffer, ARRAY_SIZE(global.adc1_buffer));
        break;
      }

      // timeout
      case 0:

      default:
        break;
    }
  }

  return RDY_RESET;
}

void
ADConverter::enableWatchdog(const bool enable)
{
  if (enable) {
    this->conversion_group.cr1 |= ADC_CR1_AWDEN | ADC_CR1_AWDIE;
    this->conversion_group.error_cb = &adcVsysErrorCallback;
  } else {
    this->conversion_group.cr1 &= ~(ADC_CR1_AWDEN | ADC_CR1_AWDIE);
    this->conversion_group.error_cb = NULL;
  }
}


