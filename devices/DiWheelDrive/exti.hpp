#ifndef EXT_HPP_
#define EXT_HPP_

extern volatile uint32_t shutdown_now;

extern EXTConfig extcfg;

void power_down_cb(EXTDriver *extp, expchannel_t channel);

#endif /* EXT_HPP_ */
