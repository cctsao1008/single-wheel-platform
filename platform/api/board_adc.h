#ifndef SWP_BOARD_ADC_H
#define SWP_BOARD_ADC_H

#include <stdbool.h>

bool board_adc_init(void);
bool board_adc_read_battery_v(float *battery_v);

#endif
