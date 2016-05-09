#ifndef IO_H_
#define IO_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void io_lock(void);
void io_unlock(void);

void io_reset(void);

void io_init(void);
void io_execute_in(void);
void io_execute_out(void);

uint8_t input_discrete(uint8_t id, int *err);
float input_analog(uint8_t id, int *err);

void output_discrete(uint8_t id, uint8_t value, int *err);

#define ADC_CHANNELS_COUNT 16

uint16_t get_adc_value(uint8_t channel);
float get_ioslot_value(uint8_t num);

#ifdef __cplusplus
}
#endif

#endif /* IO_H_ */
