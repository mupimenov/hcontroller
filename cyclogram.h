#ifndef CYCLOGRAM_H_
#define CYCLOGRAM_H_

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

struct cyclogram_params
{
	uint8_t type;
	
	uint16_t count;
	uint16_t impulse_duration;
	uint16_t pause_duration;
};

enum cyclogram_type
{
	CYCLOGRAM_SIMPLE,
	CYCLOGRAM_IMPULSE
};

struct cyclogram_state
{
	uint8_t phase;
	uint16_t remains;
	uint32_t start;
};

enum cyclogram_phase
{
	CYCLOGRAM_IDLE,
	
	CYCLOGRAM_HIGH,
	CYCLOGRAM_LOW
};

void cyclogram_reset(struct cyclogram_state *state);

void cyclogram_start(	struct cyclogram_state *state,
						struct cyclogram_params *params);
						
uint8_t cyclogram_step(		struct cyclogram_state *state,
							struct cyclogram_params *params);
							
bool cyclogram_is_stopped(struct cyclogram_state *state);

#ifdef __cplusplus
}
#endif

#endif /* CYCLOGRAM_H_ */
