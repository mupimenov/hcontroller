#include "program.h"

#include <stdbool.h>
#include <string.h>
#include <math.h>

#include "Arduino.h"

#include "config.h"
#include "datetime.h"
#include "cyclogram.h"
#include "softpwm.h"

#define OFF (0)
#define ON (1)

/* TIMER CONTROL */

struct timer_control_state
{
	uint8_t phase;
	struct cyclogram_state cyclogram;
};

enum timer_control_phase
{
	TIMER_COMPUTE,
	TIMER_OUTPUT
};

/* RELAY CONTROL */

struct relay_control_state
{
	uint8_t phase;
	uint8_t value;
	struct cyclogram_state cyclogram;
};

enum relay_control_phase
{
	RELAY_COMPUTE,
	RELAY_OUTPUT
};

/* PID CONTROL */

struct pid_control_state
{
	float integral_sum;	
	uint32_t last_millis;
	struct softpwm_params softpwm_p;
	struct softpwm_state softpwm_s;
	uint8_t value;
};

/* BUTTON CONTROL */

struct button_control_state
{
	uint8_t value;
};

struct abstract_program_state
{
	union {
		struct timer_control_state timer;
		struct relay_control_state relay;
		struct pid_control_state pid;
		struct button_control_state button;
	} data;
};

static struct abstract_program_state 	program_state[PROGRAMS_COUNT];
static bool 							need_prepare = false;

void program_init(void)
{
	memset(program_state, 0, sizeof(program_state));
	need_prepare = true;
}

void program_reset(void)
{
	need_prepare = true;
}

static bool prepare_timer_control(struct abstract_program_state *state, struct abstract_program *program)
{
	if (program->data.common.type != TIMER_CONTROL_PROGRAM)
		return false;
	
	state->data.timer.phase = TIMER_COMPUTE;
	cyclogram_reset(&state->data.timer.cyclogram);
	
	return true;
}

static bool prepare_relay_control(struct abstract_program_state *state, struct abstract_program *program)
{
	if (program->data.common.type != RELAY_CONTROL_PROGRAM)
		return false;
	
	state->data.relay.phase = RELAY_COMPUTE;
	state->data.relay.value = OFF;
	cyclogram_reset(&state->data.relay.cyclogram);
	
	return true;
}

static bool prepare_pid_control(struct abstract_program_state *state, struct abstract_program *program)
{
	if (program->data.common.type != PID_CONTROL_PROGRAM)
		return false;
	
	state->data.pid.integral_sum = 0.0f;
	state->data.pid.last_millis = millis();
	softpwm_reset(&state->data.pid.softpwm_s);
	state->data.pid.value = OFF;
	
	return true;
}

static bool prepare_button_control(struct abstract_program_state *state, struct abstract_program *program)
{
	if (program->data.common.type != BUTTON_CONTROL_PROGRAM)
		return false;

	state->data.button.value = OFF;

	return true;
}

static bool prepare_empty_program(struct abstract_program_state *state, struct abstract_program *program)
{	
	return true;
}

typedef bool (*prep_program_fn)(struct abstract_program_state *, struct abstract_program *);

static const prep_program_fn  prepare_program[] = {
	prepare_timer_control,
	prepare_relay_control,
	prepare_pid_control,
	prepare_button_control,
	
	prepare_empty_program
};

static const uint8_t prepare_program_count = sizeof(prepare_program) / sizeof(prepare_program[0]);

static void program_prepare(void)
{
	if (need_prepare)
	{
		uint8_t i;	
		
		for (i = 0; i < PROGRAMS_COUNT; ++i)
		{
			struct abstract_program program;
			uint8_t j;
			
			read_program(i, &program);
			
			for (j = 0; j < prepare_program_count; ++j)
			{
				if (prepare_program[j](&program_state[i], &program))
					break;
			}
		}
		
		need_prepare = false;
	}
}

static void timer_control_execute(struct timer_control_state *state, struct timer_control_program *program);
static void relay_control_execute(struct relay_control_state *state, struct relay_control_program *program);
static void pid_control_execute(struct pid_control_state *state, struct pid_control_program *program);
static void button_control_execute(struct button_control_state *state, struct button_control_program *program);

void program_execute(void)
{
	program_prepare();
	{
		uint8_t num;

		for (num = 0; num < PROGRAMS_COUNT; ++num)
		{
			struct abstract_program program;
			read_program(num, &program);
			
			switch (program.data.common.type)
			{
			case TIMER_CONTROL_PROGRAM:
				timer_control_execute(&program_state[num].data.timer, &program.data.timer);
				break;
			case RELAY_CONTROL_PROGRAM:
				relay_control_execute(&program_state[num].data.relay, &program.data.relay);
				break;
			case PID_CONTROL_PROGRAM:
				pid_control_execute(&program_state[num].data.pid, &program.data.pid);
				break;
			case BUTTON_CONTROL_PROGRAM:
				button_control_execute(&program_state[num].data.button, &program.data.button);
				break;
			default:
				break;
			}
		}
	}
}

/* TIMER CONTROL PROGRAM */

static void timer_control_execute(struct timer_control_state *state, struct timer_control_program *program)
{
	uint8_t value = OFF;
	int err;
	
	if (state->phase != TIMER_COMPUTE
		&& state->phase != TIMER_OUTPUT)
	{
		state->phase = TIMER_COMPUTE;
	}

	if (state->phase == TIMER_COMPUTE)
	{
		struct datetime now = datetime_now();
		if (datetime_in(&now, &program->from, &program->to, (enum time_constrains)program->constrains))
		{
			state->phase = TIMER_OUTPUT;
		}
	}
	
	if (state->phase == TIMER_OUTPUT)
	{
		if (cyclogram_is_stopped(&state->cyclogram))
			cyclogram_start(&state->cyclogram, &program->cyclogram);
		
		value = cyclogram_step(&state->cyclogram, &program->cyclogram);
		
		if (cyclogram_is_stopped(&state->cyclogram))
			state->phase = TIMER_COMPUTE;
	}
	
	output_discrete(program->output, value, &err);
}

/* RELAY CONTROL PROGRAM */

static void relay_control_execute(struct relay_control_state *state, struct relay_control_program *program)
{
	uint8_t value = OFF;
	int err;
	
	if (state->phase != RELAY_COMPUTE
		&& state->phase != RELAY_OUTPUT)
	{
		state->phase = RELAY_COMPUTE;
	}

	if (state->phase == RELAY_COMPUTE)
	{
		do
		{
			float input;
			int err;
			struct datetime now = datetime_now();
			
			if (!datetime_in(&now, &program->from, &program->to, (enum time_constrains)program->constrains))
				break;
			
			input = input_analog(program->input, &err);
			if (err)
				break;
			
			if (state->value == OFF && input < program->low_bound)
			{
				state->value = ON;
			}
			else if (state->value == ON && input < program->high_bound)
			{
				state->value = ON;
			}
			else if (state->value == ON && input >= program->high_bound)
			{
				state->value = OFF;
			}
			else if (state->value == OFF && input >= program->low_bound)
			{
				state->value = OFF;
			}
			
			if (!program->inverse)
			{
				if (state->value == ON)
					state->phase = RELAY_OUTPUT;
			}
			else
			{
				if (state->value == OFF)
					state->phase = RELAY_OUTPUT;
			}
			
		} while (0);
	}
	
	if (state->phase == RELAY_OUTPUT)
	{
		if (cyclogram_is_stopped(&state->cyclogram))
			cyclogram_start(&state->cyclogram, &program->cyclogram);
		
		value = cyclogram_step(&state->cyclogram, &program->cyclogram);
		
		if (cyclogram_is_stopped(&state->cyclogram))
			state->phase = RELAY_COMPUTE;
	}
	
	output_discrete(program->output, value, &err);
}

/* PID CONTROL PROGRAM */

static void pid_control_execute(struct pid_control_state *state, struct pid_control_program *program)
{
	const uint32_t step = 10;
	const float period_min = 5.0f;
	const float period_max = 60.0f;
	const float period_gain = 30.0f;
	const uint8_t duration_min = 1; /* seconds */
	const float dx_min = 0.1f;
	
	int err;
	uint32_t delta_millis;
	
	delta_millis = millis() - state->last_millis;
	if (delta_millis > step)
	{
		do
		{
			float dt = (float)delta_millis / 1000.0f;
			
			float proportional;
			float differential;			
			
			float dx;
			float period;
			float y;
			
			float x = input_analog(program->input, &err);
			if (err)
				break;
			
			dx = program->desired - x;
			
			proportional = program->proportional_gain * dx;
			if (proportional > 1.0f)
				proportional = 1.0f;
			else if (proportional < -1.0f)
				proportional = -1.0f;
			
			if (isnan(state->integral_sum))
				state->integral_sum = 0.0f;
			state->integral_sum += program->integral_gain * dx * dt;
			
			if (state->integral_sum > 1.0f)
				state->integral_sum = 1.0f;
			else if (state->integral_sum < -1.0f)
				state->integral_sum = -1.0f;
			
			differential = dx / dt;
			if (differential > 1.0f)
				differential = 1.0f;
			else if (differential < -1.0f)
				differential = -1.0f;
			
			y = proportional + state->integral_sum + differential;
			if (program->inverse)
			{
				y *= -1.0f;
			}
			
			if (fabs(dx) < dx_min)
			{
				period = period_max;
			}
			else
			{
				period = period_gain / fabs(dx);
				
				if (period < period_min)
					period = period_min;
				else if (period > period_max)
					period = period_max;
			}
			
			state->softpwm_p.period = period;
			state->softpwm_p.duration_min = duration_min;
			
			state->value = softpwm_step(&state->softpwm_s, &state->softpwm_p, y);
			
		} while (0);
	}
	
	output_discrete(program->output, state->value, &err);
}

/* BUTTON CONTROL PROGRAM */

static void button_control_execute(struct button_control_state *state, struct button_control_program *program)
{
	int err;
	uint8_t value;

	value = input_discrete(program->input, &err);
	if (err)
		return;

	state->value = value;

	output_discrete(program->output, state->value, &err);
}
