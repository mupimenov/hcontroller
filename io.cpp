#include "io.h"

#include <stdbool.h>
#include <math.h>
#include <string.h>

#include "Arduino.h"
#include "scmRTOS.h"
#include "DHT.h"
#include "OneWire.h"

#include "config.h"

#define DISCRETE_INPUT_FILTER_BITS 0x7

#define OFF (0)
#define ON (1)

#define UNKNOWN_VALUE 0xFF

static uint16_t adc_value[ADC_CHANNELS_COUNT];

struct common_state
{
	uint8_t driver;
	uint8_t id;
};

struct analog_input_state
{
	uint8_t driver;
	uint8_t id;
	
	float value;
};

struct discrete_input_state
{
	uint8_t driver;
	uint8_t id;
	
	uint8_t sequence;
	uint8_t value;
};

struct discrete_output_state
{
	uint8_t driver;
	uint8_t id;	
	
	uint8_t count;
	uint8_t value;
};

struct dhtxx_state
{
	uint8_t driver;
	uint8_t id;
	
	uint8_t modification;
	uint8_t parameter;
	uint8_t pin;	
	uint32_t last_millis;
	float value;
};

struct dallas_temperature_state
{
	uint8_t driver;
	uint8_t id;
	
	uint8_t pin;
	uint32_t last_millis;
	float value;
};

struct abstract_ioslot_state
{
	void (*execute)(struct abstract_ioslot_state *state, struct abstract_ioslot *ioslot, uint8_t mode);
	void (*io_discrete)(struct abstract_ioslot_state *state, uint8_t mode, uint8_t *value);
	void (*io_analog)(struct abstract_ioslot_state *state, uint8_t mode, float *value);
	
	union {
		struct common_state 			common;
		struct analog_input_state 		analog_input;
		struct discrete_input_state 	discrete_input;
		struct discrete_output_state 	discrete_output;
		struct dhtxx_state 				dhtxx;
		struct dallas_temperature_state dallas_temperature;
	} data;
};

enum MODE
{
	IN,
	OUT
};

static struct abstract_ioslot_state 	ioslot_state[IOSLOTS_COUNT];
static OS::TMutex 						mutex;
static bool 							need_prepare = false;

void io_lock(void)
{
	mutex.lock();
}

void io_unlock(void)
{
	mutex.unlock();
}

void io_init(void)
{
	memset(adc_value, 0, sizeof(adc_value));
	need_prepare = true;
}

void io_reset(void)
{
	OS::TMutexLocker locker(mutex);
	need_prepare = true;
}

static void fill_adc_values(void)
{
	uint8_t i;
	uint8_t j;
	
	for (i = 0; i < ADC_CHANNELS_COUNT; ++i)
	{
		adc_value[i] = 0;
		for (j = 0; j < 4; ++j)
			adc_value[i] += analogRead(i);
		adc_value[i] /= 4;
	}
}

/* ANALOG INPUT */

static void analog_input_execute(struct abstract_ioslot_state *state, struct abstract_ioslot *ioslot, uint8_t mode)
{
	if (mode == IN)
	{
		int16_t dx = ioslot->data.analog_input.x2 - ioslot->data.analog_input.x1;
		if (dx != 0)
		{
			float k = (ioslot->data.analog_input.y2 - ioslot->data.analog_input.y1) / (float(dx));
			float b = ioslot->data.analog_input.y2 - k * ioslot->data.analog_input.x2;

			state->data.analog_input.value = k * adc_value[ioslot->data.analog_input.num] + b;
		}
	}
}

static void analog_input_io_discrete(struct abstract_ioslot_state *state, uint8_t mode, uint8_t *value)
{
	if (mode == IN)
	{
		*value = 0;
	}
}

static void analog_input_io_analog(struct abstract_ioslot_state *state, uint8_t mode, float *value)
{
	if (mode == IN)
	{
		*value = state->data.analog_input.value;
	}
}

static bool prepare_analog_input(struct abstract_ioslot_state *state, struct abstract_ioslot *ioslot)
{
	if (ioslot->data.common.driver != ANALOG_INPUT_DRIVER)
		return false;
	
	state->execute = analog_input_execute;
	state->io_discrete = analog_input_io_discrete;
	state->io_analog = analog_input_io_analog;
	
	state->data.analog_input.driver = ioslot->data.analog_input.driver;
	state->data.analog_input.id = ioslot->data.analog_input.id;
	state->data.analog_input.value = NAN;
	
	return true;
}

/* DISCRETE INPUT */

static void discrete_input_execute(struct abstract_ioslot_state *state, struct abstract_ioslot *ioslot, uint8_t mode)
{
	if (mode == IN)
	{
		uint8_t in = (digitalRead(ioslot->data.discrete_input.pin) & 1);
		if (ioslot->data.discrete_input.inverse)
			in = in == ON? OFF: ON;
		
		state->data.discrete_input.sequence = (state->data.discrete_input.sequence << 1)
				| in;

		if (state->data.discrete_input.value == UNKNOWN_VALUE)
		{
			state->data.discrete_input.value = OFF;
		}

		if ((state->data.discrete_input.value == OFF) 
			&& ((state->data.discrete_input.sequence & DISCRETE_INPUT_FILTER_BITS) == DISCRETE_INPUT_FILTER_BITS))
		{
			state->data.discrete_input.value = ON;
		}
		else if ((state->data.discrete_input.value == ON) 
			&& ((state->data.discrete_input.sequence & DISCRETE_INPUT_FILTER_BITS) == 0x0))
		{
			state->data.discrete_input.value = OFF;
		}
	}
}

static void discrete_input_io_discrete(struct abstract_ioslot_state *state, uint8_t mode, uint8_t *value)
{
	if (mode == IN)
	{
		*value = state->data.discrete_input.value;
	}
}

static void discrete_input_io_analog(struct abstract_ioslot_state *state, uint8_t mode, float *value)
{
	if (mode == IN)
	{
		*value = 1.0f * state->data.discrete_input.value;
	}
}

static bool prepare_discrete_input(struct abstract_ioslot_state *state, struct abstract_ioslot *ioslot)
{
	if (ioslot->data.common.driver != DISCRETE_INPUT_DRIVER)
		return false;
	
	state->execute = discrete_input_execute;
	state->io_discrete = discrete_input_io_discrete;
	state->io_analog = discrete_input_io_analog;
	
	state->data.discrete_input.driver = ioslot->data.discrete_input.driver;
	state->data.discrete_input.id = ioslot->data.discrete_input.id;
	
	state->data.discrete_input.sequence = 0;
	state->data.discrete_input.value = UNKNOWN_VALUE;
	
	pinMode(ioslot->data.discrete_input.pin, INPUT_PULLUP);

	return true;
}

/* DISCRETE OUTPUT */

static void discrete_output_execute(struct abstract_ioslot_state *state, struct abstract_ioslot *ioslot, uint8_t mode)
{
	if (mode == OUT)
	{
		if (ioslot->data.discrete_output.operation == OPERATION_OR)
		{
			state->data.discrete_output.value = state->data.discrete_output.count? ON: OFF;
		}
		else if (ioslot->data.discrete_output.operation == OPERATION_AND)
		{
			uint8_t max_count = program_count_with_output(state->data.discrete_output.id);
			state->data.discrete_output.value 
				= (state->data.discrete_output.count == max_count) ? ON: OFF;
		}
		
		// INVERSE?
		if (ioslot->data.discrete_output.inverse)
			state->data.discrete_output.value = state->data.discrete_output.value == ON? OFF: ON;
		
		// OUT AND CLEAR
		digitalWrite(ioslot->data.discrete_output.pin, state->data.discrete_output.value);
		state->data.discrete_output.count = 0;
	}
}

static void discrete_output_io_discrete(struct abstract_ioslot_state *state, uint8_t mode, uint8_t *value)
{
	if (mode == IN)
	{
		*value = state->data.discrete_output.value;
	}

	if (mode == OUT)
	{
		if (*value == ON)
			state->data.discrete_output.count++;
	}
}

static void discrete_output_io_analog(struct abstract_ioslot_state *state, uint8_t mode, float *value)
{
	if (mode == IN)
	{
		*value = (float)state->data.discrete_output.value;
	}

	if (mode == OUT)
	{
		if (*value >= 0.5f)
			state->data.discrete_output.count++;
	}
}

static bool prepare_discrete_output(struct abstract_ioslot_state *state, struct abstract_ioslot *ioslot)
{
	if (ioslot->data.common.driver != DISCRETE_OUTPUT_DRIVER)
		return false;
	
	state->execute = discrete_output_execute;
	state->io_discrete = discrete_output_io_discrete;
	state->io_analog = discrete_output_io_analog;
	
	state->data.discrete_output.driver = ioslot->data.discrete_output.driver;
	state->data.discrete_output.id = ioslot->data.discrete_output.id;
	
	state->data.discrete_output.count = 0;
	state->data.discrete_output.value = 0;
	
	pinMode(ioslot->data.discrete_output.pin, OUTPUT);

	return true;
}

/* DHTxx */

static void dhtxx_execute(struct abstract_ioslot_state *state, struct abstract_ioslot *ioslot, uint8_t mode)
{
	if (mode == IN)
	{

	}
}

static void dhtxx_io_discrete(struct abstract_ioslot_state *state, uint8_t mode, uint8_t *value)
{
	if (mode == IN)
	{
		*value = 0;
	}
}

static void dhtxx_io_analog(struct abstract_ioslot_state *state, uint8_t mode, float *value)
{
	if (mode == IN)
	{
		*value = state->data.dhtxx.value;
	}
}

static bool prepare_dhtxx(struct abstract_ioslot_state *state, struct abstract_ioslot *ioslot)
{
	if (ioslot->data.common.driver != DHTxx_DRIVER)
		return false;
	
	state->execute = dhtxx_execute;
	state->io_discrete = dhtxx_io_discrete;
	state->io_analog = dhtxx_io_analog;
	
	state->data.dhtxx.driver = ioslot->data.dhtxx.driver;
	state->data.dhtxx.id = ioslot->data.dhtxx.id;

	state->data.dhtxx.modification = ioslot->data.dhtxx.modification;
	state->data.dhtxx.parameter = ioslot->data.dhtxx.parameter;
	state->data.dhtxx.pin = ioslot->data.dhtxx.pin;
	
	state->data.dhtxx.value = NAN;
	
	return true;
}

static void update_dhtxx_state(uint8_t pin, uint32_t now, float temperature, float humidity)
{
	uint8_t i;

	for (i = 0; i < IOSLOTS_COUNT; ++i)
	{
		if (ioslot_state[i].data.common.driver == DHTxx_DRIVER)
		{
			if (ioslot_state[i].data.dhtxx.pin == pin)
			{
				ioslot_state[i].data.dhtxx.last_millis = now;

				if (ioslot_state[i].data.dhtxx.parameter == DHTxx_TEMPERATURE)
					ioslot_state[i].data.dhtxx.value = temperature;
				else
					ioslot_state[i].data.dhtxx.value = humidity;
			}
		}
	}
}

static void get_dhtxx_values(void)
{
	uint8_t i;
	bool dhtxx_requested = false;

	static uint8_t dhtxx_next_num = 0;
	static const uint32_t dhtxx_period = 2000UL;

	for (i = 0; i < IOSLOTS_COUNT; ++i)
	{
		if (ioslot_state[i].data.common.driver == DHTxx_DRIVER)
		{
			if (i >= dhtxx_next_num)
			{
				uint32_t now = millis();
				uint8_t pin = ioslot_state[i].data.dhtxx.pin;
				uint32_t last_millis = ioslot_state[i].data.dhtxx.last_millis;

				if ((now > last_millis + dhtxx_period)
						|| now < last_millis)
				{
					uint8_t type = ioslot_state[i].data.dhtxx.modification == DHTxx_DHT11? DHT11: DHT22;
					DHT dht(pin, type);
					dht.begin();
					float temperature = dht.readTemperature();
					float humidity = dht.readHumidity();

					update_dhtxx_state(pin, now, temperature, humidity);

					dhtxx_requested = true;
					dhtxx_next_num = i + 1;
					break;
				}
			}
		}
	}

	if (!dhtxx_requested)
	{
		dhtxx_next_num = 0;
	}
}

/* DALLAS TEMPERATURE */

static void dallas_temperature_execute(struct abstract_ioslot_state *state, struct abstract_ioslot *ioslot, uint8_t mode)
{
	if (mode == IN)
	{

	}
}

static void dallas_temperature_io_discrete(struct abstract_ioslot_state *state, uint8_t mode, uint8_t *value)
{
	if (mode == IN)
	{
		*value = 0;
	}
}

static void dallas_temperature_io_analog(struct abstract_ioslot_state *state, uint8_t mode, float *value)
{
	if (mode == IN)
	{
		*value = state->data.dallas_temperature.value;
	}
}

static bool prepare_dallas_temperature(struct abstract_ioslot_state *state, struct abstract_ioslot *ioslot)
{
	if (ioslot->data.common.driver != DALLAS_TEMPERATURE_DRIVER)
		return false;
	
	state->execute = dallas_temperature_execute;
	state->io_discrete = dallas_temperature_io_discrete;
	state->io_analog = dallas_temperature_io_analog;
	
	state->data.dallas_temperature.driver = ioslot->data.dallas_temperature.driver;
	state->data.dallas_temperature.id = ioslot->data.dallas_temperature.id;
	state->data.dallas_temperature.pin = ioslot->data.dallas_temperature.pin;
	
	state->data.dallas_temperature.value = NAN;
	
	return true;
}

static void get_dallas_values(void)
{
	uint8_t i;
	bool dallas_requested = false;
	
	static uint8_t dallas_next_num = 0;
	static const uint32_t dallas_period = 1000L;
	
	for (i = 0; i < IOSLOTS_COUNT; ++i)
	{
		if (ioslot_state[i].data.common.driver == DALLAS_TEMPERATURE_DRIVER)
		{
			if (i >= dallas_next_num)
			{
				uint32_t now = millis();
				uint32_t last_millis = ioslot_state[i].data.dallas_temperature.last_millis;
				
				if (last_millis + dallas_period > now
					|| now < last_millis)
				{
					OneWire  ds(ioslot_state[i].data.dallas_temperature.pin);
					byte data[2];
					ds.reset();
					ds.write(0xCC);
					ds.write(0xBE);
					data[0] = ds.read();
					data[1] = ds.read();

					int temperature = (data[1]<< 8) + data[0];
					
					ds.reset();
					ds.write(0xCC);
					ds.write(0x44);

					ioslot_state[i].data.dallas_temperature.last_millis = now;
					ioslot_state[i].data.dallas_temperature.value = (float)temperature * 0.0625f;
					
					dallas_requested = true;
					dallas_next_num = i + 1;
					break;
				}
			}
		}
	}
	
	if (!dallas_requested)
	{
		dallas_next_num = 0;
	}
}

static bool prepare_empty_slot(struct abstract_ioslot_state *state, struct abstract_ioslot *ioslot)
{	
	state->execute = NULL;
	state->io_discrete = NULL;
	state->io_analog = NULL;
	
	state->data.common.driver = EMPTY_SLOT_DRIVER;
	state->data.common.id = 0;
	
	return true;
}

typedef bool (*prep_ioslot_fn)(struct abstract_ioslot_state *, struct abstract_ioslot *);

static const prep_ioslot_fn  prepare_ioslot[] = {
	prepare_analog_input,
	prepare_discrete_input,
	prepare_discrete_output,
	prepare_dhtxx,
	prepare_dallas_temperature,
	
	prepare_empty_slot
};

static const uint8_t prepare_ioslot_count = sizeof(prepare_ioslot) / sizeof(prepare_ioslot[0]);

static void io_prepare(void)
{
	if (need_prepare)
	{
		uint8_t i;	
		
		for (i = 0; i < IOSLOTS_COUNT; ++i)
		{
			struct abstract_ioslot ioslot;
			uint8_t j;
			
			read_ioslot(i, &ioslot);
			
			for (j = 0; j < prepare_ioslot_count; ++j)
			{
				if (prepare_ioslot[j](&ioslot_state[i], &ioslot))
					break;
			}
		}
		
		need_prepare = false;
	}
}

void io_execute_in(void)
{	
	io_lock();
	io_prepare();
	{
		uint8_t i;
	
		fill_adc_values();
	
		for (i = 0; i < IOSLOTS_COUNT; ++i)
		{
			struct abstract_ioslot ioslot;
			read_ioslot(i, &ioslot);
		
			if (ioslot_state[i].execute)
				ioslot_state[i].execute(&ioslot_state[i], &ioslot, IN);
		}
	
		get_dhtxx_values();
		get_dallas_values();
	}
	io_unlock();
}

void io_execute_out(void)
{
	io_lock();
	{
		uint8_t i;
		for (i = 0; i < IOSLOTS_COUNT; ++i)
		{
			struct abstract_ioslot ioslot;
			read_ioslot(i, &ioslot);
			
			if (ioslot_state[i].execute)
				ioslot_state[i].execute(&ioslot_state[i], &ioslot, OUT);
		}
	}
	io_unlock();
}

void ioslot_state_by_id(uint8_t id, struct abstract_ioslot_state **state)
{
	uint8_t i;

	for (i = 0; i < IOSLOTS_COUNT; ++i)
	{
		if (ioslot_state[i].data.common.id == id)
		{
			*state = &ioslot_state[i];
			return;
		}
	}
	
	*state = 0;
}

uint8_t input_discrete(uint8_t id, int *err)
{
	struct abstract_ioslot_state *state;
	uint8_t value = UNKNOWN_VALUE;
	
	ioslot_state_by_id(id, &state);
	if (!state)
	{
		*err = 1;
		return value;
	}
	
	if (state->io_discrete)
		state->io_discrete(state, IN, &value);
	
	if (value == UNKNOWN_VALUE)
		*err = 1;
	else
		*err = 0;
	
	return value;
}

float input_analog(uint8_t id, int *err)
{
	struct abstract_ioslot_state *state;
	float value = NAN;
	
	ioslot_state_by_id(id, &state);
	if (!state)
	{
		*err = 1;
		return value;
	}
	
	if (state->io_analog)
		state->io_analog(state, IN, &value);
	
	if (isnan(value))
		*err = 1;
	else
		*err = 0;
	
	return value;
}

void output_discrete(uint8_t id, uint8_t value, int *err)
{
	struct abstract_ioslot_state *state;
	
	ioslot_state_by_id(id, &state);
	if (!state)
	{
		*err = 1;
		return;
	}
	
	if (state->io_discrete)
		state->io_discrete(state, OUT, &value);
}

uint16_t get_adc_value(uint8_t channel)
{
	return adc_value[channel];
}

float get_ioslot_value(uint8_t num)
{
	float value = NAN;
	struct abstract_ioslot_state *state = &ioslot_state[num];

	if (state->io_analog)
		state->io_analog(state, IN, &value);

	return value;
}
