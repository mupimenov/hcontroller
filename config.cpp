#include "config.h"

#include <string.h>

#include <EEPROM.h>

#include "scmRTOS_Arduino.h"

static OS::TMutex 		mutex;

void config_init(void)
{
	EEPROM.begin();
}

void config_lock(void)
{
	mutex.lock();
}

void config_unlock(void)
{
	mutex.unlock();
}

void config_read(uint16_t address, uint8_t *data, uint16_t size)
{
	uint16_t i;
	for (i = 0; i < size; ++i)
	{
		data[i] = EEPROM.read(address + i);
	}
}

void config_write(uint16_t address, const uint8_t *data, uint16_t size)
{
	uint16_t i;
	for (i = 0; i < size; ++i)
	{
		EEPROM.write(address + i, data[i]);
	}
}

static float array_to_float(const uint8_t *arr, int offset)
{
	uint32_t v = (uint32_t)arr[offset]
		| ((uint32_t)arr[offset + 1] << 8)
		| ((uint32_t)arr[offset + 2] << 16)
		| ((uint32_t)arr[offset + 3] << 24);
	float f;
	memcpy(&f, &v, sizeof(f));
	return f;
}

static bool analog_input_parser(uint8_t *arr, struct abstract_ioslot *ioslot)
{
	if (arr[0] != ANALOG_INPUT_DRIVER)
		return false;
	
	ioslot->data.analog_input.driver = arr[0];
	ioslot->data.analog_input.id = arr[1];
	ioslot->data.analog_input.num = arr[2];
	ioslot->data.analog_input.x1 = (uint16_t)arr[3] | ((uint32_t)arr[4] << 8);
	ioslot->data.analog_input.x2 = (uint16_t)arr[5] | ((uint32_t)arr[6] << 8);
	ioslot->data.analog_input.y1 = array_to_float(arr, 7);
	ioslot->data.analog_input.y2 = array_to_float(arr, 11);
	
	return true;
}

static bool discrete_input_parser(uint8_t *arr, struct abstract_ioslot *ioslot)
{
	if (arr[0] != DISCRETE_INPUT_DRIVER)
		return false;
	
	ioslot->data.discrete_input.driver = arr[0];
	ioslot->data.discrete_input.id = arr[1];
	ioslot->data.discrete_input.pin = arr[2];
	ioslot->data.discrete_input.inverse = (arr[3] == 0x01);
	
	return true;
}

static bool discrete_output_parser(uint8_t *arr, struct abstract_ioslot *ioslot)
{
	if (arr[0] != DISCRETE_OUTPUT_DRIVER)
		return false;
	
	ioslot->data.discrete_output.driver = arr[0];
	ioslot->data.discrete_output.id = arr[1];
	ioslot->data.discrete_output.operation = arr[2];
	ioslot->data.discrete_output.pin = arr[3];
	ioslot->data.discrete_output.inverse = (arr[4] == 0x01);
		
	return true;
}

static bool dhtxx_parser(uint8_t *arr, struct abstract_ioslot *ioslot)
{
	if (arr[0] != DHTxx_DRIVER)
		return false;

	ioslot->data.dhtxx.driver = arr[0];
	ioslot->data.dhtxx.id = arr[1];

	ioslot->data.dhtxx.modification = arr[2];
	ioslot->data.dhtxx.parameter = arr[3];
	ioslot->data.dhtxx.pin = arr[4];
		
	return true;
}

static bool dallas_temperature_parser(uint8_t *arr, struct abstract_ioslot *ioslot)
{
	if (arr[0] != DALLAS_TEMPERATURE_DRIVER)
		return false;
	
	ioslot->data.dallas_temperature.driver = arr[0];
	ioslot->data.dallas_temperature.id = arr[1];
	ioslot->data.dallas_temperature.pin = arr[2];
		
	return true;
}

static bool empty_slot_parser(uint8_t *arr, struct abstract_ioslot *ioslot)
{	
	ioslot->data.common.driver = EMPTY_SLOT_DRIVER;
	ioslot->data.common.id = 0;
		
	return true;
}

typedef bool (*ioslot_parser_fn)(uint8_t *, struct abstract_ioslot *);

static const ioslot_parser_fn ioslot_parser[] = {
	analog_input_parser,
	discrete_input_parser,
	discrete_output_parser,
	dhtxx_parser,
	dallas_temperature_parser,
	empty_slot_parser
};

static const uint8_t ioslot_parser_count = sizeof(ioslot_parser) / sizeof(ioslot_parser[0]);

void read_ioslot(uint8_t num, struct abstract_ioslot *ioslot)
{
	uint16_t address = IOSLOT_START_ADDRESS + num * IOSLOT_SIZE;
	uint8_t i;
	uint8_t arr[IOSLOT_SIZE];
	
	config_read(address, arr, IOSLOT_SIZE);

	for (i = 0; i < ioslot_parser_count; ++i)
	{
		if (ioslot_parser[i](arr, ioslot))
			break;
	}
}

void read_ioslot_by_id(uint8_t id, struct abstract_ioslot *ioslot)
{
	uint8_t i;
	
	for (i = 0; i < IOSLOTS_COUNT; ++i)
	{
		read_ioslot(i, ioslot);
		if (ioslot->data.common.id == id)
			return;
	}
	
	empty_slot_parser(0, ioslot);
}

static void array_to_datetime(const uint8_t *arr, int offset, struct datetime *dt)
{
	dt->seconds = arr[offset];
	dt->minutes = arr[offset + 1];
	dt->hours = arr[offset + 2];
	dt->day = arr[offset + 3];
	dt->month = arr[offset + 4];
	dt->year = arr[offset + 5];
}

static void array_to_cyclogram(const uint8_t *arr, int offset, struct cyclogram_params *cyclogram)
{
	cyclogram->type = arr[offset];
	cyclogram->count = (uint16_t)arr[offset + 1] | ((uint16_t)arr[offset + 2] << 8);
	cyclogram->impulse_duration = (uint16_t)arr[offset + 3] | ((uint16_t)arr[offset + 4] << 8);
	cyclogram->pause_duration = (uint16_t)arr[offset + 5] | ((uint16_t)arr[offset + 6] << 8);
}

static uint8_t timer_control_output(struct abstract_program *program)
{
	return program->data.timer.output;
}

static bool timer_control_parser(uint8_t *arr, struct abstract_program *program)
{	
	if (arr[0] != TIMER_CONTROL_PROGRAM)
		return false;
	
	program->output = timer_control_output;
	
	program->data.timer.type = arr[0];
	program->data.timer.id = arr[1];
	
	program->data.timer.constrains = arr[2];
	array_to_datetime(arr, 3, &program->data.timer.from);
	array_to_datetime(arr, 9, &program->data.timer.to);
	array_to_cyclogram(arr, 15, &program->data.timer.cyclogram);
	program->data.timer.output = arr[22];
	
	return true;
}

static uint8_t relay_control_output(struct abstract_program *program)
{
	return program->data.relay.output;
}

static bool relay_control_parser(uint8_t *arr, struct abstract_program *program)
{	
	if (arr[0] != RELAY_CONTROL_PROGRAM)
		return false;
	
	program->output = relay_control_output;
	
	program->data.relay.type = arr[0];
	program->data.relay.id = arr[1];
	
	program->data.relay.input = arr[2];
	program->data.relay.constrains = arr[3];
	array_to_datetime(arr, 4, &program->data.relay.from);
	array_to_datetime(arr, 10, &program->data.relay.to);
	program->data.relay.low_bound = array_to_float(arr, 16);
	program->data.relay.high_bound = array_to_float(arr, 20);
	array_to_cyclogram(arr, 24, &program->data.relay.cyclogram);
	program->data.relay.inverse = arr[31] == 0x01? 1: 0;
	program->data.relay.output = arr[32];
	
	return true;
}

static uint8_t pid_control_output(struct abstract_program *program)
{
	return program->data.pid.output;
}

static bool pid_control_parser(uint8_t *arr, struct abstract_program *program)
{	
	if (arr[0] != PID_CONTROL_PROGRAM)
		return false;
	
	program->output = pid_control_output;
	
	program->data.pid.type = arr[0];
	program->data.pid.id = arr[1];
	
	program->data.pid.input = arr[2];
	program->data.pid.constrains = arr[3];
	array_to_datetime(arr, 4, &program->data.pid.from);
	array_to_datetime(arr, 10, &program->data.pid.to);
	program->data.pid.desired = array_to_float(arr, 16);
	program->data.pid.proportional_gain = array_to_float(arr, 20);
	program->data.pid.integral_gain = array_to_float(arr, 24);
	program->data.pid.differential_gain = array_to_float(arr, 28);
	program->data.pid.inverse = arr[32] == 0x01? 1: 0;
	program->data.pid.output = arr[33];
	
	return true;
}

static uint8_t button_control_output(struct abstract_program *program)
{
	return program->data.button.output;
}

static bool button_control_parser(uint8_t *arr, struct abstract_program *program)
{
	if (arr[0] != BUTTON_CONTROL_PROGRAM)
		return false;

	program->output = button_control_output;

	program->data.button.type = arr[0];
	program->data.button.id = arr[1];

	program->data.button.input = arr[2];
	program->data.button.output = arr[3];

	return true;
}

static uint8_t empty_program_output(struct abstract_program *program)
{
	return 0;
}

static bool empty_program_parser(uint8_t *arr, struct abstract_program *program)
{
	program->output = empty_program_output;
	
	program->data.common.type = EMPTY_PROGRAM;
	program->data.common.id = 0;
	
	return true;
}

typedef bool (*prog_parser)(uint8_t *, struct abstract_program *);

static const prog_parser program_parser[] = {
	timer_control_parser,
	relay_control_parser,
	pid_control_parser,
	button_control_parser,

	empty_program_parser
};

static const uint8_t program_parser_count = sizeof(program_parser) / sizeof(program_parser[0]);


void read_program(uint8_t num, struct abstract_program *program)
{
	uint16_t address = PROGRAM_START_ADDRESS + num * PROGRAM_SIZE;
	uint8_t i;
	uint8_t arr[PROGRAM_SIZE];

	config_read(address, arr, PROGRAM_SIZE);
	
	for (i = 0; i < program_parser_count; ++i)
	{
		if (program_parser[i](arr, program))
			return;
	}
}

uint8_t program_count_with_output(uint8_t id)
{
	uint8_t count = 0;
	uint8_t i;
	
	struct abstract_program program;
	
	for (i = 0; i < PROGRAMS_COUNT; ++i)
	{
		read_program(i, &program);
		if (program.output(&program) == id)
			++count;
	}
	
	return count;
}
